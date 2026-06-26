/*
 * RECEPTOR — ESP32
 * Pantalla ST7735 128x128 + Sensor MAX30102 + Bluetooth Serial
 *
 * Pantalla 1: Estado BT (gatekeeper — bloquea muestreo si no hay conexión)
 * Pantalla 2: Monitoreo en vivo (corazón + LPM, gotitas + SpO2)
 * Pantalla 3: Resumen de fases de sueño (al sonar alarma), luego borra array
 */

#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Wire.h>             // Requerido para manejar los pines I2C de forma limpia
#include "MAX30105.h"         // SparkFun MAX3010x library
#include "heartRate.h"        // SparkFun helper para BPM

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth no habilitado. Revisar sdkconfig / menuconfig.
#endif

// ─────────────────────────────────────────────
//  PINES DEFINIDOS Y CORREGIDOS
// ─────────────────────────────────────────────
#define MOTOR_PIN   13 

// ST7735 Nativos / Asignados
#define TFT_CS      5
#define TFT_RST     4
#define TFT_DC      2
// Nota física: MOSI va al GPIO 23, SCK va al GPIO 18 (por defecto en ESP32 SPI)

// MAX30102 Pines I2C Nativos de Hardware
#define I2C_SDA     21
#define I2C_SCL     22

// ─────────────────────────────────────────────
//  COLORES (RGB565)
// ─────────────────────────────────────────────
#define C_BG        0x0000   // Negro
#define C_WHITE     0xFFFF
#define C_RED       0xF800
#define C_BLUE      0x001F
#define C_CYAN      0x07FF
#define C_GREEN     0x07E0
#define C_ORANGE    0xFC60
#define C_GRAY      0x8410
#define C_DARKGRAY  0x4208
#define C_BTBLUE    0x035F   // Azul Bluetooth

// ─────────────────────────────────────────────
//  UMBRALES DE CLASIFICACIÓN DE FASES
// ─────────────────────────────────────────────
#define PULSO_PROF_MIN    40
#define PULSO_PROF_MAX    60
#define PULSO_LIG_MIN     55
#define PULSO_LIG_MAX     75
#define PULSO_REM_MIN     60

#define UMBRAL_VAR_BAJO   5    // Diferencia abs BPM entre muestras → "estable"
#define UMBRAL_VAR_ALTO   10   // Diferencia abs BPM entre muestras → "fluctuante" (REM)

// ─────────────────────────────────────────────
//  MUESTRAS DE SUEÑO
// ─────────────────────────────────────────────
#define MAX_MUESTRAS  960
#define INTERVALO_MUESTRA_MS  30000UL  // 30 segundos

enum FaseSueno { FASE_NINGUNA = 0, FASE_LIGERO, FASE_PROFUNDO, FASE_REM };

uint8_t  muestrasFase[MAX_MUESTRAS];   // almacena FaseSueno por muestra
uint16_t indiceMuestra = 0;
uint32_t tUltimaMuestra = 0;

// ─────────────────────────────────────────────
//  ESTADO GLOBAL
// ─────────────────────────────────────────────
BluetoothSerial SerialBT;
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
MAX30105 particleSensor;

volatile bool estadoConectado  = false;
bool oximetroActivo   = true;   
bool alarmaActiva     = false;  

uint8_t pantallaActual = 0;  

int32_t bpmActual  = 0;
int32_t spo2Actual = 0;
bool    lecturaValida = false;

int32_t bpmAnterior = 0;

// ─────────────────────────────────────────────
//  PROTOTIPOS DE FUNCIONES
// ─────────────────────────────────────────────
void dibujarPantalla1BT();
void dibujarPantalla2Monitoreo();
void dibujarPantalla3Resumen();
void actualizarValoresMonitoreo();
void registrarMuestra();
FaseSueno clasificarFase(int32_t bpm, int32_t variabilidad);
void calcularResumen(uint16_t &ligero, uint16_t &profundo, uint16_t &rem, uint16_t &total);
void borrarArrayMuestras();
void dibujarCorazon(int16_t x, int16_t y, uint16_t color, uint8_t escala);
void dibujarGotita(int16_t x, int16_t y, uint16_t color);
void dibujarIconoBT(int16_t cx, int16_t cy, uint16_t color, uint8_t r);
String formatearTiempo(uint16_t muestras);

// ─────────────────────────────────────────────
//  CALLBACK BLUETOOTH
// ─────────────────────────────────────────────
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    estadoConectado = true;
    oximetroActivo  = true;   
    Serial.println("[BT] Emisor conectado exitosamente.");
    
    // Alerta por vibración corta al enlazar
    digitalWrite(MOTOR_PIN, HIGH);
    delay(400);
    digitalWrite(MOTOR_PIN, LOW);
    
    pantallaActual = 0; // Fuerza el refresco de pantalla en el loop

  } else if (event == ESP_SPP_CLOSE_EVT) {
    estadoConectado = false;
    Serial.println("[BT] Emisor desconectado.");
    pantallaActual = 0; // Fuerza el refresco a pantalla de desconexión
  }
}

// ─────────────────────────────────────────────
//  SETUP (Reorganizado por estabilidad eléctrica)
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // ── 1. Inicializar Pantalla TFT Primero ──
  tft.initR(INITR_144GREENTAB);   
  tft.setRotation(0);
  tft.fillScreen(C_BG);

  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 55);
  tft.println("Iniciando TFT...");

  // ── 2. Inicializar Oxímetro ──
  Wire.begin(I2C_SDA, I2C_SCL, I2C_SPEED_FAST);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    tft.fillScreen(C_BG);
    tft.setTextColor(C_RED);
    tft.setTextSize(1);
    tft.setCursor(4, 50);
    tft.println("ERROR MAX30102");
    tft.setCursor(4, 65);
    tft.println("Verificar I2C");
    Serial.println("[ERROR] MAX30102 no encontrado en pines 21 y 22.");
  } else {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("[OXI] Oxímetro iniciado correctamente en Pines 21 y 22.");
  }

  // Pequeña pausa para estabilizar el bus físico antes de encender radiofrecuencia (RF)
  delay(200); 

  // ── 3. Inicializar Bluetooth al Final ──
  SerialBT.register_callback(btCallback);
  if (SerialBT.begin("ESP32_Receptor")) {
    Serial.println("[BT] Receptor listo y transmitiendo en el aire.");
  } else {
    Serial.println("[BT] ERROR crítico al iniciar el Bluetooth.");
  }

  borrarArrayMuestras();
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
void loop() {
  // Atender los mensajes entrantes del Emisor
  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print("[BT] Recibido: "); Serial.println(c);

    if (c == '1') {
      alarmaActiva   = true;
      oximetroActivo = true;   
      digitalWrite(MOTOR_PIN, HIGH);
      dibujarPantalla3Resumen();
      pantallaActual = 3;

    } else if (c == '0') {
      alarmaActiva = false;
      digitalWrite(MOTOR_PIN, LOW);
      pantallaActual = 0;   

    } else if (c == '2') {
      oximetroActivo = false;
      Serial.println("[OXI] Muestreo desactivado por combo de botones.");
    }
  }

  // Máquina de estados visual inteligente (Evita parpadeos recurrentes)
  if (!estadoConectado) {
    if (pantallaActual != 1) {
      dibujarPantalla1BT();
      pantallaActual = 1;
    }
  } else {
    // Si está conectado, pero no estamos en la pantalla de monitoreo ni de resumen, vamos a monitoreo
    if (pantallaActual != 2 && pantallaActual != 3) {
      dibujarPantalla2Monitoreo();
      pantallaActual = 2;
    }
  }

  // Lógica de muestreo activo en segundo plano
  if (estadoConectado && oximetroActivo && pantallaActual == 2) {
    actualizarValoresMonitoreo();

    if (millis() - tUltimaMuestra >= INTERVALO_MUESTRA_MS && lecturaValida) {
      registrarMuestra();
      tUltimaMuestra = millis();
    }
  }

  delay(20);
}

// ─────────────────────────────────────────────
//  FUNCIONES GRÁFICAS Y DE PROCESAMIENTO
// ─────────────────────────────────────────────
void dibujarPantalla1BT() { 
  tft.fillScreen(C_BG); 
  dibujarIconoBT(64, 44, C_BTBLUE, 20); 
  tft.setTextSize(2); 
  tft.setTextColor(C_RED); 
  tft.setCursor(6, 72); 
  tft.println("BT DESCON."); 
  tft.setTextSize(1); 
  tft.setTextColor(C_GRAY); 
  tft.setCursor(14, 96); 
  tft.println("Esperando emisor..."); 
  tft.setCursor(8, 110); 
  tft.println("Sin registro de datos"); 
}

void dibujarPantalla2Monitoreo() { 
  tft.fillScreen(C_BG); 
  tft.fillRect(0, 0, 128, 14, C_DARKGRAY);
  dibujarIconoBT(6, 7, C_CYAN, 4);
  tft.setTextSize(1); 
  tft.setTextColor(C_CYAN); 
  tft.setCursor(16, 3); 
  tft.print("BT OK"); 
  
  if (!oximetroActivo) { 
    tft.setTextColor(C_ORANGE); 
    tft.setCursor(60, 3); 
    tft.print("OXI OFF"); 
  } 
  
  tft.drawFastHLine(0, 14, 128, C_GRAY); 
  tft.drawFastHLine(0, 70, 128, C_GRAY); 
  dibujarCorazon(54, 30, C_RED, 3); 
  tft.setTextSize(3); 
  tft.setTextColor(C_WHITE); 
  tft.setCursor(36, 44); 
  
  if (lecturaValida && bpmActual > 0) tft.printf("%3d", (int)bpmActual); 
  else tft.print("---"); 
  
  tft.setTextSize(1); 
  tft.setTextColor(C_GRAY); 
  tft.setCursor(52, 62); tft.print("LPM"); 
  
  dibujarGotita(34, 92, C_CYAN); 
  dibujarGotita(56, 92, C_CYAN); 
  dibujarGotita(78, 92, C_CYAN); 
  tft.setTextSize(3); 
  tft.setTextColor(C_WHITE); 
  tft.setCursor(28, 106); 
  
  if (lecturaValida && spo2Actual > 0) tft.printf("%2d%%", (int)spo2Actual); 
  else tft.print("--%"); 
  
  tft.setTextSize(1); 
  tft.setTextColor(C_GRAY); 
  tft.setCursor(46, 122); 
  tft.print("SpO2"); 
}

void dibujarPantalla3Resumen() { 
  tft.fillScreen(C_BG); 
  tft.fillRect(0, 0, 128, 16, C_DARKGRAY); 
  tft.setTextSize(1); 
  tft.setTextColor(C_WHITE); 
  tft.setCursor(12, 4); 
  tft.print("RESUMEN DEL SUENO"); 
  
  uint16_t mLigero, mProfundo, mREM, mTotal; 
  calcularResumen(mLigero, mProfundo, mREM, mTotal); 
  String sLig = formatearTiempo(mLigero); 
  String sProf = formatearTiempo(mProfundo); 
  String sREM = formatearTiempo(mREM); 
  String sTot = formatearTiempo(mTotal); 
  
  uint16_t barMax = 80; 
  
  uint16_t wLig = (mTotal > 0) ? (uint16_t)((long)mLigero * barMax / mTotal) : 0; 
  tft.setTextSize(1);
  tft.setTextColor(C_GREEN); 
  tft.setCursor(2, 26); 
  tft.print("Ligero"); 
  tft.fillRect(44, 24, wLig > 0 ? wLig : 2, 9, C_GREEN); 
  tft.setTextColor(C_WHITE); 
  tft.setCursor(126 - 6*(int)sLig.length(), 26); 
  tft.print(sLig); 
  
  uint16_t wProf = (mTotal > 0) ? (uint16_t)((long)mProfundo * barMax / mTotal) : 0; 
  tft.setTextColor(C_BLUE); 
  tft.setCursor(2, 46); 
  tft.print("Profundo"); 
  tft.fillRect(44, 44, wProf > 0 ? wProf : 2, 9, C_BLUE); 
  tft.setTextColor(C_WHITE); 
  tft.setCursor(126 - 6*(int)sProf.length(), 46); 
  tft.print(sProf); 
  
  uint16_t wREM = (mTotal > 0) ? (uint16_t)((long)mREM * barMax / mTotal) : 0;
  tft.setTextColor(C_ORANGE);
  tft.setCursor(2, 66); 
  tft.print("REM"); 
  tft.fillRect(44, 64, wREM > 0 ? wREM : 2, 9, C_ORANGE); 
  tft.setTextColor(C_WHITE); 
  tft.setCursor(126 - 6*(int)sREM.length(), 66); tft.print(sREM); 
  
  tft.drawFastHLine(0, 84, 128, C_GRAY); 
  tft.setTextSize(1); 
  tft.setTextColor(C_GRAY); 
  tft.setCursor(2, 90); 
  tft.print("Total:"); 
  tft.setTextColor(C_WHITE); 
  tft.setTextSize(2); 
  tft.setCursor(46, 88); 
  tft.print(sTot); 
  
  tft.setTextSize(1); 
  tft.setTextColor(C_GRAY); 
  tft.setCursor(28, 112); 
  tft.print("Datos borrados"); 
  tft.fillTriangle(60, 124, 68, 124, 64, 128, C_GRAY); 
  
  delay(200); 
  borrarArrayMuestras(); 
  Serial.println("[SUENO] Array de muestras borrado."); 
}

void actualizarValoresMonitoreo() { 
  static const byte TASA_PROMEDIO = 4; 
  static byte tasas[TASA_PROMEDIO]; 
  static byte indice = 0; 
  static long totalTasas = 0; 
  static long ultimoLatido = 0; 
  
  long valor = particleSensor.getIR(); 
  
  if (valor < 50000) { 
    lecturaValida = false; 
    tft.setTextSize(3); 
    tft.setTextColor(C_WHITE); 
    tft.fillRect(36, 44, 60, 22, C_BG); 
    tft.setCursor(36, 44); 
    tft.print("---"); 
    tft.fillRect(28, 106, 72, 22, C_BG); 
    tft.setCursor(28, 106); 
    tft.print("--%"); 
    return; 
  } 
  
  if (checkForBeat(valor)) { 
    long delta = millis() - ultimoLatido; 
    ultimoLatido = millis(); 
    float beatsPerMinute = 60.0f / (delta / 1000.0f); 
    if (beatsPerMinute < 255 && beatsPerMinute > 20) { 
      totalTasas -= tasas[indice]; 
      tasas[indice] = (byte)beatsPerMinute; 
      totalTasas += tasas[indice]; 
      indice = (indice + 1) % TASA_PROMEDIO; 
      bpmActual = totalTasas / TASA_PROMEDIO; 
    } 
  } 
  
  long rojo = particleSensor.getRed(); 
  if (rojo > 0 && valor > 0) { 
    float ratio = (float)rojo / (float)valor; 
    float spo2Calc = 110.0f - 25.0f * ratio; 
    spo2Calc = constrain(spo2Calc, 85.0f, 100.0f); 
    spo2Actual = (int32_t)spo2Calc; 
    lecturaValida = true; 
  } 
  
  static int32_t bpmAnteriorPantalla = -1; 
  static int32_t spo2AnteriorPantalla = -1; 
  
  if (bpmActual != bpmAnteriorPantalla) { 
    tft.setTextSize(3); 
    tft.setTextColor(C_WHITE); 
    tft.fillRect(28, 44, 80, 22, C_BG); 
    tft.setCursor(36, 44); 
    if (bpmActual > 0) tft.printf("%3d", (int)bpmActual); 
    else tft.print("---"); 
    bpmAnteriorPantalla = bpmActual; 
  } 
  
  if (spo2Actual != spo2AnteriorPantalla) { 
    tft.setTextSize(3); 
    tft.setTextColor(C_WHITE); 
    tft.fillRect(20, 106, 88, 22, C_BG); 
    tft.setCursor(28, 106); 
    if (spo2Actual > 0) tft.printf("%2d%%", (int)spo2Actual); 
    else tft.print("--%"); 
    spo2AnteriorPantalla = spo2Actual; 
  } 
}

void registrarMuestra() { 
  if (indiceMuestra >= MAX_MUESTRAS) return; 
  int32_t variabilidad = abs(bpmActual - bpmAnterior); 
  FaseSueno fase = clasificarFase(bpmActual, variabilidad); 
  muestrasFase[indiceMuestra++] = (uint8_t)fase; 
  bpmAnterior = bpmActual; 
  Serial.printf("[MUESTRA %d] BPM=%d Var=%d Fase=%d\n", indiceMuestra, (int)bpmActual, (int)variabilidad, (int)fase); 
}

FaseSueno clasificarFase(int32_t bpm, int32_t variabilidad) { 
  if (bpm <= 0) return FASE_NINGUNA; 
  if (variabilidad >= UMBRAL_VAR_ALTO && bpm >= PULSO_REM_MIN) return FASE_REM; 
  if (bpm >= PULSO_PROF_MIN && bpm <= PULSO_PROF_MAX && variabilidad < UMBRAL_VAR_BAJO) return FASE_PROFUNDO; 
  if (bpm >= PULSO_LIG_MIN && bpm <= PULSO_LIG_MAX) return FASE_LIGERO; 
  if (bpm >= 60 && bpm <= 75) { 
    return (variabilidad >= UMBRAL_VAR_ALTO) ? FASE_REM : FASE_LIGERO; 
  } 
  return FASE_LIGERO; 
}

void calcularResumen(uint16_t &ligero, uint16_t &profundo, uint16_t &rem, uint16_t &total) { 
  ligero = profundo = rem = 0; 
  for (uint16_t i = 0; i < indiceMuestra; i++) { 
    switch (muestrasFase[i]) { 
      case FASE_LIGERO: ligero++; break; 
      case FASE_PROFUNDO: profundo++; break; 
      case FASE_REM: rem++; break; 
      default: break; 
    } 
  } 
  total = ligero + profundo + rem; 
}

void borrarArrayMuestras() { 
  memset(muestrasFase, 0, sizeof(muestrasFase)); 
  indiceMuestra = 0; 
  bpmAnterior = 0; 
  tUltimaMuestra = millis(); 
}

String formatearTiempo(uint16_t muestras) { 
  uint32_t segundos = (uint32_t)muestras * 30; 
  uint16_t horas = segundos / 3600; 
  uint16_t minutos = (segundos % 3600) / 60; 
  char buf[8]; 
  snprintf(buf, sizeof(buf), "%dh%02dm", horas, minutos); 
  return String(buf); 
}

void dibujarCorazon(int16_t x, int16_t y, uint16_t color, uint8_t escala) { 
  static const uint8_t corazon[4][5] = { 
    {0,1,0,1,0}, 
    {1,1,1,1,1}, 
    {0,1,1,1,0}, 
    {0,0,1,0,0}, 
  }; 
  int16_t ox = x - (5 * escala) / 2; 
  int16_t oy = y - (4 * escala) / 2; 
  for (int fy = 0; fy < 4; fy++) { 
    for (int fx = 0; fx < 5; fx++) { 
      if (corazon[fy][fx]) { 
        tft.fillRect(ox + fx * escala, oy + fy * escala, escala, escala, color); 
      } 
    } 
  } 
}

void dibujarGotita(int16_t x, int16_t y, uint16_t color) { 
  tft.fillCircle(x, y + 4, 5, color); 
  tft.fillTriangle(x, y - 5, x - 4, y + 2, x + 4, y + 2, color); 
}

void dibujarIconoBT(int16_t cx, int16_t cy, uint16_t color, uint8_t r) { 
  tft.drawFastVLine(cx, cy - r, 2 * r, color); 
  tft.drawLine(cx, cy - r, cx + r, cy - r/2, color); 
  tft.drawLine(cx + r, cy - r/2, cx, cy, color); 
  tft.drawLine(cx, cy, cx + r, cy + r/2, color); 
  tft.drawLine(cx + r, cy + r/2, cx, cy + r, color); 
}