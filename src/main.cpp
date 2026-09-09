/*
 * RECEPTOR — ESP32
 * Pantalla ST7735 128x128 + Sensor MAX30102 + Bluetooth Serial + Lectura Batería (Sin Pin TP4056)
 */

#include <Arduino.h>
#include "BluetoothSerial.h"
#include <TFT_eSPI.h>         // Bus SPI por hardware (config de pines en platformio.ini)
#include <Wire.h>             
#include "MAX30105.h"         
#include "heartRate.h"        
#include <Preferences.h>      // Para guardar la última MAC conectada en NVS (memoria no volátil)

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth no habilitado. Revisar sdkconfig / menuconfig.
#endif

#define MOTOR_PIN   23 

// Pines ST7789 (deben coincidir con los definidos en platformio.ini via build_flags)
#define TFT_CS      5
#define TFT_RST     4
#define TFT_DC      2

// MAX30102 Pines
#define I2C_SDA     21
#define I2C_SCL     22

// CONFIGURACIÓN DIVISOR DE TENSIÓN
#define PIN_BAT_ADC 34  // GPIO 34 para el divisor de tensión

const float R1 = 10000.0; // 10k Ohm
const float R2 = 10000.0; // 10k Ohm
const float FACTOR_DIVISOR = (R1 + R2) / R2; // Factor x2.0

// COLORES (RGB565)
#define C_BG        0x0000   
#define C_WHITE     0xFFFF
#define C_RED       0xF800
#define C_BLUE      0x001F
#define C_CYAN      0x07FF
#define C_GREEN     0x07E0
#define C_ORANGE    0xFC60
#define C_GRAY      0x8410
#define C_DARKGRAY  0x4208
#define C_BTBLUE    0x035F   
#define C_YELLOW    0xFFE0

#define PULSO_PROF_MIN    40
#define PULSO_PROF_MAX    60
#define PULSO_LIG_MIN     55
#define PULSO_LIG_MAX     75
#define PULSO_REM_MIN     60
#define UMBRAL_VAR_BAJO   5    
#define UMBRAL_VAR_ALTO   10   

#define MAX_MUESTRAS  960
#define INTERVALO_MUESTRA_MS  30000UL  

enum FaseSueno { FASE_NINGUNA = 0, FASE_LIGERO, FASE_PROFUNDO, FASE_REM };

uint8_t  muestrasFase[MAX_MUESTRAS];   
uint16_t indiceMuestra = 0;
uint32_t tUltimaMuestra = 0;

// ESTADO GLOBAL
BluetoothSerial SerialBT;
TFT_eSPI tft = TFT_eSPI();   
MAX30105 particleSensor;

volatile bool estadoConectado  = false;
bool oximetroActivo   = true;   
bool alarmaActiva     = false;   

// Portón de medición
bool configuracionConfirmada = false;

// RECONEXIÓN BT AUTOMÁTICA
Preferences prefs;
uint8_t  direccionGuardada[6] = {0};
volatile bool hayDireccionGuardada  = false;
volatile bool debeGuardarDireccion  = false;   
TaskHandle_t tareaReconexionHandle  = NULL;

uint8_t pantallaActual = 0;  

int32_t bpmActual  = 0;
int32_t spo2Actual = 0;
bool    lecturaValida = false;
int32_t bpmAnterior = 0;

// Control del apagado diferido de pantalla en Receptor
unsigned long tApagadoPantalla = 0;
bool pantallaDebeApagarse = false;

// Control de refresco y variables globales de batería
unsigned long tUltimaLecturaBat = 0;
int porcentajeBatActual = -1;
bool cargandoActual = false;

// PROTOTIPOS
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
void tareaReconexionBT(void *parametro);

// FUNCIONES BATERÍA Y DETECCIÓN DE CARGA POR SOFTWARE
// FUNCIONES BATERÍA CON LÍMITES AJUSTADOS (3.3V - 4.2V)
float obtenerVoltajeBateria() {
  long suma = 0;
  for (int i = 0; i < 30; i++) { // Promediado para eliminar ruido analógico
    suma += analogRead(PIN_BAT_ADC);
    delayMicroseconds(200);
  }
  float adcPromedio = suma / 30.0;
  
  // Para ADC_11db el voltaje de referencia práctico en el ESP32 es ~3.15V
  float voltajePin = (adcPromedio / 4095.0) * 3.15f; 
  return voltajePin * FACTOR_DIVISOR; // FACTOR_DIVISOR = 2.0 (10k / 10k)
}

int calcularPorcentaje(float voltaje) {
  // 3.30V = 0%  --->  4.20V = 100%
  float V_MIN = 3.30f;
  float V_MAX = 4.20f;
  
  int pct = (int)((voltaje - V_MIN) / (V_MAX - V_MIN) * 100.0f);
  return constrain(pct, 0, 100);
}

bool detectarCarga(float voltaje) {
  // El TP4056 mantiene la línea a ~4.20V - 4.22V cuando está cargando una celda llena
  return (voltaje >= 4.21f);
}

void actualizarEstadoBateria() {
  if (millis() - tUltimaLecturaBat > 500 || porcentajeBatActual == -1) {
    tUltimaLecturaBat = millis();
    
    float vBat = obtenerVoltajeBateria();
    int nuevoPct = calcularPorcentaje(vBat);
    bool nuevaCarga = detectarCarga(vBat);

    // Muestra lecturas en el Monitor Serie para verificación con la fuente
    Serial.printf("[BAT] Volts leídos: %.2fV | Porcentaje: %d%%\n", vBat, nuevoPct);

    // Refrescar en pantalla si varía el porcentaje o el estado de carga
    if (nuevoPct != porcentajeBatActual || nuevaCarga != cargandoActual) {
      porcentajeBatActual = nuevoPct;
      cargandoActual = nuevaCarga;

      // Pantalla 2 (Monitoreo)
      if (pantallaActual == 2) {
        tft.fillRect(80, 0, 48, 14, C_DARKGRAY);
        tft.setTextSize(1);
        tft.setTextColor(cargandoActual ? C_GREEN : C_WHITE);
        tft.setCursor(82, 3);
        tft.printf("%d%%", porcentajeBatActual);
        if (cargandoActual) {
          tft.setTextColor(C_YELLOW);
          tft.print("+");
        }
      } 
      // Pantalla 1 (Sin conexión BT)
      else if (pantallaActual == 1) {
        tft.fillRect(30, 105, 90, 20, C_BG);
        tft.setTextColor(cargandoActual ? C_GREEN : C_WHITE);
        tft.setCursor(34, 108);
        tft.printf("BAT: %d%% %s", porcentajeBatActual, cargandoActual ? "[+]" : "");
      }
    }
  }
}

// CALLBACK BLUETOOTH
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT || event == ESP_SPP_OPEN_EVT) {
    bool exitoso = (event == ESP_SPP_SRV_OPEN_EVT)
                     ? (param->srv_open.status == ESP_SPP_SUCCESS)
                     : (param->open.status == ESP_SPP_SUCCESS);
    if (!exitoso) return;

    estadoConectado = true;
    oximetroActivo  = true;   
    pantallaDebeApagarse = false; 
    Serial.println(event == ESP_SPP_SRV_OPEN_EVT
                      ? "[BT] Emisor conectado exitosamente (entrante)."
                      : "[BT] Reconexión saliente exitosa.");

    const uint8_t *mac = (event == ESP_SPP_SRV_OPEN_EVT) ? param->srv_open.rem_bda
                                                          : param->open.rem_bda;
    memcpy(direccionGuardada, mac, 6);
    hayDireccionGuardada = true;
    debeGuardarDireccion = true;

    digitalWrite(MOTOR_PIN, HIGH);
    delay(400);
    digitalWrite(MOTOR_PIN, LOW);
    
    pantallaActual = 0; 

  } else if (event == ESP_SPP_CLOSE_EVT) {
    estadoConectado = false;
    pantallaDebeApagarse = false;
    configuracionConfirmada = false; 
    Serial.println("[BT] Desconectado.");
    pantallaActual = 0; 
  }
}

// TAREA DE RECONEXIÓN AUTOMÁTICA
void tareaReconexionBT(void *parametro) {
  const uint32_t INTERVALO_REINTENTO_MS = 4000;

  for (;;) {
    if (!estadoConectado && hayDireccionGuardada) {
      Serial.println("[BT] Sin conexión. Intentando reconectar al último dispositivo conocido...");
      bool ok = SerialBT.connect(direccionGuardada);
      if (ok) {
        Serial.println("[BT] connect() saliente devolvió éxito.");
      } else {
        Serial.println("[BT] connect() saliente falló, se reintentará.");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(INTERVALO_REINTENTO_MS));
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // Configuración del pin analógico de la batería (GPIO 34 es ADC1_CH6)
  analogSetAttenuation(ADC_11db); // Rango de hasta ~3.3V

  // 1. Pantalla TFT Primero
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(C_BG);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 55);
  tft.println("Iniciando TFT...");

  // 2. Oxímetro
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    tft.fillScreen(C_BG);
    tft.setTextColor(C_RED);
    tft.setTextSize(1);
    tft.setCursor(4, 50);
    tft.println("ERROR MAX30102");
    Serial.println("[ERROR] MAX30102 no encontrado.");
  } else {
    particleSensor.setup(0x1F, 4, 2, 400, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x3F);
    particleSensor.setPulseAmplitudeIR(0x3F);
    Serial.println("[OXI] Oxímetro iniciado.");
  }

  delay(200); 

  // 3. Bluetooth al final
  SerialBT.register_callback(btCallback);
  if (SerialBT.begin("ESP32_Receptor")) {
    Serial.println("[BT] Receptor listo.");
  }

  // Cargar la última dirección MAC conectada desde NVS
  prefs.begin("btrecept", false);
  size_t leidos = prefs.getBytes("lastAddr", direccionGuardada, 6);
  hayDireccionGuardada = (leidos == 6);
  if (hayDireccionGuardada) {
    Serial.printf("[BT] Última dirección conocida cargada: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  direccionGuardada[0], direccionGuardada[1], direccionGuardada[2],
                  direccionGuardada[3], direccionGuardada[4], direccionGuardada[5]);
  } else {
    Serial.println("[BT] No hay ninguna dirección guardada todavía.");
  }

  xTaskCreatePinnedToCore(tareaReconexionBT, "ReconexionBT", 4096, NULL, 1,
                            &tareaReconexionHandle, 0);

  borrarArrayMuestras();
}

// LOOP
void loop() {
  if (debeGuardarDireccion) {
    debeGuardarDireccion = false;
    prefs.putBytes("lastAddr", direccionGuardada, 6);
    Serial.println("[BT] Dirección guardada en memoria no volátil.");
  }

  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print("[BT] Recibido: "); Serial.println(c);

    if (c == '1') {
      alarmaActiva   = true;
      oximetroActivo = true;   
      pantallaDebeApagarse = false; 
      digitalWrite(MOTOR_PIN, HIGH);
      dibujarPantalla3Resumen();
      pantallaActual = 3;

    } else if (c == '0') {
      alarmaActiva = false;
      digitalWrite(MOTOR_PIN, LOW);
      pantallaActual = 0;   

    } else if (c == '2') {
      oximetroActivo = false;
      pantallaActual = 4; 
      
      tft.fillScreen(C_BG);
      tft.setTextSize(1);
      tft.setTextColor(C_RED);
      tft.setCursor(12, 55);
      tft.println("PULSOMETRO APAGADO");
      
      tApagadoPantalla = millis() + 3000; 
      pantallaDebeApagarse = true;

    } else if (c == '3') {
      configuracionConfirmada = true;
      Serial.println("[CONFIG] Alarma confirmada en el emisor. Habilitando mediciones.");
    }
  }

  // Apagado diferido
  if (pantallaDebeApagarse && millis() >= tApagadoPantalla) {
    tft.fillScreen(C_BG); 
    pantallaActual = 5;   
    pantallaDebeApagarse = false;
    Serial.println("[DISPLAY] Pantalla en negro.");
  }

  // Máquina de estados visual
  if (!estadoConectado) {
    if (pantallaActual != 1) {
      dibujarPantalla1BT();
      pantallaActual = 1;
    }
  } else {
    if (pantallaActual != 2 && pantallaActual != 3 && pantallaActual != 4 && pantallaActual != 5) {
      dibujarPantalla2Monitoreo();
      pantallaActual = 2;
    }
  }

  // Refresco continuo de batería
  actualizarEstadoBateria();

  // Captura de datos
  if (estadoConectado && oximetroActivo && configuracionConfirmada && pantallaActual == 2) {
    actualizarValoresMonitoreo();

    if (millis() - tUltimaMuestra >= INTERVALO_MUESTRA_MS && lecturaValida) {
      registrarMuestra();
      tUltimaMuestra = millis();
    }
  }

  delay(20);
}

void dibujarPantalla1BT() { 
  tft.fillScreen(C_BG); 
  dibujarIconoBT(64, 38, C_BTBLUE, 16); 
  tft.setTextSize(2); tft.setTextColor(C_RED); tft.setCursor(6, 62); tft.println("BT DESCON."); 
  tft.setTextSize(1); tft.setTextColor(C_GRAY); tft.setCursor(14, 84); tft.println("Esperando emisor..."); 
  
  // Muestra estado de batería sin conexión BT
  float v = obtenerVoltajeBateria();
  bool cargando = detectarCarga(v);
  tft.setTextColor(cargando ? C_GREEN : C_WHITE);
  tft.setCursor(34, 108);
  tft.printf("BAT: %d%% %s", calcularPorcentaje(v), cargando ? "[+]" : "");
}

void dibujarPantalla2Monitoreo() { 
  tft.fillScreen(C_BG); 
  tft.fillRect(0, 0, 128, 14, C_DARKGRAY);
  dibujarIconoBT(6, 7, C_CYAN, 4); 
  tft.setTextSize(1); tft.setTextColor(C_CYAN); tft.setCursor(16, 3); tft.print("BT"); 
  
  if (!configuracionConfirmada) {
    tft.setTextColor(C_ORANGE); tft.setCursor(32, 3); tft.print("CONF..");
  } else if (!oximetroActivo) { 
    tft.setTextColor(C_ORANGE); tft.setCursor(32, 3); tft.print("OFF"); 
  } 

  // Estado inicial de batería
  float vBat = obtenerVoltajeBateria();
  porcentajeBatActual = calcularPorcentaje(vBat);
  cargandoActual = detectarCarga(vBat);
  
  tft.setTextColor(cargandoActual ? C_GREEN : C_WHITE);
  tft.setCursor(82, 3);
  tft.printf("%d%%", porcentajeBatActual);
  if (cargandoActual) {
    tft.setTextColor(C_YELLOW);
    tft.print("+");
  }

  tft.drawFastHLine(0, 14, 128, C_GRAY); tft.drawFastHLine(0, 70, 128, C_GRAY); 
  dibujarCorazon(54, 30, C_RED, 3); tft.setTextSize(3); tft.setTextColor(C_WHITE); tft.setCursor(36, 44); 
  if (lecturaValida && bpmActual > 0) 
    tft.printf("%3d", (int)bpmActual); 
  else 
    tft.print("---"); 
  
  tft.setTextSize(1); tft.setTextColor(C_GRAY); tft.setCursor(52, 62); tft.print("LPM"); 
  dibujarGotita(34, 92, C_CYAN); 
  dibujarGotita(56, 92, C_CYAN); 
  dibujarGotita(78, 92, C_CYAN); 
  tft.setTextSize(3); tft.setTextColor(C_WHITE); tft.setCursor(28, 106); 
  if (lecturaValida && spo2Actual > 0) 
    tft.printf("%2d%%", (int)spo2Actual); 
  else 
    tft.print("--%"); 
  tft.setTextSize(1); tft.setTextColor(C_GRAY); tft.setCursor(46, 122); tft.print("SpO2"); 
}

void dibujarPantalla3Resumen() { 
  tft.fillScreen(C_BG); tft.fillRect(0, 0, 128, 16, C_DARKGRAY); tft.setTextSize(1); tft.setTextColor(C_WHITE); tft.setCursor(12, 4); tft.print("RESUMEN DEL SUENO"); 
  uint16_t mLigero, mProfundo, mREM, mTotal; 
  calcularResumen(mLigero, mProfundo, mREM, mTotal); 
  String sLig = formatearTiempo(mLigero); 
  String sProf = formatearTiempo(mProfundo); 
  String sREM = formatearTiempo(mREM); 
  String sTot = formatearTiempo(mTotal); 
  uint16_t barMax = 80; 
  uint16_t wLig = (mTotal > 0) ? (uint16_t)((long)mLigero * barMax / mTotal) : 0; 
  tft.setTextColor(C_GREEN); tft.setCursor(2, 26); tft.print("Ligero"); tft.fillRect(44, 24, wLig > 0 ? wLig : 2, 9, C_GREEN); 
  tft.setTextColor(C_WHITE); tft.setCursor(126 - 6*(int)sLig.length(), 26); tft.print(sLig); 
  uint16_t wProf = (mTotal > 0) ? (uint16_t)((long)mProfundo * barMax / mTotal) : 0; 
  tft.setTextColor(C_BLUE); tft.setCursor(2, 46); tft.print("Profundo"); tft.fillRect(44, 44, wProf > 0 ? wProf : 2, 9, C_BLUE); 
  tft.setTextColor(C_WHITE); tft.setCursor(126 - 6*(int)sProf.length(), 46); tft.print(sProf); 
  uint16_t wREM = (mTotal > 0) ? (uint16_t)((long)mREM * barMax / mTotal) : 0;
  tft.setTextColor(C_ORANGE); tft.setCursor(2, 66); tft.print("REM"); tft.fillRect(44, 64, wREM > 0 ? wREM : 2, 9, C_ORANGE); 
  tft.setTextColor(C_WHITE); tft.setCursor(126 - 6*(int)sREM.length(), 66); tft.print(sREM); 
  tft.drawFastHLine(0, 84, 128, C_GRAY); tft.setTextSize(1); tft.setTextColor(C_GRAY); tft.setCursor(2, 90); tft.print("Total:"); 
  tft.setTextColor(C_WHITE); tft.setTextSize(2); tft.setCursor(46, 88); tft.print(sTot); 
  borrarArrayMuestras(); 
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
    tft.setCursor(36, 44); tft.print("---"); 
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
    spo2Actual = (int32_t)spo2Calc; lecturaValida = true; 
  }

  static int32_t bAnt = -1, sAnt = -1; 
  if (bpmActual != bAnt) {
    tft.setTextSize(3); 
    tft.setTextColor(C_WHITE); 
    tft.fillRect(28, 44, 80, 22, C_BG); 
    tft.setCursor(36, 44); 
    if (bpmActual > 0) 
      tft.printf("%3d", (int)bpmActual); 
    else tft.print("---"); 
    bAnt = bpmActual; 
  }

  if (spo2Actual != sAnt){ 
    tft.setTextSize(3); 
    tft.setTextColor(C_WHITE); 
    tft.fillRect(20, 106, 88, 22, C_BG); 
    tft.setCursor(28, 106); 
    if (spo2Actual > 0) 
      tft.printf("%2d%%", (int)spo2Actual); else tft.print("--%"); 
    sAnt = spo2Actual; 
  } 
}

void registrarMuestra() { 
  if (indiceMuestra >= MAX_MUESTRAS) 
    return; 
  int32_t variabilidad = abs(bpmActual - bpmAnterior); 
  FaseSueno fase = clasificarFase(bpmActual, variabilidad); 
  muestrasFase[indiceMuestra++] = (uint8_t)fase; bpmAnterior = bpmActual;
}

FaseSueno clasificarFase(int32_t bpm, int32_t variabilidad) {
  if (bpm <= 0) 
    return FASE_NINGUNA;
  if (variabilidad >= UMBRAL_VAR_ALTO && bpm >= PULSO_REM_MIN) 
    return FASE_REM; 
  if (bpm >= PULSO_PROF_MIN && bpm <= PULSO_PROF_MAX && variabilidad < UMBRAL_VAR_BAJO) 
    return FASE_PROFUNDO; 
  return FASE_LIGERO; 
}

void calcularResumen(uint16_t &l, uint16_t &p, uint16_t &r, uint16_t &t) {
  l = p = r = 0; 
  for (uint16_t i = 0; i < indiceMuestra; i++) { 
    if (muestrasFase[i] == FASE_LIGERO) 
      l++; 
    else if (muestrasFase[i] == FASE_PROFUNDO) 
      p++; 
    else if (muestrasFase[i] == FASE_REM) 
      r++;
  } 
  t = l + p + r;
}

void borrarArrayMuestras() { 
  memset(muestrasFase, 0, sizeof(muestrasFase));
  indiceMuestra = 0; bpmAnterior = 0;
  tUltimaMuestra = millis();
}

String formatearTiempo(uint16_t muestras) {
  uint32_t seg = (uint32_t)muestras * 30; char buf[8];
  snprintf(buf, sizeof(buf), "%dh%02dm", (int)(seg / 3600), (int)((seg % 3600) / 60));
  return String(buf);
}

void dibujarCorazon(int16_t x, int16_t y, uint16_t color, uint8_t escala) {
  static const uint8_t corazon[4][5] = { {0,1,0,1,0}, {1,1,1,1,1}, {0,1,1,1,0}, {0,0,1,0,0} };
  int16_t ox = x - (5 * escala) / 2;
  int16_t oy = y - (4 * escala) / 2;
  for (int fy = 0; fy < 4; fy++) { 
    for (int fx = 0; fx < 5; fx++) {
      if (corazon[fy][fx]) 
        tft.fillRect(ox + fx * escala, oy + fy * escala, escala, escala, color);
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