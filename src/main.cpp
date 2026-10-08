// Receptor ESP32: TFT horizontal + MAX30102 + Bluetooth Serial

#include <Arduino.h>
#include "BluetoothSerial.h"
#include <TFT_eSPI.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <esp_gap_bt_api.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth no habilitado. Revisar sdkconfig / menuconfig.
#endif

#define MOTOR_PIN   23

// 1 = horizontal, 3 = horizontal invertida
#define ROTACION    1

// Pines I2C del MAX30102
#define I2C_SDA     21
#define I2C_SCL     22

// Colores RGB565
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

// Umbrales de fases de sueño
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

BluetoothSerial SerialBT;
TFT_eSPI tft = TFT_eSPI();
MAX30105 particleSensor;

volatile bool estadoConectado  = false;
bool oximetroActivo   = true;
bool alarmaActiva     = false;
bool configuracionConfirmada = false;

// Vibración de bienvenida (se dispara en el callback, se ejecuta en el loop)
volatile bool pulsoMotorPendiente = false;
unsigned long tFinPulsoMotor = 0;
bool pulsoMotorActivo = false;

volatile uint8_t pantallaActual = 0;

int32_t bpmActual  = 0;
int32_t spo2Actual = 0;
bool    lecturaValida = false;
int32_t bpmAnterior = 0;

// Apagado diferido de pantalla
unsigned long tApagadoPantalla = 0;
bool pantallaDebeApagarse = false;

// Layout (se calcula según el tamaño real de la pantalla)
int16_t  W = 160, H = 128;
uint8_t  TAM_NUM   = 4;
uint8_t  TAM_TOTAL = 3;
const int16_t HEADER_H = 16;
int16_t  Y_ICON  = 42;
int16_t  Y_NUM   = 60;
int16_t  Y_LABEL = 98;

// Últimos valores dibujados
int32_t bpmMostrado  = -1;
int32_t spo2Mostrado = -1;

void calcularLayout();
void texto(int16_t x, int16_t y, const String &s, uint8_t size, uint16_t color, uint8_t datum);
void dibujarPantalla1BT();
void dibujarPantalla2Monitoreo();
void dibujarPantalla3Resumen();
void dibujarPantalla4Apagado();
void actualizarValoresMonitoreo();
void mostrarBPM(int32_t v);
void mostrarSpO2(int32_t v);
void dibujarValorGrande(int16_t cx, const char *txt);
void registrarMuestra();
FaseSueno clasificarFase(int32_t bpm, int32_t variabilidad);
void calcularResumen(uint16_t &ligero, uint16_t &profundo, uint16_t &rem, uint16_t &total);
void borrarArrayMuestras();
void dibujarCorazon(int16_t x, int16_t y, uint16_t color, uint8_t escala);
void dibujarGotita(int16_t x, int16_t y, uint16_t color);
void dibujarIconoBT(int16_t cx, int16_t cy, uint16_t color, uint8_t r);
String formatearTiempo(uint16_t muestras);

// Toma el tamaño real de la pantalla y ajusta posiciones
void calcularLayout() {
  W = tft.width();
  H = tft.height();
  TAM_NUM   = (W >= 160) ? 4 : 3;
  TAM_TOTAL = (W >= 160) ? 3 : 2;
  Y_ICON  = HEADER_H + 26;
  Y_NUM   = HEADER_H + 44;
  Y_LABEL = Y_NUM + 8 * TAM_NUM + 6;
}

// Texto alineado según el datum
void texto(int16_t x, int16_t y, const String &s, uint8_t size, uint16_t color, uint8_t datum) {
  tft.setTextDatum(datum);
  tft.setTextSize(size);
  tft.setTextColor(color);
  tft.drawString(s, x, y);
}

// Callback BT: sin delay() ni TFT acá
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT || event == ESP_SPP_OPEN_EVT) {
    bool exitoso = (event == ESP_SPP_SRV_OPEN_EVT)
                     ? (param->srv_open.status == ESP_SPP_SUCCESS)
                     : (param->open.status == ESP_SPP_SUCCESS);
    if (!exitoso) return;

    estadoConectado = true;
    oximetroActivo  = true;
    pantallaDebeApagarse = false;
    Serial.println("[BT] Emisor conectado exitosamente.");

    pulsoMotorPendiente = true;
    pantallaActual = 0;

  } else if (event == ESP_SPP_CLOSE_EVT) {
    estadoConectado = false;
    pantallaDebeApagarse = false;
    configuracionConfirmada = false;
    Serial.println("[BT] Desconectado.");
    pantallaActual = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // Pantalla
  tft.init();
  tft.setRotation(ROTACION);
  calcularLayout();
  tft.fillScreen(C_BG);
  texto(W / 2, H / 2, "Iniciando TFT...", 1, C_WHITE, MC_DATUM);

  // Oxímetro
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    tft.fillScreen(C_BG);
    texto(W / 2, H / 2, "ERROR MAX30102", 1, C_RED, MC_DATUM);
    Serial.println("[ERROR] MAX30102 no encontrado.");
  } else {
    particleSensor.setup(0x1F, 4, 2, 400, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x3F);
    particleSensor.setPulseAmplitudeIR(0x3F);
    Serial.println("[OXI] Oximetro iniciado.");
  }

  delay(200);

  // Bluetooth esclavo: solo el emisor inicia la conexión
  SerialBT.register_callback(btCallback);
  if (SerialBT.begin("ESP32_Receptor")) {
    Serial.println("[BT] Receptor listo.");
  }

  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

  borrarArrayMuestras();
}

void loop() {
  // Vibración de bienvenida no bloqueante
  if (pulsoMotorPendiente) {
    pulsoMotorPendiente = false;
    digitalWrite(MOTOR_PIN, HIGH);
    tFinPulsoMotor   = millis() + 400;
    pulsoMotorActivo = true;
  }
  if (pulsoMotorActivo && millis() >= tFinPulsoMotor && !alarmaActiva) {
    digitalWrite(MOTOR_PIN, LOW);
    pulsoMotorActivo = false;
  }

  // Comandos del emisor
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
      dibujarPantalla4Apagado();

      tApagadoPantalla = millis() + 3000;
      pantallaDebeApagarse = true;

    } else if (c == '3') {
      configuracionConfirmada = true;
      Serial.println("[CONFIG] Alarma confirmada en el emisor. Habilitando mediciones.");
    }
  }

  // Apagado diferido de pantalla
  if (pantallaDebeApagarse && millis() >= tApagadoPantalla) {
    tft.fillScreen(C_BG);
    pantallaActual = 5;
    pantallaDebeApagarse = false;
    Serial.println("[DISPLAY] Pantalla en negro.");
  }

  // Selección de pantalla
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

  // Medición y registro de muestras
  if (estadoConectado && oximetroActivo && configuracionConfirmada && pantallaActual == 2) {
    actualizarValoresMonitoreo();

    if (millis() - tUltimaMuestra >= INTERVALO_MUESTRA_MS && lecturaValida) {
      registrarMuestra();
      tUltimaMuestra = millis();
    }
  }

  delay(20);
}

// Pantalla 1: Bluetooth desconectado
void dibujarPantalla1BT() {
  tft.fillScreen(C_BG);

  int16_t colTexto = (2 * W) / 3;
  int16_t cy = H / 2 - 6;

  dibujarIconoBT(W / 6, cy, C_BTBLUE, 18);
  texto(colTexto, cy - 20, "BT",      3, C_RED, MC_DATUM);
  texto(colTexto, cy + 12, "DESCON.", 2, C_RED, MC_DATUM);

  texto(W / 2, H - 30, "Esperando emisor...", 1, C_GRAY, MC_DATUM);
}

// Pantalla 2: monitoreo en dos columnas (LPM | SpO2)
void dibujarPantalla2Monitoreo() {
  tft.fillScreen(C_BG);

  // Barra superior
  tft.fillRect(0, 0, W, HEADER_H, C_DARKGRAY);
  dibujarIconoBT(7, HEADER_H / 2, C_CYAN, 5);
  texto(18, HEADER_H / 2, "BT", 1, C_CYAN, ML_DATUM);

  if (!configuracionConfirmada) {
    texto(34, HEADER_H / 2, "CONF", 1, C_ORANGE, ML_DATUM);
  } else if (!oximetroActivo) {
    texto(34, HEADER_H / 2, "OFF", 1, C_ORANGE, ML_DATUM);
  }

  // Separadores
  tft.drawFastHLine(0, HEADER_H, W, C_GRAY);
  tft.drawFastVLine(W / 2, HEADER_H + 6, H - HEADER_H - 12, C_DARKGRAY);

  // Columna izquierda: LPM
  dibujarCorazon(W / 4, Y_ICON, C_RED, TAM_NUM);
  texto(W / 4, Y_LABEL, "LPM", 1, C_GRAY, TC_DATUM);

  // Columna derecha: SpO2
  dibujarGotita((3 * W) / 4, Y_ICON - 2, C_CYAN);
  texto((3 * W) / 4, Y_LABEL, "SpO2", 1, C_GRAY, TC_DATUM);

  // Fuerza redibujado de valores
  bpmMostrado  = -1;
  spo2Mostrado = -1;
  mostrarBPM((lecturaValida && bpmActual > 0) ? bpmActual : 0);
  mostrarSpO2((lecturaValida && spo2Actual > 0) ? spo2Actual : 0);
}

// Pantalla 3: resumen del sueño
void dibujarPantalla3Resumen() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, W, HEADER_H, C_DARKGRAY);
  texto(W / 2, HEADER_H / 2, "RESUMEN DEL SUENO", 1, C_WHITE, MC_DATUM);

  uint16_t mLigero, mProfundo, mREM, mTotal;
  calcularResumen(mLigero, mProfundo, mREM, mTotal);

  const int16_t BAR_X  = 56;
  const int16_t BAR_H  = 10;
  const int16_t barMax = W - BAR_X - 44;

  struct Fila { const char *nombre; uint16_t color; uint16_t muestras; int16_t y; };
  Fila filas[3] = {
    { "Ligero",   C_GREEN,  mLigero,   28 },
    { "Profundo", C_BLUE,   mProfundo, 50 },
    { "REM",      C_ORANGE, mREM,      72 },
  };

  for (int i = 0; i < 3; i++) {
    uint16_t w = (mTotal > 0) ? (uint16_t)((long)filas[i].muestras * barMax / mTotal) : 0;
    texto(4, filas[i].y + BAR_H / 2, filas[i].nombre, 1, filas[i].color, ML_DATUM);
    tft.fillRect(BAR_X, filas[i].y, barMax, BAR_H, C_DARKGRAY);
    tft.fillRect(BAR_X, filas[i].y, w > 0 ? w : 2, BAR_H, filas[i].color);
    texto(W - 4, filas[i].y + BAR_H / 2, formatearTiempo(filas[i].muestras), 1, C_WHITE, MR_DATUM);
  }

  tft.drawFastHLine(0, 94, W, C_GRAY);
  texto(4, 111, "Total", 1, C_GRAY, ML_DATUM);
  texto(W - 4, 111, formatearTiempo(mTotal), TAM_TOTAL, C_WHITE, MR_DATUM);

  borrarArrayMuestras();
}

// Pantalla 4: pulsómetro apagado
void dibujarPantalla4Apagado() {
  tft.fillScreen(C_BG);
  texto(W / 2, H / 2 - 12, "PULSOMETRO", 2, C_RED, MC_DATUM);
  texto(W / 2, H / 2 + 12, "APAGADO",    2, C_RED, MC_DATUM);
}

// Número grande centrado en su columna
void dibujarValorGrande(int16_t cx, const char *txt) {
  int16_t colW = W / 2;
  tft.fillRect(cx - colW / 2 + 3, Y_NUM, colW - 6, 8 * TAM_NUM, C_BG);
  texto(cx, Y_NUM, txt, TAM_NUM, C_WHITE, TC_DATUM);
}

void mostrarBPM(int32_t v) {
  if (v == bpmMostrado) return;
  bpmMostrado = v;
  char buf[8];
  if (v > 0) snprintf(buf, sizeof(buf), "%d", (int)v);
  else       strcpy(buf, "---");
  dibujarValorGrande(W / 4, buf);
}

void mostrarSpO2(int32_t v) {
  if (v == spo2Mostrado) return;
  spo2Mostrado = v;
  char buf[8];
  if (v > 0) snprintf(buf, sizeof(buf), "%d%%", (int)v);
  else       strcpy(buf, "--%");
  dibujarValorGrande((3 * W) / 4, buf);
}

// Lee el sensor, calcula BPM y SpO2 y actualiza la pantalla
void actualizarValoresMonitoreo() {
  static const byte TASA_PROMEDIO = 4;
  static byte tasas[TASA_PROMEDIO];
  static byte indice = 0;
  static byte muestrasValidas = 0;
  static long totalTasas = 0;
  static long ultimoLatido = 0;

  long valor = particleSensor.getIR();

  // Sin dedo: reinicia el promedio
  if (valor < 50000) {
    lecturaValida = false;
    muestrasValidas = 0;
    indice = 0;
    totalTasas = 0;
    ultimoLatido = 0;
    bpmActual = 0;

    mostrarBPM(0);
    mostrarSpO2(0);
    return;
  }

  // BPM: promedio de los últimos latidos
  if (checkForBeat(valor)) {
    long delta = millis() - ultimoLatido;
    ultimoLatido = millis();
    float beatsPerMinute = 60.0f / (delta / 1000.0f);
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      totalTasas -= tasas[indice];
      tasas[indice] = (byte)beatsPerMinute;
      totalTasas += tasas[indice];
      indice = (indice + 1) % TASA_PROMEDIO;

      if (muestrasValidas < TASA_PROMEDIO) muestrasValidas++;
      bpmActual = totalTasas / muestrasValidas;
    }
  }

  // SpO2 estimada por relación rojo/infrarrojo
  long rojo = particleSensor.getRed();
  if (rojo > 0 && valor > 0) {
    float ratio = (float)rojo / (float)valor;
    float spo2Calc = 110.0f - 25.0f * ratio;
    spo2Calc = constrain(spo2Calc, 85.0f, 100.0f);
    spo2Actual = (int32_t)spo2Calc; lecturaValida = true;
  }

  mostrarBPM(bpmActual);
  mostrarSpO2(spo2Actual);
}

// Guarda la fase de sueño de la muestra actual
void registrarMuestra() {
  if (indiceMuestra >= MAX_MUESTRAS)
    return;
  int32_t variabilidad = abs(bpmActual - bpmAnterior);
  FaseSueno fase = clasificarFase(bpmActual, variabilidad);
  muestrasFase[indiceMuestra++] = (uint8_t)fase; bpmAnterior = bpmActual;
}

// Clasifica la fase según BPM y variabilidad
FaseSueno clasificarFase(int32_t bpm, int32_t variabilidad) {
  if (bpm <= 0)
    return FASE_NINGUNA;
  if (variabilidad >= UMBRAL_VAR_ALTO && bpm >= PULSO_REM_MIN)
    return FASE_REM;
  if (bpm >= PULSO_PROF_MIN && bpm <= PULSO_PROF_MAX && variabilidad < UMBRAL_VAR_BAJO)
    return FASE_PROFUNDO;
  return FASE_LIGERO;
}

// Cuenta muestras por fase
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

// Cada muestra son 30 s; devuelve "XhYYm"
String formatearTiempo(uint16_t muestras) {
  uint32_t seg = (uint32_t)muestras * 30; char buf[10];
  snprintf(buf, sizeof(buf), "%dh%02dm", (int)(seg / 3600), (int)((seg % 3600) / 60));
  return String(buf);
}

// Corazón pixelado 5x4
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