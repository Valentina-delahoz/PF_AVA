#include <Arduino.h>
#include <ESP32Servo.h>

// =============================================================================
// DEFINICIÓN DE PINES
// =============================================================================

// --- Serial2 (comunicación con RPi) ---
#define RXD2                16
#define TXD2                17

// --- Sensor ultrasónico izquierdo (HC-SR04) ---
#define US_LEFT_TRIG        14
#define US_LEFT_ECHO        27

// --- Sensor ultrasónico derecho (HC-SR04) ---
#define US_RIGHT_TRIG       13
#define US_RIGHT_ECHO       12

// --- Driver L298N ---
#define MOTOR_IN1           32
#define MOTOR_IN2           33
#define MOTOR_ENA           25      // Pin PWM

// --- Servo MG995 ---
#define SERVO_PIN           26      // Pin PWM

// =============================================================================
// PARÁMETROS DEL SISTEMA
// =============================================================================

// Temporización
#define BOOT_DELAY_MS           5000   // Delay inicial para que arranque la RPi (90s)
#define SAMPLE_INTERVAL_MS      100     // Frecuencia de muestreo (ms)
#define SAFETY_TIMEOUT_MS       500     // Timeout watchdog (ms)

// Offset físico sensor izquierdo (está 0.5cm más adelante que el derecho)
#define US_LEFT_OFFSET_CM       0.5f

// PWM Motor (canal 0)
#define PWM_CHANNEL_MOTOR       1
#define PWM_FREQ_MOTOR          1000    // Hz
#define PWM_RESOLUTION_MOTOR    8       // bits → rango 0-255

// Servo MG995 — manejado por librería ESP32Servo
#define SERVO_MIN_US            500     // Pulso mínimo (µs)
#define SERVO_MAX_US            2500    // Pulso máximo (µs)
#define SERVO_CENTER_DEG        30      // Posición central mecánica real (grados)
#define SERVO_MAX_LEFT          0       // Límite izquierdo (grados)
#define SERVO_MAX_RIGHT         60      // Límite derecho (grados)

// Protocolo ASCII
#define FRAME_START             '<'
#define FRAME_END               '>'
#define FRAME_SEP               ','
#define MAX_FRAME_LEN           64

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================

// Parser serial
String  rx_buffer       = "";
bool    frame_started   = false;

// Objeto servo
Servo servo_mg995;

// Control
uint8_t motor_duty      = 0;        // Duty cycle motor (0-255)
float   servo_angular   = 0.0f;     // Ángulo servo (rad/s normalizado)

// Temporización
unsigned long last_sample_time  = 0;
unsigned long last_cmd_time     = 0;

// Estado de seguridad
bool emergency_stop = false;

// =============================================================================
// PROTOTIPOS DE FUNCIONES
// =============================================================================

void    init_hardware();
void    read_serial();
void    parse_frame(String frame);
float   read_ultrasonic(uint8_t trig, uint8_t echo);
void    set_motor_duty(uint8_t duty);
void    set_servo_angle(float angular_z);
void    send_telemetry();
void    send_heartbeat();
void    execute_emergency_stop();
void    safety_check();

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);   // Monitor serie para depuración (USB)
  init_hardware();

  // ---------------------------------------------------------------------------
  // DELAY INICIAL — Espera a que la RPi arranque completamente
  // ---------------------------------------------------------------------------
  // Durante este tiempo NO se transmite nada por Serial2, evitando que la RPi
  // reciba datos por su RX antes de completar el arranque del sistema.
  // El monitor USB (Serial) sí está disponible para verificar el progreso.
  // ---------------------------------------------------------------------------
  Serial.println("[ESP32] Iniciando delay de arranque (90s)...");
  Serial.println("[ESP32] Esperando que la RPi arranque completamente.");

  unsigned long boot_start = millis();
  while (millis() - boot_start < BOOT_DELAY_MS) {
    // Log de progreso cada 10 segundos por USB
    unsigned long elapsed = (millis() - boot_start) / 1000;
    Serial.print("[ESP32] Tiempo transcurrido: ");
    Serial.print(elapsed);
    Serial.print("s / ");
    Serial.print(BOOT_DELAY_MS / 1000);
    Serial.println("s");
    delay(10000);
  }

  // Inicializar timestamps después del delay
  last_sample_time = millis();
  last_cmd_time    = millis();

  Serial.println("[ESP32] Delay de arranque completado.");
  Serial.println("[ESP32] Sistema listo. Esperando comandos desde RPi...");
}

// =============================================================================
// LOOP PRINCIPAL
// =============================================================================

void loop() {
  unsigned long now = millis();

  // 1. Leer y parsear bytes entrantes desde la RPi
  read_serial();

  // 2. Verificar watchdog de seguridad
  safety_check();

  // 3. Ciclo de muestreo cada 100ms
  if (now - last_sample_time >= SAMPLE_INTERVAL_MS) {
    last_sample_time = now;

    // Leer sensores y enviar telemetría a la RPi
    send_telemetry();

    // Enviar heartbeat
    send_heartbeat();
  }
}

// =============================================================================
// INICIALIZACIÓN DE HARDWARE
// =============================================================================

void init_hardware() {
  // Serial2 — comunicación con RPi
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Sensores ultrasónicos
  pinMode(US_LEFT_TRIG,   OUTPUT);
  pinMode(US_LEFT_ECHO,   INPUT);
  pinMode(US_RIGHT_TRIG,  OUTPUT);
  pinMode(US_RIGHT_ECHO,  INPUT);

  // Motor L298N — dirección fija hacia adelante
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, HIGH);   // Dirección fija: adelante
  digitalWrite(MOTOR_IN2, LOW);    // Dirección fija: adelante

  // PWM Motor — API actualizada (reemplaza ledcSetup + ledcAttachPin)
  ledcAttachChannel(MOTOR_ENA, PWM_FREQ_MOTOR, PWM_RESOLUTION_MOTOR, PWM_CHANNEL_MOTOR);
  ledcWriteChannel(PWM_CHANNEL_MOTOR, 0); // Motor detenido al inicio

  // Servo MG995 — manejado por ESP32Servo
  servo_mg995.setPeriodHertz(50);                        // 50Hz estándar
  servo_mg995.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  set_servo_angle(0.0f);           // Servo centrado al inicio
}

// =============================================================================
// LECTURA Y PARSING SERIAL
// =============================================================================

void read_serial() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();

    if (c == FRAME_START) {
      // Inicio de trama — resetear buffer
      rx_buffer     = "";
      frame_started = true;
    }
    else if (c == FRAME_END && frame_started) {
      // Fin de trama — procesar
      parse_frame(rx_buffer);
      rx_buffer     = "";
      frame_started = false;
    }
    else if (frame_started) {
      // Acumular caracteres dentro de la trama
      rx_buffer += c;

      // Protección contra desbordamiento de buffer
      if (rx_buffer.length() > MAX_FRAME_LEN) {
        rx_buffer     = "";
        frame_started = false;
        Serial.println("[WARN] Buffer desbordado, trama descartada.");
      }
    }
  }
}

void parse_frame(String frame) {
  // Formato esperado: "CMD,{duty},{angular}"
  // Ejemplo:          "CMD,180,0.10"

  // Verificar tipo de trama
  if (!frame.startsWith("CMD")) {
    Serial.println("[WARN] Trama desconocida: " + frame);
    return;
  }

  // Extraer campos por índice de comas
  int sep1 = frame.indexOf(FRAME_SEP);         // Entre CMD y duty
  int sep2 = frame.indexOf(FRAME_SEP, sep1+1); // Entre duty y angular

  if (sep1 == -1 || sep2 == -1) {
    Serial.println("[WARN] Trama CMD malformada: " + frame);
    return;
  }

  // Parsear valores
  uint8_t duty    = (uint8_t) frame.substring(sep1+1, sep2).toInt();
  float   angular = frame.substring(sep2+1).toFloat();

  // Actualizar timestamp watchdog
  last_cmd_time = millis();
  emergency_stop = false;

  // Aplicar comandos
  motor_duty    = duty;
  servo_angular = angular;

  set_motor_duty(motor_duty);
  set_servo_angle(servo_angular);

  // Log de depuración
  Serial.print("[CMD] duty=");
  Serial.print(duty);
  Serial.print(" angular=");
  Serial.println(angular, 3);
}

// =============================================================================
// SENSOR ULTRASÓNICO
// =============================================================================

float read_ultrasonic(uint8_t trig_pin, uint8_t echo_pin) {
  // Generar pulso trigger de 10µs
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // Medir duración del echo (timeout 30ms → ~500cm máximo)
  long duration = pulseIn(echo_pin, HIGH, 30000);

  if (duration == 0) return -1.0f; // Sin respuesta → fuera de rango

  // Convertir a centímetros
  return (duration * 0.0343f) / 2.0f;
}

// =============================================================================
// CONTROL DE MOTOR
// =============================================================================

void set_motor_duty(uint8_t duty) {
  ledcWriteChannel(PWM_CHANNEL_MOTOR, duty);
}

// =============================================================================
// CONTROL DE SERVO
// =============================================================================

void set_servo_angle(float angular_z) {
  // Mapear angular_z normalizado (-1.0 a 1.0) al rango mecánico real
  // Centro: 30°  |  Izq. máx: 0°  |  Der. máx: 60°
  float angle_deg = SERVO_CENTER_DEG + (angular_z * 30.0f);

  // Saturación con límites mecánicos reales
  if (angle_deg > SERVO_MAX_RIGHT) angle_deg = SERVO_MAX_RIGHT;  // 60°
  if (angle_deg <  SERVO_MAX_LEFT) angle_deg = SERVO_MAX_LEFT;   //  0°

  // ESP32Servo escribe directamente en grados
  servo_mg995.write((int) angle_deg);
}

// =============================================================================
// ENVÍO DE TELEMETRÍA
// =============================================================================

void send_telemetry() {
  // Leer sensores con compensación de offset físico
  float dist_izq = read_ultrasonic(US_LEFT_TRIG,  US_LEFT_ECHO);
  float dist_der = read_ultrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO);

  // Aplicar offset al sensor izquierdo (está 0.5cm más adelante)
  if (dist_izq > 0) dist_izq += US_LEFT_OFFSET_CM;

  // Construir y enviar trama TLM
  String frame = "<TLM," + String(dist_izq, 2) + "," + String(dist_der, 2) + ">";
  Serial2.println(frame);

  // Log de depuración
  Serial.println("[TLM] " + frame);
}

// =============================================================================
// ENVÍO DE HEARTBEAT
// =============================================================================

void send_heartbeat() {
  Serial2.println("<HBT,OK>");
}

// =============================================================================
// PROTOCOLO DE SEGURIDAD — WATCHDOG
// =============================================================================

void safety_check() {
  if (millis() - last_cmd_time > SAFETY_TIMEOUT_MS) {
    if (!emergency_stop) {
      execute_emergency_stop();
    }
  }
}

void execute_emergency_stop() {
  emergency_stop = true;
  motor_duty     = 0;
  servo_angular  = 0.0f;

  // Detener motor
  set_motor_duty(0);

  // Centrar servo
  set_servo_angle(0.0f);

  // Notificar a la RPi
  Serial2.println("<EMG,STOP>");
  Serial.println("[EMG] Parada de emergencia ejecutada.");
}
