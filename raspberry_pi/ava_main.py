 =============================================================================
# AVA — Script integrado completo
# =============================================================================
# Percepción:  YOLO (5 clases) + Sensores ultrasónicos (vía ESP32)
# Control:     Comandos de duty cycle al ESP32 vía UART (Serial2 GPIO 16/17)
# Modo:        Headless (sin visualización gráfica, logs por terminal)
#
# Jerarquía de decisiones (de mayor a menor prioridad):
#   P1 — Seguridad inmediata (ultrasónicos)
#         Obstáculo <15cm → DETENIDO
#         Obstáculo <30cm → LENTO
#   P2 — Señalización obligatoria (YOLO)
#         red-light       → DETENIDO hasta ver green-light explícito
#         stop-sign       → DETENIDO 3s, luego reanuda
#         traffic-light   → DETENIDO (detención preventiva)
#   P3 — Interacción con entorno (YOLO)
#         vehicle         → LENTO
#   P4 — Estado por defecto
#         green-light o sin detecciones → NORMAL
#
# Estado inicial: DETENIDO durante 7 segundos antes de arrancar.
#
# Requisitos:
#   pip install ultralytics opencv-python pyserial
# =============================================================================

import cv2
import serial
import threading
import time
from ultralytics import YOLO

# =============================================================================
# CONFIGURACIÓN
# =============================================================================

# --- Modelo YOLO ---
MODEL_PATH              = "best_ncnn_model"
CONF_THRESHOLD          = 0.50
CAMERA_INDEX            = 0

# --- Filtro de tamaño de bounding box (detección temprana) ---
MIN_BBOX_AREA_RATIO     = 0.01      # 1% del frame total

# --- Comunicación serial con ESP32 ---
PUERTO                  = '/dev/ttyAMA0'
BAUDIOS                 = 115200

# --- Duty cycles por estado ---
DUTY_DETENIDO           = 0
DUTY_LENTO              = 120
DUTY_NORMAL             = 180
ANGULAR_RECTO           = 0.0

# --- Tiempos ---
INTERVALO_CMD_S         = 0.1       # Frecuencia de envío al ESP32 (10 Hz)
INTERVALO_LOG_S         = 2.0       # Frecuencia de log de estado
ARRANQUE_TIMEOUT_S      = 7.0       # Espera inicial antes de arrancar
DURACION_STOP_SIGN_S    = 3.0       # Detención por señal de stop

# --- Umbrales de ultrasónicos (cm) ---
US_UMBRAL_DETENER       = 15.0
US_UMBRAL_LENTO         = 30.0

# --- Índices de clases del modelo ---
CLS_GREEN_LIGHT         = 0
CLS_RED_LIGHT           = 1
CLS_STOP_SIGN           = 2
CLS_TRAFFIC_LIGHT       = 3
CLS_VEHICLE             = 4

NOMBRE_CLASES = {
    0: "green-light",
    1: "red-light",
    2: "stop-sign",
    3: "traffic-light",
    4: "vehicle",
}

# =============================================================================
# ESTADO GLOBAL DEL VEHÍCULO
# =============================================================================

class EstadoVehiculo:
    def __init__(self):
        # Comando actual
        self.duty                   = DUTY_DETENIDO
        self.angular                = ANGULAR_RECTO
        self.estado_nombre          = "ARRANQUE"

        # Lecturas de sensores (vienen del ESP32)
        self.us_izq                 = -1.0
        self.us_der                 = -1.0

        # Banderas de control
        self.fin_arranque_ts        = 0.0           # Timestamp de fin de arranque
        self.arrancado              = False         # Ya cumplió timeout inicial

        # Lógica de stop-sign (Opción A: una detención por aparición)
        self.detenido_stop_hasta    = 0.0
        self.stop_ya_procesada      = False

        # Lógica de red-light (regla estricta: necesita green-light explícito)
        self.bloqueado_por_red      = False

        # Concurrencia
        self.lock                   = threading.Lock()
        self.running                = True

estado = EstadoVehiculo()

# =============================================================================
# HILO DE CONTROL — Envía comandos al ESP32 a 10 Hz
# =============================================================================

def hilo_control(serial_port):
    print("[CONTROL] Hilo iniciado.")

    while estado.running:
        with estado.lock:
            duty_actual    = estado.duty
            angular_actual = estado.angular

        enviar_cmd(serial_port, duty_actual, angular_actual)
        time.sleep(INTERVALO_CMD_S)

    # Parada segura al terminar
    print("[CONTROL] Enviando parada segura...")
    fin = time.time() + 1.0
    while time.time() < fin:
        enviar_cmd(serial_port, 0, 0.0)
        time.sleep(INTERVALO_CMD_S)

    print("[CONTROL] Hilo detenido.")


def enviar_cmd(serial_port, duty, angular):
    duty    = max(0, min(255, int(duty)))
    angular = max(-1.0, min(1.0, float(angular)))
    trama   = f"<CMD,{duty},{angular:.3f}>\n"
    try:
        serial_port.write(trama.encode('utf-8'))
    except Exception as e:
        print(f"  [ERROR] Al enviar CMD: {e}")

# =============================================================================
# HILO DE TELEMETRÍA — Lee tramas TLM del ESP32 (ultrasónicos)
# =============================================================================

def hilo_telemetria(serial_port):
    print("[TELEMETRÍA] Hilo iniciado.")

    buffer       = ""
    dentro_trama = False

    while estado.running:
        try:
            if serial_port.in_waiting > 0:
                byte = serial_port.read(1).decode('utf-8', errors='ignore')

                if byte == '<':
                    buffer       = ""
                    dentro_trama = True
                elif byte == '>' and dentro_trama:
                    procesar_trama(buffer)
                    buffer       = ""
                    dentro_trama = False
                elif dentro_trama:
                    buffer += byte
                    if len(buffer) > 64:
                        buffer       = ""
                        dentro_trama = False
        except Exception as e:
            print(f"  [ERROR] Telemetría: {e}")
            time.sleep(0.1)

    print("[TELEMETRÍA] Hilo detenido.")


def procesar_trama(raw):
    """Procesa tramas TLM,HBT,EMG entrantes desde el ESP32."""
    campos = raw.strip().split(',')
    if not campos:
        return

    tipo = campos[0]

    if tipo == "TLM" and len(campos) == 3:
        try:
            us_izq = float(campos[1])
            us_der = float(campos[2])
            with estado.lock:
                estado.us_izq = us_izq
                estado.us_der = us_der
        except ValueError:
            pass

    elif tipo == "EMG":
        print("  ⚠ [ESP32] Reporta parada de emergencia (watchdog).")

# =============================================================================
# LÓGICA DE DECISIÓN — Jerarquía de prioridades
# =============================================================================

def filtrar_detecciones(detecciones, area_frame):
    """
    Filtra detecciones por tamaño mínimo de bounding box.
    Retorna diccionario {clase_id: max_confianza} de detecciones válidas.
    """
    detecciones_validas = {}

    for box in detecciones:
        x1, y1, x2, y2 = box.xyxy[0]
        area_bbox      = float((x2 - x1) * (y2 - y1))
        ratio          = area_bbox / area_frame

        if ratio < MIN_BBOX_AREA_RATIO:
            continue

        class_id   = int(box.cls)
        confidence = float(box.conf)

        # Conservar la detección con mayor confianza por clase
        if class_id not in detecciones_validas or confidence > detecciones_validas[class_id]:
            detecciones_validas[class_id] = confidence

    return detecciones_validas


def decidir_estado(detecciones_validas):
    """
    Aplica la jerarquía de decisiones y retorna (duty, nombre_estado, razón).
    Modifica el estado interno cuando es necesario.
    """
    ahora = time.time()

    with estado.lock:
        # ------------------------------------------------------------
        # Fase de arranque — esperar timeout antes de moverse
        # ------------------------------------------------------------
        if not estado.arrancado:
            if ahora >= estado.fin_arranque_ts:
                estado.arrancado = True
                print(f"\n  [ARRANQUE] Timeout de {ARRANQUE_TIMEOUT_S}s cumplido — vehículo activo\n")
            else:
                restante = estado.fin_arranque_ts - ahora
                return (DUTY_DETENIDO, "ARRANQUE", f"Esperando ({restante:.1f}s)")

        # ------------------------------------------------------------
        # Capturar lecturas de sensores
        # ------------------------------------------------------------
        us_izq = estado.us_izq
        us_der = estado.us_der

        # Distancia mínima válida (ignorar -1.0 que es fuera de rango)
        distancias_validas = [d for d in (us_izq, us_der) if d > 0]
        dist_min = min(distancias_validas) if distancias_validas else 999.0

        # ------------------------------------------------------------
        # P1 — SEGURIDAD INMEDIATA (ultrasónicos)
        # ------------------------------------------------------------
        if dist_min < US_UMBRAL_DETENER:
            return (DUTY_DETENIDO, "DETENIDO", f"Obstáculo crítico a {dist_min:.1f}cm")

        # ------------------------------------------------------------
        # P2 — SEÑALIZACIÓN OBLIGATORIA (YOLO)
        # ------------------------------------------------------------

        # red-light → bloqueo hasta ver green-light explícito
        if CLS_RED_LIGHT in detecciones_validas:
            estado.bloqueado_por_red = True

        # Si está bloqueado por red, solo desbloquea con green-light
        if estado.bloqueado_por_red:
            if CLS_GREEN_LIGHT in detecciones_validas:
                estado.bloqueado_por_red = False
                print(f"\n  ✓ [RED→GREEN] green-light detectada — desbloqueando movimiento\n")
            else:
                return (DUTY_DETENIDO, "DETENIDO", "Bloqueado por red-light")

        # stop-sign (Opción A — una detención por aparición)
        stop_detectada = CLS_STOP_SIGN in detecciones_validas
        en_stop = ahora < estado.detenido_stop_hasta

        if stop_detectada and not estado.stop_ya_procesada and not en_stop:
            estado.detenido_stop_hasta = ahora + DURACION_STOP_SIGN_S
            estado.stop_ya_procesada   = True
            conf = detecciones_validas[CLS_STOP_SIGN]
            print(f"\n  ⚠ [DETECCIÓN] STOP-SIGN  conf={conf:.2f}")
            print(f"     [ACCIÓN]    Deteniendo {DURACION_STOP_SIGN_S}s\n")

        if not stop_detectada and not en_stop:
            if estado.stop_ya_procesada:
                print(f"     [ESTADO]    Stop-sign fuera de vista — listo para próxima\n")
            estado.stop_ya_procesada = False

        if ahora < estado.detenido_stop_hasta:
            restante = estado.detenido_stop_hasta - ahora
            return (DUTY_DETENIDO, "DETENIDO", f"Stop-sign ({restante:.1f}s)")

        # traffic-light sin color identificado → detención preventiva
        # SOLO si no hay green-light ni red-light en la escena (ya manejados arriba)
        hay_semaforo_con_color = (
            CLS_GREEN_LIGHT in detecciones_validas or
            CLS_RED_LIGHT   in detecciones_validas
        )
        if CLS_TRAFFIC_LIGHT in detecciones_validas and not hay_semaforo_con_color:
            return (DUTY_DETENIDO, "DETENIDO", "Semáforo sin color — precaución")

        # ------------------------------------------------------------
        # P3 — INTERACCIÓN CON ENTORNO
        # ------------------------------------------------------------
        if dist_min < US_UMBRAL_LENTO:
            return (DUTY_LENTO, "LENTO", f"Obstáculo cercano a {dist_min:.1f}cm")

        if CLS_VEHICLE in detecciones_validas:
            return (DUTY_LENTO, "LENTO", "Vehículo detectado")

        # ------------------------------------------------------------
        # P4 — ESTADO POR DEFECTO
        # ------------------------------------------------------------
        if CLS_GREEN_LIGHT in detecciones_validas:
            return (DUTY_NORMAL, "NORMAL", "Vía libre (green-light)")

        return (DUTY_NORMAL, "NORMAL", "Sin restricciones")


def aplicar_decision(duty, nombre_estado):
    """Actualiza el estado del vehículo con la decisión tomada."""
    with estado.lock:
        estado.duty          = duty
        estado.angular       = ANGULAR_RECTO
        estado.estado_nombre = nombre_estado

# =============================================================================
# HILO DE PERCEPCIÓN — YOLO + Decisión
# =============================================================================

def hilo_percepcion():
    print("[PERCEPCIÓN] Cargando modelo...")
    model = YOLO(MODEL_PATH)
    print(f"[PERCEPCIÓN] Modelo cargado. Clases: {model.names}")

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] No se pudo abrir la cámara.")
        estado.running = False
        return

    print("[PERCEPCIÓN] Cámara abierta. Presiona Ctrl+C para salir.\n")

    # Iniciar timeout de arranque
    with estado.lock:
        estado.fin_arranque_ts = time.time() + ARRANQUE_TIMEOUT_S
    print(f"[ARRANQUE] Vehículo detenido por {ARRANQUE_TIMEOUT_S}s antes de activarse...\n")

    ultimo_log     = time.time()
    frames_periodo = 0

    while estado.running:
        ret, frame = cap.read()
        if not ret:
            continue

        frames_periodo += 1
        h, w = frame.shape[:2]
        area_frame = float(h * w)

        # Inferencia YOLO
        t1 = time.time()
        results = model(frame, conf=CONF_THRESHOLD, verbose=False)[0]
        t_inference = time.time() - t1

        # Filtrar y decidir
        detecciones_validas = filtrar_detecciones(results.boxes, area_frame)
        duty, nombre, razon = decidir_estado(detecciones_validas)
        aplicar_decision(duty, nombre)

        # Log periódico
        ahora = time.time()
        if ahora - ultimo_log >= INTERVALO_LOG_S:
            with estado.lock:
                us_izq = estado.us_izq
                us_der = estado.us_der

            fps = frames_periodo / (ahora - ultimo_log)
            detecciones_str = ", ".join(
                f"{NOMBRE_CLASES[c]}({conf:.2f})"
                for c, conf in detecciones_validas.items()
            ) if detecciones_validas else "ninguna"

            print(f"  [STATUS] {nombre}  duty={duty}  |  FPS: {fps:.1f}  |  Inf: {t_inference*1000:.0f}ms")
            print(f"  [SENSOR] US_izq={us_izq:.1f}cm  US_der={us_der:.1f}cm")
            print(f"  [VISIÓN] {detecciones_str}")
            print(f"  [RAZÓN]  {razon}")
            print()

            ultimo_log     = ahora
            frames_periodo = 0

    cap.release()
    print("[PERCEPCIÓN] Hilo detenido.")

# =============================================================================
# MAIN
# =============================================================================

def main():
    print("="*60)
    print("  AVA — Script integrado completo")
    print("="*60)

    # Abrir puerto serial
    try:
        serial_port = serial.Serial(
            port        = PUERTO,
            baudrate    = BAUDIOS,
            bytesize    = serial.EIGHTBITS,
            parity      = serial.PARITY_NONE,
            stopbits    = serial.STOPBITS_ONE,
            timeout     = 1
        )
        print(f"[MAIN] Puerto {PUERTO} abierto.\n")
    except serial.SerialException as e:
        print(f"[ERROR] No se pudo abrir el puerto: {e}")
        return

    # Lanzar hilos secundarios
    t_control = threading.Thread(
        target=hilo_control, args=(serial_port,), daemon=True
    )
    t_telemetria = threading.Thread(
        target=hilo_telemetria, args=(serial_port,), daemon=True
    )

    t_control.start()
    t_telemetria.start()

    # Espera para que ESP32 esté listo y empiecen a llegar TLM
    print("[MAIN] Esperando 3s al ESP32...")
    time.sleep(3)

    # Ejecutar percepción en el hilo principal
    try:
        hilo_percepcion()
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupción manual (Ctrl+C).")
        estado.running = False

    # Esperar terminación limpia
    t_control.join(timeout=3)
    t_telemetria.join(timeout=2)

    if serial_port.is_open:
        serial_port.close()
        print("[MAIN] Puerto serial cerrado.")


if __name__ == "__main__":
    main()
