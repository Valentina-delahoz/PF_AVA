# AVA — Autonomous Vehicle Prototype Powered by AI Agents

> Vehículo autónomo a escala con dos agentes de IA especializados para navegación segura en entornos urbanos controlados.

**Universidad del Norte** · Ingeniería Eléctrica y Electrónica · 2026-1

**Autores:** Valentina De la Hoz · Roger Coronell · Samir Albor  
**Asesores:** Ph.D. Christian Quintero · Ph.D. Mauricio Pardo

---

## ¿Qué es AVA?

AVA es un prototipo de vehículo autónomo a escala 1:10, construido íntegramente con recursos de la Universidad del Norte (carrocería en impresión 3D), que integra dos agentes de inteligencia artificial:

- **Agente de percepción** — detecta señales de tránsito, semáforos, vehículos y sigue el carril en tiempo real
- **Agente de decisión** — interpreta las percepciones y ejecuta acciones autónomas de conducción mediante una Máquina de Estados Finitos (FSM)

---

## Resultados

| Métrica | Valor |
|---|---|
| mAP@0.5 (detección de objetos) | 91.3% |
| F1-Score | 0.94 |
| TDA media global (30 experimentos) | 90.60% |
| Latencia del sistema | ~50 ms |
| FPS en operación real | ~10 FPS |
| Autonomía de batería | ~2 horas |

> Con un **95% de confianza**, AVA resuelve correctamente más del **85%** de los eventos de navegación en pistas de distinta complejidad (prueba t de Student, α = 0.05).

---

## Arquitectura del sistema

```
Cámara web
    │
    ▼
Raspberry Pi 4  ──── YOLOv8 (detección de objetos)
    │           ──── Transformada de Hough (seguimiento de carril)
    │           ──── FSM (agente de decisión)
    │
    ▼  UART (Serial)
ESP32
    │
    ├── Motor DC 25GA370 (tracción) → Driver L298N
    ├── Servomotor MG995 (dirección)
    └── 2× Ultrasonidos HC-SR04 (detección de obstáculos)
```

**Software:** Python · ROS2 · OpenCV · YOLOv8 (Ultralytics)  
**Hardware:** Raspberry Pi 4 · ESP32 · LiPo 3S 2200mAh · Webcam Lightbek-PLU

---

## Estructura del repositorio

```
PF_AVA/
├── raspberry_pi/
│   └── ava_main.py          # Agente de percepción y control (Python)
├── esp32/
│   └── esp32_firmware.ino   # Firmware de actuación y telemetría (C++)
├── docs/
│   └── manual_usuario.pdf   # Guía de inicio rápido
└── README.md
```

---

## Clases detectadas por YOLOv8

| Clase | Acción del vehículo |
|---|---|
| `green-traffic-light` | Avanzar |
| `red-traffic-light` | Detenerse (hasta ver verde) |
| `stop-sign` | Detenerse 3 segundos |
| `vehicle` | Reducir velocidad |

---

## Jerarquía de decisiones (FSM)

```
P1 — Obstáculo <15 cm          → DETENIDO (ultrasónicos)
P2 — Semáforo rojo / Stop sign → DETENIDO (visión)
P3 — Vehículo detectado        → LENTO
P4 — Vía libre / semáforo verde → NORMAL
```

---

## Pistas de prueba

| Pista | Descripción | Checkpoints |
|---|---|---|
| Pista 1 | Curvas y rectas — sin señales | 4 |
| Pista 2 | Recta con semáforos y señales | 5–6 |
| Pista 3 | Intersección completa con obstáculos | 6 |

---

## Instalación

### Raspberry Pi 4

```bash
pip install ultralytics opencv-python pyserial
python3 raspberry_pi/ava_main.py
```

### ESP32

1. Abre `esp32/esp32_firmware.ino` en Arduino IDE
2. Instala la librería `ESP32Servo`
3. Selecciona la placa `ESP32 Dev Module`
4. Sube el firmware

---

## Cómo usar AVA

1. Carga la batería LiPo 3S 2200mAh completamente
2. Activa el switch físico del vehículo
3. Conéctate por SSH vía ZeroTier: `ssh pi@10.116.170.97`
4. La navegación autónoma inicia automáticamente
5. Para detener: apaga el switch físico

> Consulta el [Manual de Usuario](docs/manual_usuario.pdf) para instrucciones detalladas de configuración de pistas y operación.

---

## Publicación

Informe final disponible en el Repositorio Institucional de la Universidad del Norte.

---

*Proyecto de grado — Ingeniería Eléctrica y Electrónica, Universidad del Norte, Barranquilla, Colombia, 2026.*
