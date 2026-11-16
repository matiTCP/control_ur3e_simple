"""
Paso 10 - Seguir la mano con MediaPipe (y, opcionalmente, mover la Base del robot).

MediaPipe Hand Landmarker detecta 21 puntos de la mano en cada imagen. Calculamos el centro de
la palma y, según lo lejos que esté del centro de la imagen, una velocidad para la Base:
    - mano dentro del rectángulo central ("zona muerta")  -> velocidad 0
    - mano a la izquierda / derecha                         -> la Base gira a un lado u otro
    - no se ve la mano                                      -> velocidad 0 (el robot se para)

Requisitos:
    python -m pip install mediapipe
    Modelo: descargar "hand_landmarker.task" en la carpeta "modelos" (ver documento del taller).

Uso:
    python p10_seguir_mano.py            solo muestra la detección y la velocidad calculada
    python p10_seguir_mano.py --robot    además mueve la Base del robot (¡MUEVE EL ROBOT!)
Tecla q = salir.
"""

import math
import os
import sys
import time

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

import ur_basico as ur

IP = "192.168.0.15"
MOVER_ROBOT = "--robot" in sys.argv
CARPETA = os.path.dirname(os.path.abspath(__file__))
MODELO = os.path.join(CARPETA, "modelos", "hand_landmarker.task")
ZONA_MUERTA = 0.18          # fracción de media imagen en la que la mano no mueve el robot
VEL_MAX = 10.0              # °/s máximos de la Base
CONEXIONES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
              (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]


def fraccion_velocidad(d, zona=ZONA_MUERTA):
    """d = distancia al centro en [-1, 1] -> fracción de velocidad en [-1, 1] con zona muerta."""
    if abs(d) < zona:
        return 0.0
    return math.copysign(min(1.0, (abs(d) - zona) / (1 - zona)), d)


# 1) Crear el detector de manos (modo VIDEO: usa la imagen anterior para seguir la mano mejor)
if not os.path.exists(MODELO):
    sys.exit(f"Falta el modelo: {MODELO}")
opciones = vision.HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODELO),
    running_mode=vision.RunningMode.VIDEO,
    num_hands=1,                           # solo nos interesa una mano
    min_hand_detection_confidence=0.6)     # confianza mínima (0 a 1)
detector = vision.HandLandmarker.create_from_options(opciones)

# 2) Si vamos a mover el robot, abrimos una conexión permanente
robot = None
if MOVER_ROBOT:
    if not ur.confirmar("La Base del robot seguirá tu mano. ¿Continuar?"):
        sys.exit("Cancelado.")
    robot = ur.ConexionUR(IP).iniciar()

camara = cv2.VideoCapture(0, cv2.CAP_DSHOW)
t0 = time.time()
moviendo = False
while True:
    ok, imagen = camara.read()
    if not ok:
        break
    imagen = cv2.flip(imagen, 1)                         # espejo
    alto, ancho = imagen.shape[:2]

    # 3) Detectar: MediaPipe quiere la imagen en RGB y una marca de tiempo creciente en ms
    rgb = cv2.cvtColor(imagen, cv2.COLOR_BGR2RGB)
    resultado = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                                          int((time.time() - t0) * 1000))

    velocidad = 0.0
    zx, zy = int(ZONA_MUERTA * ancho / 2), int(ZONA_MUERTA * alto / 2)
    cv2.rectangle(imagen, (ancho // 2 - zx, alto // 2 - zy), (ancho // 2 + zx, alto // 2 + zy), (0, 200, 0), 1)
    if resultado.hand_landmarks:
        puntos = resultado.hand_landmarks[0]             # 21 puntos con x, y entre 0 y 1
        pix = [(int(p.x * ancho), int(p.y * alto)) for p in puntos]
        for a, b in CONEXIONES:                          # dibujar el "esqueleto" de la mano
            cv2.line(imagen, pix[a], pix[b], (255, 255, 255), 1)
        # Centro de la palma = promedio de la muñeca (0) y los nudillos (5, 9, 13, 17)
        cx = sum(puntos[i].x for i in (0, 5, 9, 13, 17)) / 5
        cy = sum(puntos[i].y for i in (0, 5, 9, 13, 17)) / 5
        cv2.circle(imagen, (int(cx * ancho), int(cy * alto)), 7, (255, 120, 0), -1)
        velocidad = fraccion_velocidad((cx - 0.5) * 2) * VEL_MAX

    cv2.putText(imagen, f"Base: {velocidad:+.1f} grados/s", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Seguir la mano - q para salir", imagen)

    # 4) Enviar la velocidad al robot (mismo método seguro que en el paso 4)
    if robot is not None and ur.robot_listo(robot.estado):
        try:
            if velocidad:
                robot.enviar(ur.script_speedj([velocidad, 0, 0, 0, 0, 0]))
                moviendo = True
            elif moviendo:
                robot.enviar(ur.SCRIPT_PARAR)
                moviendo = False
        except OSError:
            pass

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

if robot is not None:
    try:
        robot.enviar(ur.SCRIPT_PARAR)
    except OSError:
        pass
    robot.detener()
camara.release()
cv2.destroyAllWindows()
detector.close()
