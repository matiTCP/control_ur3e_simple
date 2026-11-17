"""
mano_robot.py - Taller corto: mover el UR3e con la mano frente a la cámara.

Mano a la izquierda o a la derecha del centro -> la articulación gira en un sentido u otro.
Mano en el centro o fuera de la imagen        -> velocidad 0 (el robot se detiene).

Uso:
    python mano_robot.py           PRUEBA: muestra la velocidad, NO mueve el robot
    python mano_robot.py --robot   mueve el robot (¡zona despejada y parada de emergencia!)
Tecla q = salir.
Necesita: python -m pip install mediapipe
          y el modelo modelos/hand_landmarker.task junto a este archivo.
"""
import os
import sys
import time

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

import ur_basico as ur          # biblioteca del taller: conexión, estado y URScript

IP = "192.168.0.15"             # IP del robot
ARTICULACION = 0                # 0 Base, 1 Hombro, 2 Codo, 3 Muñeca 1, 4 Muñeca 2, 5 Muñeca 3
VEL_MAX = 10.0                  # velocidad máxima en grados/s (¡empieza bajo!)
ZONA = 0.2                      # zona central (0 a 1) en la que el robot no se mueve

# 1) Detector de manos de MediaPipe (21 puntos por mano)
carpeta = os.path.dirname(os.path.abspath(__file__))
modelo = os.path.join(carpeta, "modelos", "hand_landmarker.task")
with open(modelo, "rb") as f:
    opciones = vision.HandLandmarkerOptions(base_options=BaseOptions(model_asset_buffer=f.read()),
                                            running_mode=vision.RunningMode.VIDEO, num_hands=1)
detector = vision.HandLandmarker.create_from_options(opciones)

# 2) Conexión con el robot, solo si se pidió --robot y la persona confirma
robot = None
if "--robot" in sys.argv and ur.confirmar("El robot seguirá tu mano. ¿Continuar?"):
    robot = ur.ConexionUR(IP).iniciar()

camara = cv2.VideoCapture(0)
inicio = time.time()
ultimo_envio = 0.0
while True:
    ok, imagen = camara.read()
    if not ok:
        break
    imagen = cv2.flip(imagen, 1)                         # espejo: tu derecha queda a la derecha
    alto, ancho = imagen.shape[:2]

    # 3) Buscar la mano y calcular la velocidad
    rgb = cv2.cvtColor(imagen, cv2.COLOR_BGR2RGB)
    resultado = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                                          int((time.time() - inicio) * 1000))
    velocidad = 0.0
    if resultado.hand_landmarks:
        palma = resultado.hand_landmarks[0][9]   # punto 9 = centro de la palma (x, y de 0 a 1)
        d = (palma.x - 0.5) * 2                  # -1 izquierda ... 0 centro ... +1 derecha
        if abs(d) > ZONA:
            velocidad = d * VEL_MAX
        cv2.circle(imagen, (int(palma.x * ancho), int(palma.y * alto)), 12, (0, 220, 0), -1)

    # 4) Enviar la velocidad cada 0,1 s. Cada orden speedj dura 0,3 s: si el programa
    #    se detiene o se cuelga, el robot se para solo.
    if robot and ur.robot_listo(robot.estado) and time.time() - ultimo_envio >= 0.1:
        velocidades = [0.0] * 6
        velocidades[ARTICULACION] = velocidad
        try:
            robot.enviar(ur.script_speedj(velocidades))
        except OSError:
            pass
        ultimo_envio = time.time()

    texto = f"velocidad: {velocidad:+.1f} grados/s"
    cv2.putText(imagen, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imshow("Mano -> robot (q = salir)", imagen)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

if robot:
    try:
        robot.enviar(ur.SCRIPT_PARAR)                    # frenar al salir
    except OSError:
        pass
    robot.detener()
camara.release()
cv2.destroyAllWindows()
