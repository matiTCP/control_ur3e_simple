"""
Paso 9 - Ver la cámara con OpenCV y capturar fotos.

Teclas en la ventana de vídeo:
    c = capturar foto (se guarda en la carpeta "capturas")
    x = mostrar/ocultar cruz central
    q = salir

Requiere OpenCV:  python -m pip install opencv-python   (o mediapipe, que ya lo incluye)
"""

import os
import sys
import time

import cv2     # OpenCV: visión artificial (cámaras, imágenes, dibujo...)

INDICE_CAMARA = int(sys.argv[1]) if len(sys.argv) > 1 else 0    # 0 = primera cámara del PC
CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "capturas")

# CAP_DSHOW = usar DirectShow en Windows (abre la cámara más rápido)
camara = cv2.VideoCapture(INDICE_CAMARA, cv2.CAP_DSHOW)
if not camara.isOpened():
    sys.exit("No se pudo abrir la cámara (¿la está usando otro programa?).")

cruz = True
t_anterior, fps = time.time(), 0.0
while True:
    ok, imagen = camara.read()          # imagen = matriz (alto x ancho x 3) en formato BGR
    if not ok:
        print("La cámara no entregó imagen.")
        break
    imagen = cv2.flip(imagen, 1)        # efecto espejo: más natural al mirarse
    alto, ancho = imagen.shape[:2]

    vista = imagen.copy()               # dibujamos sobre una copia (la foto se guarda limpia)
    if cruz:
        cv2.line(vista, (0, alto // 2), (ancho, alto // 2), (0, 255, 0), 1)
        cv2.line(vista, (ancho // 2, 0), (ancho // 2, alto), (0, 255, 0), 1)
    ahora = time.time()
    fps = 0.9 * fps + 0.1 / max(ahora - t_anterior, 1e-6)   # media suavizada de fotogramas/s
    t_anterior = ahora
    cv2.putText(vista, f"{fps:.0f} fps  |  c: foto  x: cruz  q: salir", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imshow("Camara - Taller UR3e", vista)

    tecla = cv2.waitKey(1) & 0xFF       # espera 1 ms por una tecla (necesario para refrescar)
    if tecla == ord("q"):
        break
    elif tecla == ord("x"):
        cruz = not cruz
    elif tecla == ord("c"):
        os.makedirs(CARPETA, exist_ok=True)
        ruta = os.path.join(CARPETA, time.strftime("foto_%Y%m%d_%H%M%S.png"))
        cv2.imwrite(ruta, imagen)
        print("Foto guardada:", ruta)

camara.release()
cv2.destroyAllWindows()
