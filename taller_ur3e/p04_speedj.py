"""
Paso 4 - Movimiento continuo y seguro con speedj.

En vez de dar una posición final, damos una VELOCIDAD. Cada orden dura solo 0,3 s y la
reenviamos cada 0,1 s: mientras sigamos enviando, el robot se mueve; si el programa se cuelga
o se corta la red, la última orden caduca y el robot se detiene solo ("perro guardián").

Este programa mueve la Muñeca 3 a +5 °/s durante 1,5 s, la detiene y luego la devuelve.

¡ATENCIÓN! Este programa MUEVE el robot.
"""

import sys
import time

import ur_basico as ur

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.15"
VEL = 5.0          # °/s
DURACION = 1.5     # segundos de movimiento
PERIODO = 0.1      # cada cuánto reenviamos la orden (s)
VIDA = 0.3         # cuánto dura cada orden si no llega otra (s)

estado = ur.leer_estado_una_vez(IP)
if not ur.robot_listo(estado):
    sys.exit("El robot no está listo (estado Normal, sin paradas, modo Remoto).")
inicial = estado["q"]

if not ur.confirmar(f"La Muñeca 3 girará a {VEL} °/s durante {DURACION} s. ¿Continuar?"):
    sys.exit("Cancelado.")

# Usamos UNA conexión permanente (ConexionUR) para enviar todas las órdenes. Además de ser más
# rápida que abrir una conexión por orden, va LEYENDO lo que envía el robot: si nadie lee esos
# datos, la conexión se satura y las órdenes llegan con retraso.
robot = ur.ConexionUR(IP).iniciar()
while robot.estado is None:                       # esperar a que la conexión esté lista
    time.sleep(0.05)
velocidades = [0, 0, 0, 0, 0, VEL]                # solo se mueve la articulación 6 (índice 5)
inicio = time.time()
enviadas = 0
while time.time() - inicio < DURACION:
    robot.enviar(ur.script_speedj(velocidades, acel=60, vida=VIDA))
    enviadas += 1
    time.sleep(PERIODO)
robot.enviar(ur.SCRIPT_PARAR)                     # al terminar, frenamos explícitamente
robot.detener()

time.sleep(1.0)
final = ur.leer_estado_una_vez(IP)["q"]
print(f"Órdenes enviadas: {enviadas}. La Muñeca 3 avanzó {final[5] - inicial[5]:.2f}°")

# Volvemos a la posición inicial con movej
ur.enviar_script(IP, ur.script_movej(inicial, vel=10, acel=20))
time.sleep(2.0)
print("De vuelta en:", [round(a, 2) for a in ur.leer_estado_una_vez(IP)["q"]])
