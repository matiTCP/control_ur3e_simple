"""
Paso 6 - Mover la herramienta (TCP) en el espacio cartesiano con movel.

Lee la pose de la herramienta (X, Y, Z y orientación), calcula una pose nueva girada unos
grados alrededor del eje Z DE LA HERRAMIENTA y la envía con movel (movimiento en línea recta).
Girar sobre el propio eje de la herramienta es el movimiento más seguro para probar: la punta
de la herramienta no se desplaza.

Incluye las funciones para pasar de "vector de rotación" a matriz de rotación y viceversa,
que son la base de cualquier cálculo con orientaciones.

¡ATENCIÓN! Este programa MUEVE el robot.
¡IMPORTANTE! Los movimientos en línea recta (movel, speedl) NO se deben hacer cerca de una
singularidad (por ejemplo en la posición Home de fábrica). El programa lo comprueba y se niega
a moverse; antes, lleva el robot con el paso 5 a una postura con el codo doblado (unos 90°)
y la Muñeca 2 lejos de 0° y de 180° (por ejemplo -90°).
"""

import math
import sys
import time

import ur_basico as ur

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.15"
GIRO = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0      # grados alrededor del eje Z de la herramienta


def rotvec_a_matriz(r):
    """Vector de rotación (rx, ry, rz) -> matriz de rotación 3x3 (fórmula de Rodrigues).

    El vector indica el EJE de giro (su dirección) y el ÁNGULO (su longitud, en radianes).
    """
    angulo = math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)
    if angulo < 1e-12:                                   # sin giro: matriz identidad
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    kx, ky, kz = (c / angulo for c in r)                 # eje unitario
    c, s, v = math.cos(angulo), math.sin(angulo), 1 - math.cos(angulo)
    return [[kx * kx * v + c,      kx * ky * v - kz * s, kx * kz * v + ky * s],
            [ky * kx * v + kz * s, ky * ky * v + c,      ky * kz * v - kx * s],
            [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c]]


def matriz_a_rotvec(R):
    """Matriz de rotación 3x3 -> vector de rotación (inversa de la función anterior)."""
    coseno = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))
    angulo = math.acos(coseno)
    if angulo < 1e-9:
        return [0.0, 0.0, 0.0]
    if math.pi - angulo < 1e-4:                          # caso especial: giro de casi 180°
        x = math.sqrt(max(0.0, (R[0][0] + 1) / 2))
        y = math.copysign(math.sqrt(max(0.0, (R[1][1] + 1) / 2)), R[0][1])
        z = math.copysign(math.sqrt(max(0.0, (R[2][2] + 1) / 2)), R[0][2])
        return [angulo * x, angulo * y, angulo * z]
    s = 2 * math.sin(angulo)
    return [angulo * (R[2][1] - R[1][2]) / s,
            angulo * (R[0][2] - R[2][0]) / s,
            angulo * (R[1][0] - R[0][1]) / s]


def multiplicar(A, B):
    """Producto de dos matrices 3x3."""
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


estado = ur.leer_estado_una_vez(IP)
if not ur.robot_listo(estado):
    sys.exit("El robot no está listo (estado Normal, sin paradas, modo Remoto).")

# Seguridad: nunca hacer movimientos lineales cerca de una singularidad
singularidad = ur.cerca_de_singularidad(estado["q"])
if singularidad:
    sys.exit(f"No se mueve: el robot está cerca de una singularidad ({singularidad}).\n"
             "Llévalo primero a una postura con el codo doblado y la Muñeca 2 lejos de 0°/180° (paso 5).")

p = estado["tcp_mm"]                      # posición actual de la herramienta (mm)
R = rotvec_a_matriz(estado["tcp_rotvec"])  # orientación actual como matriz
print(f"Herramienta en X={p[0]:.1f} Y={p[1]:.1f} Z={p[2]:.1f} mm")

# Giro alrededor del eje Z de la HERRAMIENTA: se multiplica a la DERECHA (R · Rz).
# (Para girar alrededor del eje Z de la BASE se multiplicaría a la izquierda: Rz · R.)
Rz = rotvec_a_matriz([0, 0, math.radians(GIRO)])
R_nueva = multiplicar(R, Rz)

if not ur.confirmar(f"La herramienta girará {GIRO:+.1f}° sobre su eje Z. ¿Continuar?"):
    sys.exit("Cancelado.")

ur.enviar_script(IP, ur.script_movel(p, matriz_a_rotvec(R_nueva), vel_mm=50))
time.sleep(3)
e = ur.leer_estado_una_vez(IP)
print(f"Después del giro: X={e['tcp_mm'][0]:.1f} Y={e['tcp_mm'][1]:.1f} Z={e['tcp_mm'][2]:.1f} mm "
      f"(la posición no cambia) | Muñeca 3 = {e['q'][5]:.2f}°")

# Volver a la orientación original
ur.enviar_script(IP, ur.script_movel(p, estado["tcp_rotvec"], vel_mm=50))
time.sleep(3)
print("De vuelta. Muñeca 3 =", round(ur.leer_estado_una_vez(IP)["q"][5], 2), "°")
