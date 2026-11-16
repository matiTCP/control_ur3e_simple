"""
Paso 3 - Primer movimiento: movej.

Lee la posición actual, gira la Muñeca 3 (la que hace girar la herramienta) unos grados,
espera a que llegue y la devuelve a donde estaba.

¡ATENCIÓN! Este programa MUEVE el robot. Antes de ejecutarlo:
    - Zona de trabajo despejada y la parada de emergencia al alcance.
    - PolyScope en estado "Normal" y en modo "Remoto".

Uso:
    python p03_movej.py              (gira 10°)
    python p03_movej.py 192.168.0.15 5
"""

import sys
import time

import ur_basico as ur

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.15"
GIRO = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0     # grados que gira la Muñeca 3
VELOCIDAD = 10.0                                              # °/s (lento a propósito)


def esperar_llegada(objetivo, tolerancia=0.2, limite=15.0):
    """Lee el estado hasta que todas las articulaciones estén a menos de 'tolerancia' grados
    del objetivo, o hasta que pasen 'limite' segundos. Devuelve el último estado leído."""
    inicio = time.time()
    with ur.conectar(IP) as sock:
        while True:
            e = ur.leer_estado(sock)
            error = max(abs(a - b) for a, b in zip(e["q"], objetivo))
            if error < tolerancia or time.time() - inicio > limite or e.get("proteccion"):
                return e


# 1) Comprobar que el robot está listo
estado = ur.leer_estado_una_vez(IP)
if not ur.robot_listo(estado):
    sys.exit("El robot no está listo: revisa que esté en estado Normal, sin paradas y en modo Remoto.")

inicial = estado["q"]                         # copia de los 6 ángulos actuales
print("Posición inicial:", [round(a, 2) for a in inicial])

# 2) Calcular el objetivo: igual que ahora, pero con la Muñeca 3 (índice 5) girada
objetivo = list(inicial)
objetivo[5] += GIRO

# 3) Pedir confirmación: nunca movemos un robot sin que la persona lo apruebe
if not ur.confirmar(f"Se va a girar la Muñeca 3 {GIRO:+.1f}° a {VELOCIDAD:.0f} °/s. ¿Continuar?"):
    sys.exit("Cancelado.")

# 4) Ida
ur.enviar_script(IP, ur.script_movej(objetivo, vel=VELOCIDAD, acel=2 * VELOCIDAD))
e = esperar_llegada(objetivo)
print(f"Ida:    Muñeca 3 = {e['q'][5]:.2f}° (objetivo {objetivo[5]:.2f}°)")

# 5) Vuelta a la posición inicial
ur.enviar_script(IP, ur.script_movej(inicial, vel=VELOCIDAD, acel=2 * VELOCIDAD))
e = esperar_llegada(inicial)
print(f"Vuelta: Muñeca 3 = {e['q'][5]:.2f}° (objetivo {inicial[5]:.2f}°)")
