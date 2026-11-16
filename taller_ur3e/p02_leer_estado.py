"""
Paso 2 - Leer el estado del robot.

Usa la biblioteca ur_basico.py para conectarse al puerto 30002 y muestra, varias veces por
segundo, los ángulos de las 6 articulaciones y la posición de la herramienta.
No mueve el robot.

Uso:
    python p02_leer_estado.py            (Ctrl+C para salir)
"""

import math
import sys
import time

import ur_basico as ur     # nuestra biblioteca (debe estar en la misma carpeta)

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.15"
LECTURAS = int(sys.argv[2]) if len(sys.argv) > 2 else 0     # 0 = sin límite

with ur.conectar(IP) as sock:                 # una sola conexión para todas las lecturas
    n = 0
    try:
        while LECTURAS == 0 or n < LECTURAS:
            e = ur.leer_estado(sock)          # espera al siguiente paquete de estado (~0,1 s)
            n += 1
            modo = ur.MODOS_ROBOT.get(e.get("modo"), e.get("modo"))
            print(f"\n--- Lectura {n} | Estado: {modo}"
                  f"{' | PARADA DE EMERGENCIA' if e.get('emergencia') else ''}"
                  f"{' | PARADA DE PROTECCIÓN' if e.get('proteccion') else ''}")
            for nombre, angulo in zip(ur.NOMBRES_ARTICULACIONES, e["q"]):
                print(f"  {nombre:<9} {angulo:8.2f} °")
            if "tcp_mm" in e:
                x, y, z = e["tcp_mm"]
                rx, ry, rz = (math.degrees(r) for r in e["tcp_rotvec"])
                print(f"  Herramienta: X={x:7.1f}  Y={y:7.1f}  Z={z:7.1f} mm"
                      f" | RX={rx:7.2f}  RY={ry:7.2f}  RZ={rz:7.2f} °")
            print("  ¿Listo para moverse?:", "SÍ" if ur.robot_listo(e) else "NO")
            time.sleep(0.5)                   # mostramos 2 lecturas por segundo
    except KeyboardInterrupt:                 # Ctrl+C
        print("\nFin.")
