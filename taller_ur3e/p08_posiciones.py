"""
Paso 8 - Guardar posiciones y volver a ellas (archivo JSON).

Uso:
    python p08_posiciones.py listar
    python p08_posiciones.py guardar "encima de la pieza"
    python p08_posiciones.py ir "encima de la pieza"
    python p08_posiciones.py home

JSON es un formato de texto muy usado para guardar datos. El archivo queda así:
    {"home": [0, -90, 0, -90, 0, 0],
     "posiciones": {"encima de la pieza": [12.5, -80.1, 40.2, -50.0, -90.0, 0.0]}}

¡ATENCIÓN! "ir" y "home" MUEVEN el robot.
"""

import json      # leer/escribir archivos JSON
import os
import sys

import ur_basico as ur

IP = "192.168.0.15"
ARCHIVO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "posiciones.json")
HOME_FABRICA = [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]      # Home de fábrica de Universal Robots


def cargar():
    """Lee el archivo; si no existe (primera vez) devuelve datos vacíos."""
    try:
        with open(ARCHIVO, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {"home": HOME_FABRICA, "posiciones": {}}


def guardar(datos):
    with open(ARCHIVO, "w", encoding="utf-8") as f:
        json.dump(datos, f, ensure_ascii=False, indent=2)   # indent=2: legible para personas


def mover_a(q, nombre):
    estado = ur.leer_estado_una_vez(IP)
    if not ur.robot_listo(estado):
        sys.exit("El robot no está listo.")
    mayor = max(abs(a - b) for a, b in zip(q, estado["q"]))   # mayor giro de una articulación
    if not ur.confirmar(f"Ir a «{nombre}» (la articulación que más gira: {mayor:.0f}°). ¿Continuar?"):
        sys.exit("Cancelado.")
    ur.enviar_script(IP, ur.script_movej(q, vel=20, acel=40))
    print(f"Moviendo a «{nombre}»…")


datos = cargar()
orden = sys.argv[1] if len(sys.argv) > 1 else "listar"
nombre = sys.argv[2] if len(sys.argv) > 2 else ""

if orden == "listar":
    print("Home:", datos["home"])
    for n, q in datos["posiciones"].items():
        print(f"  {n}: {[round(a, 1) for a in q]}")
elif orden == "guardar" and nombre:
    q = ur.leer_estado_una_vez(IP)["q"]
    datos["posiciones"][nombre] = [round(a, 3) for a in q]
    guardar(datos)
    print(f"Guardada «{nombre}»: {[round(a, 1) for a in q]}")
elif orden == "ir" and nombre in datos["posiciones"]:
    mover_a(datos["posiciones"][nombre], nombre)
elif orden == "home":
    mover_a(datos["home"], "Home")
else:
    print(__doc__)
