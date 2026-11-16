"""
Paso 1 - Comprobar la conexión con el robot.

Intenta abrir una conexión TCP con cada puerto conocido del robot UR y dice cuáles responden.
No envía nada al robot: es totalmente seguro.

Uso:
    python p01_conexion.py               (IP por defecto 192.168.0.15)
    python p01_conexion.py 192.168.0.20
"""

import socket   # biblioteca estándar para comunicaciones de red
import sys      # para leer los argumentos de la línea de comandos (sys.argv)

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.15"

# Puertos habituales de un robot UR e-Series y para qué sirven
PUERTOS = {
    29999: "Dashboard (encender, cargar programas...)",
    30001: "Interfaz primaria (estado + URScript)",
    30002: "Interfaz secundaria (estado + URScript)  <- la que usamos",
    30003: "Interfaz de tiempo real (500 Hz)",
    30004: "RTDE (intercambio de datos en tiempo real)",
    502:   "Modbus TCP (registros, lo usamos para el gripper)",
}


def puerto_abierto(ip, puerto, espera=2.0):
    """Devuelve True si se puede abrir una conexión TCP con ip:puerto antes de 'espera' segundos."""
    try:
        # create_connection hace todo el trabajo: resuelve la dirección y conecta.
        # El bloque "with" cierra la conexión automáticamente al terminar.
        with socket.create_connection((ip, puerto), timeout=espera):
            return True
    except OSError:          # incluye "conexión rechazada" y "tiempo agotado"
        return False


print(f"Comprobando el robot en {IP}...\n")
for puerto, descripcion in PUERTOS.items():
    estado = "ABIERTO" if puerto_abierto(IP, puerto) else "cerrado"
    print(f"  Puerto {puerto:>5}: {estado:8}  {descripcion}")

print("\nSi el 30002 aparece ABIERTO, ya podemos seguir con el paso 2.")
