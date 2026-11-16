"""
ur_basico.py - Biblioteca mínima para comunicarse con un robot Universal Robots (e-Series).

Taller de Robótica: control de un brazo colaborativo UR3e con Python.

Qué hace esta biblioteca:
    1. Abre una conexión TCP/IP con el robot (puerto 30002, "interfaz secundaria").
    2. Lee y decodifica los paquetes binarios que el robot envía unas 10 veces por segundo:
       posición de las articulaciones, posición de la herramienta y estado del robot.
    3. Construye pequeños programas en URScript (el lenguaje del robot) y se los envía.

Solo usa la biblioteca estándar de Python: no hay que instalar nada.
"""

import math        # funciones matemáticas: radians(), degrees()
import socket      # comunicación por red TCP/IP
import struct      # convertir bytes <-> números (int, double...)
import threading   # ejecutar la lectura del robot "en paralelo" (hilos)

# ---------------------------------------------------------------------------- constantes
PUERTO_SECUNDARIO = 30002          # puerto que usamos para leer el estado Y enviar URScript

NOMBRES_ARTICULACIONES = ["Base", "Hombro", "Codo", "Muñeca 1", "Muñeca 2", "Muñeca 3"]

# Modo general del robot (campo "robotMode" del paquete "Robot mode data")
MODOS_ROBOT = {
    -1: "Sin controlador", 0: "Desconectado", 1: "Confirmar seguridad", 2: "Arrancando",
    3: "Brazo apagado", 4: "Encendido", 5: "Frenos puestos", 6: "Backdrive", 7: "Funcionando",
}
MODO_FUNCIONANDO = 7               # el brazo está encendido y con los frenos liberados
MODO_ARTICULACION_OK = 253         # cada articulación informa 253 cuando está lista para moverse


# ---------------------------------------------------------------------------- lectura
def recibir_exacto(sock, n):
    """Recibe exactamente n bytes del socket.

    sock.recv(n) puede devolver MENOS bytes de los pedidos (la red entrega los datos por trozos),
    así que repetimos hasta juntar los n bytes. Si el robot cierra la conexión, recv devuelve b"".
    """
    datos = bytearray()
    while len(datos) < n:
        trozo = sock.recv(n - len(datos))
        if not trozo:
            raise ConnectionError("el robot cerró la conexión")
        datos += trozo
    return bytes(datos)


def conectar(ip, puerto=PUERTO_SECUNDARIO, timeout=5.0):
    """Abre una conexión TCP con el robot y la devuelve (un objeto socket)."""
    return socket.create_connection((ip, puerto), timeout=timeout)


def leer_estado(sock):
    """Lee mensajes del robot hasta encontrar uno de "estado del robot" y lo decodifica.

    Formato de cada mensaje del puerto 30002 (todos los números en "big-endian", símbolo "!"):
        int32  tamaño total del mensaje (incluye estos 4 bytes)
        uint8  tipo de mensaje (16 = estado del robot; otros = versión, avisos...)
        ...    contenido
    El mensaje de estado (tipo 16) contiene varios "subpaquetes", cada uno con:
        int32  tamaño del subpaquete
        uint8  tipo de subpaquete (0 = modo del robot, 1 = articulaciones, 4 = herramienta...)

    Devuelve un diccionario con:
        "modo", "emergencia", "proteccion", "programa",
        "q" (6 ángulos en grados), "modos_art" (6 números),
        "tcp_mm" (X, Y, Z en mm) y "tcp_rotvec" (RX, RY, RZ en radianes)
    """
    while True:
        tamano, tipo = struct.unpack("!iB", recibir_exacto(sock, 5))   # "i" = int32, "B" = uint8
        cuerpo = recibir_exacto(sock, tamano - 5)                       # el resto del mensaje
        if tipo != 16:
            continue                                                    # no es de estado: lo saltamos

        estado = {}
        pos = 0                                                         # posición dentro del cuerpo
        while pos + 5 <= len(cuerpo):
            sub_tam, sub_tipo = struct.unpack_from("!iB", cuerpo, pos)
            if sub_tam <= 0:
                break
            if sub_tipo == 0:                                           # Robot mode data
                # bytes: 5-12 timestamp, 13 conectado, 14 habilitado, 15 encendido,
                #        16 emergencia, 17 parada de protección, 18 programa en marcha,
                #        19 programa en pausa, 20 modo del robot
                estado["emergencia"] = bool(cuerpo[pos + 16])
                estado["proteccion"] = bool(cuerpo[pos + 17])
                estado["programa"] = bool(cuerpo[pos + 18])
                estado["modo"] = struct.unpack_from("!b", cuerpo, pos + 20)[0]   # "b" = int8 con signo
            elif sub_tipo == 1:                                         # Joint data
                # 6 bloques de 41 bytes: q_actual (double), q_objetivo (double), velocidad (double),
                # corriente (float), tensión (float), temp. motor (float), (float), modo (uint8)
                q, modos = [], []
                for j in range(6):
                    base = pos + 5 + j * 41
                    q_rad = struct.unpack_from("!d", cuerpo, base)[0]  # "d" = double de 8 bytes
                    q.append(math.degrees(q_rad))                      # el robot trabaja en radianes
                    modos.append(cuerpo[base + 40])
                estado["q"] = q
                estado["modos_art"] = modos
            elif sub_tipo == 4:                                         # Cartesian info
                x, y, z, rx, ry, rz = struct.unpack_from("!6d", cuerpo, pos + 5)
                estado["tcp_mm"] = [x * 1000, y * 1000, z * 1000]       # metros -> milímetros
                estado["tcp_rotvec"] = [rx, ry, rz]                     # vector de rotación (rad)
            pos += sub_tam                                              # saltar al siguiente subpaquete
        if "q" in estado:
            return estado


def leer_estado_una_vez(ip):
    """Conecta, lee un estado y desconecta. Cómodo para programas sencillos."""
    with conectar(ip) as sock:
        return leer_estado(sock)


def cerca_de_singularidad(q, margen=15.0):
    """Devuelve un texto si la postura q (grados) está cerca de una singularidad, o "" si no.

    En una singularidad el robot "pierde" una dirección de movimiento: para seguir una línea
    recta necesitaría velocidades y aceleraciones enormes en alguna articulación. El controlador
    lo detecta y detiene el brazo con un fallo (por ejemplo C306A3 "Acceleration failed to pass
    sanity check"). Las dos más comunes en un UR:
        - Codo cerca de 0°: brazo estirado.
        - Muñeca 2 cerca de 0° o 180°: los ejes de la Muñeca 1 y la Muñeca 3 quedan alineados.
    La posición Home de fábrica (0, -90, 0, -90, 0, 0) cumple LAS DOS.
    """
    def distancia(angulo, referencia):          # diferencia de ángulos en el rango -180..180
        return abs((angulo - referencia + 180) % 360 - 180)
    avisos = []
    if distancia(q[2], 0) < margen:
        avisos.append("codo casi estirado")
    if distancia(q[4], 0) < margen or distancia(q[4], 180) < margen:
        avisos.append("Muñeca 2 cerca de 0°/180°")
    return " y ".join(avisos)


def confirmar(pregunta):
    """Pregunta en la consola y devuelve True solo si la respuesta es s / si / sí.

    Regla de oro: un programa nunca debe mover el robot sin que una persona lo apruebe.
    (lstrip quita un carácter invisible que a veces añade Windows al inicio del texto)
    """
    respuesta = input(f"{pregunta} (s/n): ").strip().lower().lstrip("﻿")
    return respuesta in ("s", "si", "sí")


def robot_listo(estado):
    """True si el robot puede moverse: funcionando, sin paradas y con las 6 articulaciones listas."""
    return (estado is not None
            and estado.get("modo") == MODO_FUNCIONANDO
            and not estado.get("emergencia")
            and not estado.get("proteccion")
            and all(m == MODO_ARTICULACION_OK for m in estado.get("modos_art", [])))


# ---------------------------------------------------------------------------- URScript
def enviar_script(ip, script):
    """Envía un programa URScript al robot por una conexión nueva.

    El robot solo acepta programas desde la red si PolyScope está en modo "Remoto".
    Un programa nuevo SUSTITUYE al que se esté ejecutando.
    """
    with conectar(ip) as sock:
        sock.sendall(script.encode("utf-8"))


def script_movej(q_grados, vel=20.0, acel=40.0):
    """Programa que mueve las articulaciones a q_grados (6 ángulos en grados).

    vel en grados/s y acel en grados/s². URScript trabaja en radianes, por eso convertimos.
    """
    q = ", ".join(f"{math.radians(a):.6f}" for a in q_grados)
    return ("def mover():\n"
            f"  movej([{q}], a={math.radians(acel):.5f}, v={math.radians(vel):.5f})\n"
            "end\n")


def script_movel(p_mm, rotvec_rad, vel_mm=50.0, acel_mm=200.0):
    """Programa que lleva la herramienta en LÍNEA RECTA a la pose indicada.

    p_mm: X, Y, Z en milímetros. rotvec_rad: orientación como vector de rotación (radianes).
    """
    pose = ", ".join([f"{c / 1000:.6f}" for c in p_mm] + [f"{r:.6f}" for r in rotvec_rad])
    return ("def mover_lineal():\n"
            f"  movel(p[{pose}], a={acel_mm / 1000:.4f}, v={vel_mm / 1000:.4f})\n"
            "end\n")


def script_speedj(qd_grados, acel=60.0, vida=0.3):
    """Programa que da VELOCIDAD a las articulaciones (grados/s) durante 'vida' segundos.

    Al terminar el programa el robot se detiene solo. Si lo reenviamos cada 0,1 s,
    el movimiento es continuo; si dejamos de enviarlo, se para (seguridad).
    """
    qd = ", ".join(f"{math.radians(v):.5f}" for v in qd_grados)
    return f"def jog():\n  speedj([{qd}], {math.radians(acel):.4f}, {vida})\nend\n"


SCRIPT_PARAR = "def parar():\n  stopj(3.0)\nend\n"   # frena las articulaciones (3 rad/s²)


# ---------------------------------------------------------------------------- conexión permanente
class ConexionUR:
    """Mantiene UNA conexión abierta con el robot.

    - Un hilo en segundo plano lee el estado sin parar y guarda el último en self.estado.
    - self.enviar(script) manda URScript por esa misma conexión (más rápido que abrir una nueva).
    Se usa en las interfaces gráficas, donde necesitamos leer y enviar a la vez.
    """

    def __init__(self, ip):
        self.ip = ip
        self.estado = None          # último estado leído (o None si no hay conexión)
        self.error = None           # texto del último error de conexión
        self._sock = None
        self._candado = threading.Lock()     # evita que dos hilos envíen a la vez
        self._fin = threading.Event()        # señal para terminar el hilo
        self._hilo = threading.Thread(target=self._bucle, daemon=True)

    def iniciar(self):
        self._hilo.start()
        return self

    def _bucle(self):
        while not self._fin.is_set():
            try:
                with conectar(self.ip, timeout=3) as sock:
                    self._sock = sock
                    while not self._fin.is_set():
                        self.estado = leer_estado(sock)
                        self.error = None
            except (OSError, struct.error) as e:        # sin red, robot apagado, etc.
                self._sock = None
                self.estado = None
                self.error = str(e)
                self._fin.wait(1.5)                     # esperar y reintentar

    def enviar(self, script):
        with self._candado:
            if self._sock is None:
                raise ConnectionError("sin conexión con el robot")
            self._sock.sendall(script.encode("utf-8"))

    def detener(self):
        self._fin.set()
