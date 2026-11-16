#!/usr/bin/env python3
"""
Control del UR3e por TCP/IP: articulaciones, herramienta (TCP) y gripper Robotiq.

- Lee el estado del robot por el puerto 30002 (interfaz secundaria).
- Mueve el robot enviando URScript por el mismo puerto (speedj / speedl / movej / movel).
- Gripper Robotiq: el script se conecta desde dentro del robot al servicio de la URCap
  (127.0.0.1:63352) y deja el estado en los registros Modbus 128-131, que el PC lee por el puerto 502.
- Solo usa la biblioteca estándar de Python.

Uso:
    python control_ur3e.py                 -> abre la ventana (IP por defecto 192.168.0.15)
    python control_ur3e.py 192.168.0.20    -> abre la ventana con otra IP
    python control_ur3e.py --leer [IP]     -> imprime la posición actual y sale (no mueve nada)

Mover: mantener pulsado − / + en la ventana, o activar «Teclado» y mantener la tecla:
    Q/A · W/S · E/D · R/F · T/G · Y/H  (una pareja por fila)
    O / C = abrir / cerrar gripper · Z / X = velocidad · Espacio o Esc = PARAR

Requisitos en el robot (PolyScope e-Series):
    1. Brazo encendido y frenos liberados (estado "Normal").
    2. Control Remoto activado y modo "Remoto" seleccionado (esquina superior derecha).
    3. Para el gripper: URCap de Robotiq instalada y gripper conectado.
"""

import base64
import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
try:
    import cv2                  # opcional: solo para el visor de cámara
except ImportError:
    cv2 = None

IP_POR_DEFECTO = "192.168.0.15"
PUERTO = 30002
PUERTO_MODBUS = 502

NOMBRES = ["Base", "Hombro", "Codo", "Muñeca 1", "Muñeca 2", "Muñeca 3"]
NOMBRES_TCP = ["X", "Y", "Z", "Giro X", "Giro Y", "Giro Z"]
UNIDADES_TCP = ["mm", "mm", "mm", "°", "°", "°"]
TECLAS = [("q", "a"), ("w", "s"), ("e", "d"), ("r", "f"), ("t", "g"), ("y", "h")]  # (+, −)
TECLAS_TODAS = {k for par in TECLAS for k in par}

# Articulaciones
LIMITE_GRADOS = 360.0
MARGEN_LIMITE = 5.0
VEL_MAX = 90.0
VEL_JOG_MAX = 45.0              # °/s con velocidad al 100 %
ACEL_JOG = 60.0
AVISO_SALTO = 45.0

# Herramienta
HOMBRO_MM = (0.0, 0.0, 151.9)
LONG_BRAZO_MM = 243.65 + 213.25 + 131.05 + 85.35 + 92.1
VEL_LIN_JOG_MAX = 150.0         # mm/s con velocidad al 100 %
VEL_LIN_MAX = 250.0
ACEL_LIN_JOG = 0.25
ACEL_ROT_JOG = 1.0
AVISO_DIST_MM = 100.0
AVISO_GIRO = 30.0
MARGEN_SINGULAR = 15.0          # cerca de una singularidad se BLOQUEAN movel/speedl (ver fallo C306A3)

# Gripper Robotiq
REG_GRIPPER = 128               # 128 STA, 129 POS, 130 OBJ, 131 paso del script, 132 nº de orden terminada
FUERZAS = {"Baja": 50, "Media": 150, "Alta": 255}
VEL_GRIPPER = 200
GRIPPER_ABIERTO, GRIPPER_CERRADO = 0, 255

PERIODO_JOG_MS = 100
VIDA_ORDEN_JOG = 0.3

# Posiciones guardadas (archivo junto al programa)
CARPETA = os.path.dirname(os.path.abspath(__file__))
ARCHIVO_POS = os.path.join(CARPETA, "posiciones_ur3e.json")
CARPETA_CAPTURAS = os.path.join(CARPETA, "capturas")
MAX_CAMARAS = 4

# Seguimiento de mano (MediaPipe Hand Landmarker)
MODELO_MANO = os.path.join(CARPETA, "modelos", "hand_landmarker.task")
ZONA_MUERTA = 0.18              # fracción de media pantalla en la que la mano no mueve el robot
VIDA_VEL_MANO = 0.3            # s: si el visor deja de actualizar, se ignora la última velocidad
HOME_FABRICA = [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]   # Home por defecto de Universal Robots (brazo vertical)

MODOS_ROBOT = {
    -1: "Sin controlador", 0: "Desconectado", 1: "Confirmar seguridad", 2: "Arrancando",
    3: "Brazo apagado", 4: "Encendido", 5: "Frenos puestos", 6: "Backdrive", 7: "Funcionando",
    8: "Actualizando firmware",
}
MODO_FUNCIONANDO = 7
MODO_ARTICULACION_OK = 253

# Colores y tipografía
F = "Segoe UI"
BG = "#F3F5F8"
CARD = "#FFFFFF"
BORDE = "#E3E7ED"
GRIS = "#EEF1F5"
TEXTO = "#1F2937"
SUAVE = "#6B7280"
DESACT = "#B8BEC8"
ACENTO = "#2563EB"
ACENTO_SUAVE = "#E3ECFD"
ROJO = "#DC2626"
ROJO_OSCURO = "#991B1B"
AMBAR = "#B45309"
PASTILLAS = {
    "ok": ("#DCFCE7", "#15803D"), "mov": ("#DBEAFE", ACENTO), "aviso": ("#FEF3C7", AMBAR),
    "error": ("#FEE2E2", "#B91C1C"), "neutro": (GRIS, SUAVE),
}


# ----------------------------------------------------------------------------- geometría

def rotvec_a_matriz(r):
    th = math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)
    if th < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    kx, ky, kz = r[0] / th, r[1] / th, r[2] / th
    c, s = math.cos(th), math.sin(th)
    v = 1 - c
    return [[kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s],
            [ky * kx * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s],
            [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c]]


def matriz_a_rotvec(R):
    c = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return [0.0, 0.0, 0.0]
    if math.pi - th < 1e-4:
        x = math.sqrt(max(0.0, (R[0][0] + 1) / 2))
        y = math.sqrt(max(0.0, (R[1][1] + 1) / 2))
        z = math.sqrt(max(0.0, (R[2][2] + 1) / 2))
        if x >= y and x >= z:
            y, z = math.copysign(y, R[0][1]), math.copysign(z, R[0][2])
        elif y >= z:
            x, z = math.copysign(x, R[0][1]), math.copysign(z, R[1][2])
        else:
            x, y = math.copysign(x, R[0][2]), math.copysign(y, R[1][2])
        return [th * x, th * y, th * z]
    s = 2 * math.sin(th)
    return [th * (R[2][1] - R[1][2]) / s, th * (R[0][2] - R[2][0]) / s, th * (R[1][0] - R[0][1]) / s]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpuesta(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def angulo_entre(R1, R2):
    R = matmul(transpuesta(R1), R2)
    return math.degrees(math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))))


def aviso_singularidad(q):
    avisos = []
    codo = (q[2] + 180) % 360 - 180
    mu2 = (q[4] + 180) % 360 - 180
    if abs(codo) < MARGEN_SINGULAR:
        avisos.append("codo casi estirado")
    if abs(mu2) < MARGEN_SINGULAR or abs(abs(mu2) - 180) < MARGEN_SINGULAR:
        avisos.append("Muñeca 2 cerca de 0°/180°")
    return " y ".join(avisos)


# ----------------------------------------------------------------------------- posiciones guardadas

def _es_q(v):
    return isinstance(v, list) and len(v) == 6 and all(isinstance(x, (int, float)) for x in v) \
        and all(abs(x) <= LIMITE_GRADOS for x in v)


def cargar_posiciones():
    """Devuelve (home, [ {nombre, q, tcp}, … ]) desde el archivo; valores por defecto si no existe."""
    try:
        with open(ARCHIVO_POS, encoding="utf-8") as f:
            datos = json.load(f)
    except (OSError, ValueError):
        datos = {}
    home = datos.get("home") if _es_q(datos.get("home")) else list(HOME_FABRICA)
    posiciones = [p for p in datos.get("posiciones", [])
                  if isinstance(p, dict) and isinstance(p.get("nombre"), str) and _es_q(p.get("q"))]
    return home, posiciones


def guardar_posiciones(home, posiciones):
    """Guarda en el archivo. Devuelve None si fue bien o el texto del error."""
    try:
        tmp = ARCHIVO_POS + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"home": home, "posiciones": posiciones}, f, ensure_ascii=False, indent=2)
        os.replace(tmp, ARCHIVO_POS)
        return None
    except OSError as e:
        return str(e)


ARCHIVO_ERRORES = os.path.join(CARPETA, "errores_ur3e.log")


def registrar_error(e):
    """Añade el error con su traza al archivo errores_ur3e.log (junto al programa)."""
    import traceback
    try:
        with open(ARCHIVO_ERRORES, "a", encoding="utf-8") as f:
            f.write(f"--- {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("".join(traceback.format_exception(type(e), e, e.__traceback__)))
    except OSError:
        pass


def texto_q(q):
    return "  ·  ".join(f"{x:.0f}" if abs(x - round(x)) < 0.05 else f"{x:.1f}" for x in q) + " °"


def texto_gripper(pos):
    if pos is None:
        return ""
    if pos <= 12:
        return "abierto"
    if pos >= 215:
        return "cerrado"
    return f"cerrado {round(100 * pos / 255)} %"


# ----------------------------------------------------------------------------- cámara

def detectar_camaras():
    """Devuelve [(índice, nombre)] de las cámaras que entregan imagen. Requiere OpenCV."""
    if cv2 is None:
        return []
    encontradas = []
    for i in range(MAX_CAMARAS):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        try:
            if cap.isOpened() and cap.read()[0]:
                encontradas.append(i)
        finally:
            cap.release()
    nombres = []
    if sys.platform == "win32" and encontradas:
        try:
            r = subprocess.run(["powershell", "-NoProfile", "-Command",
                                "Get-PnpDevice -PresentOnly -Class Camera,Image -ErrorAction SilentlyContinue | "
                                "ForEach-Object { $_.FriendlyName }"],
                               capture_output=True, text=True, timeout=15,
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            nombres = [n.strip() for n in r.stdout.splitlines() if n.strip()]
        except (OSError, subprocess.SubprocessError):
            pass
    usar_nombres = len(nombres) == len(encontradas)
    return [(i, nombres[k] if usar_nombres else f"Cámara {i}") for k, i in enumerate(encontradas)]


class HiloCamara(threading.Thread):
    """Lee fotogramas de una cámara y, si tiene detector, busca la mano en cada uno (en segundo plano)."""

    def __init__(self, indice, detector=None, espejo=True):
        super().__init__(daemon=True)
        self.indice = indice
        self.detector = detector
        self.espejo = espejo
        self.frame = None               # imagen tal como se muestra (con espejo si procede)
        self.frame_original = None      # imagen sin espejo (para las capturas)
        self.mano = None                # resultado del detector para self.frame
        self.fps = 0.0
        self.error = None
        self._lock = threading.Lock()
        self._fin = threading.Event()

    def run(self):
        cap = cv2.VideoCapture(self.indice, cv2.CAP_DSHOW)
        if not cap.isOpened():
            self.error = "No se pudo abrir la cámara (¿la usa otro programa?)"
            return
        n, t0 = 0, time.time()
        try:
            while not self._fin.is_set():
                ok, f = cap.read()
                if not ok:
                    self.error = "La cámara dejó de enviar imagen"
                    time.sleep(0.2)
                    continue
                self.error = None
                vista = cv2.flip(f, 1) if self.espejo else f
                mano = None
                if self.detector is not None:
                    try:
                        mano = self.detector.detectar(vista)
                    except Exception as e:
                        registrar_error(e)
                with self._lock:
                    self.frame, self.frame_original, self.mano = vista, f, mano
                n += 1
                if time.time() - t0 >= 1.0:
                    self.fps, n, t0 = n / (time.time() - t0), 0, time.time()
        finally:
            cap.release()

    def ultimo(self):
        """(imagen mostrada, imagen original, mano) del último fotograma."""
        with self._lock:
            if self.frame is None:
                return None, None, None
            return self.frame.copy(), self.frame_original, self.mano

    def detener(self):
        self._fin.set()


CONEXIONES_MANO = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
                   (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]


class DetectorManoMP:
    """Detección de la mano con MediaPipe Hand Landmarker (21 puntos por mano)."""

    def __init__(self):
        self.det = None
        self.error = None
        self.mp = None
        self.t0 = time.time()
        self.ultimo_ts = -1
        self._lock = threading.Lock()
        os.environ.setdefault("GLOG_minloglevel", "2")
        try:
            import mediapipe as mp
            from mediapipe.tasks.python import BaseOptions, vision
        except Exception:
            self.error = "Falta MediaPipe. Instálalo con: python -m pip install mediapipe"
            return
        if not os.path.exists(MODELO_MANO):
            self.error = f"Falta el modelo de la mano: {MODELO_MANO}"
            return
        try:
            opciones = vision.HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=MODELO_MANO),
                running_mode=vision.RunningMode.VIDEO, num_hands=1,
                min_hand_detection_confidence=0.6, min_hand_presence_confidence=0.6,
                min_tracking_confidence=0.5)
            self.det = vision.HandLandmarker.create_from_options(opciones)
            self.mp = mp
        except Exception as e:
            self.error = f"No se pudo cargar MediaPipe: {e}"
            registrar_error(e)

    def detectar(self, bgr):
        """Devuelve {"x", "y": centro de la palma (0..1), "puntos": 21 (x, y) normalizados} o None."""
        if self.det is None:
            return None
        with self._lock:
            ts = max(int((time.time() - self.t0) * 1000), self.ultimo_ts + 1)
            self.ultimo_ts = ts
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            r = self.det.detect_for_video(self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), ts)
        if not r.hand_landmarks:
            return None
        puntos = [(p.x, p.y) for p in r.hand_landmarks[0]]
        palma = (0, 5, 9, 13, 17)       # muñeca y nudillos
        return {"x": sum(puntos[i][0] for i in palma) / 5, "y": sum(puntos[i][1] for i in palma) / 5,
                "puntos": puntos}

    def cerrar(self):
        with self._lock:
            if self.det is not None:
                try:
                    self.det.close()
                except Exception:
                    pass
                self.det = None


def fraccion_velocidad(d, zona=ZONA_MUERTA):
    """d en [-1, 1] (distancia al centro) -> fracción de velocidad en [-1, 1] con zona muerta."""
    a = abs(d)
    if a < zona:
        return 0.0
    return math.copysign(min(1.0, (a - zona) / (1 - zona)), d)


def frame_a_photo(img):
    """Convierte una imagen BGR de OpenCV en tk.PhotoImage sin bibliotecas extra (formato PPM)."""
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    datos = b"P6 %d %d 255\n" % (w, h) + rgb.tobytes()
    try:
        return tk.PhotoImage(width=w, height=h, data=datos, format="PPM")
    except tk.TclError:
        return tk.PhotoImage(width=w, height=h, data=base64.b64encode(datos), format="PPM")


# ----------------------------------------------------------------------------- red

def recibir(s, n):
    datos = bytearray()
    while len(datos) < n:
        trozo = s.recv(n - len(datos))
        if not trozo:
            raise ConnectionError("el robot cerró la conexión")
        datos += trozo
    return bytes(datos)


def leer_estado(s):
    """Lee mensajes del puerto 30002 hasta encontrar uno de estado del robot (tipo 16)."""
    while True:
        tam, tipo = struct.unpack("!iB", recibir(s, 5))
        if not 5 <= tam < 100_000:
            raise ConnectionError(f"mensaje inesperado ({tam} bytes)")
        cuerpo = recibir(s, tam - 5)
        if tipo != 16:
            continue
        estado = {}
        pos = 0
        while pos + 5 <= len(cuerpo):
            st, sty = struct.unpack_from("!iB", cuerpo, pos)
            if st <= 0:
                break
            if sty == 0 and st >= 22:
                estado["emergencia"] = bool(cuerpo[pos + 16])
                estado["proteccion"] = bool(cuerpo[pos + 17])
                estado["programa"] = bool(cuerpo[pos + 18])
                estado["modo"] = struct.unpack_from("!b", cuerpo, pos + 20)[0]
                if st >= 30:
                    estado["velocidad_pct"] = struct.unpack_from("!d", cuerpo, pos + 22)[0] * 100
            elif sty == 1 and st >= 5 + 6 * 41:
                q, modos = [], []
                for j in range(6):
                    base = pos + 5 + j * 41
                    q.append(math.degrees(struct.unpack_from("!d", cuerpo, base)[0]))
                    modos.append(cuerpo[base + 40])
                estado["q"] = q
                estado["modos_art"] = modos
            elif sty == 4 and st >= 5 + 6 * 8:
                estado["tcp"] = list(struct.unpack_from("!6d", cuerpo, pos + 5))
                if st >= 5 + 12 * 8:
                    estado["tcp_offset"] = list(struct.unpack_from("!6d", cuerpo, pos + 5 + 48))
            pos += st
        if "q" in estado:
            return estado


def enviar_script(ip, script):
    with socket.create_connection((ip, PUERTO), timeout=3) as s:
        s.sendall(script.encode("utf-8"))


def script_movej(q_grados, vel, acel):
    q = ", ".join(f"{math.radians(x):.6f}" for x in q_grados)
    return ("def mover_desde_pc():\n"
            f"  movej([{q}], a={math.radians(acel):.5f}, v={math.radians(vel):.5f})\n"
            "end\n")


def script_movel(p_mm, rotvec_rad, vel_mm, acel_mm):
    p = ", ".join([f"{x / 1000:.6f}" for x in p_mm] + [f"{r:.6f}" for r in rotvec_rad])
    return ("def mover_tcp_desde_pc():\n"
            f"  movel(p[{p}], a={acel_mm / 1000:.4f}, v={vel_mm / 1000:.4f})\n"
            "end\n")


def script_speedj(qd_grados, acel, vida):
    qd = ", ".join(f"{math.radians(x):.5f}" for x in qd_grados)
    return f"def jog_desde_pc():\n  speedj([{qd}], {math.radians(acel):.4f}, {vida})\nend\n"


def script_speedl(xd, vida):
    v = ", ".join(f"{x:.5f}" for x in xd)
    return f"def jog_tcp_desde_pc():\n  speedl([{v}], {ACEL_LIN_JOG}, {vida}, {ACEL_ROT_JOG})\nend\n"


SCRIPT_PARAR = "def parar_desde_pc():\n  stopj(3.0)\nend\n"
SCRIPT_PARAR_L = "def parar_tcp_desde_pc():\n  stopl(0.5)\nend\n"


def script_gripper(seq, pos=None, vel=VEL_GRIPPER, fuerza=150, antes=""):
    """Script que habla con la URCap de Robotiq desde dentro del robot.
    pos=None solo consulta el estado. Con pos (0 abierto … 255 cerrado) activa si hace falta y mueve.
    `antes`: líneas de URScript que se ejecutan primero (p. ej. un movej para ir a una posición).
    Deja STA, POS, OBJ y el paso en los registros Modbus 128-131 y, al terminar, `seq` en el 132."""
    r = REG_GRIPPER
    seq = int(seq) % 60000
    accion = ""
    if pos is not None:
        pos, vel, fuerza = (int(max(0, min(255, x))) for x in (pos, vel, fuerza))
        accion = f"""
    if sta != 3:
      reg({r + 3}, 4)
      rq_set("SET ACT 1")
      t = 0
      while sta != 3 and t < 100:
        sleep(0.1)
        sta = rq_get("STA")
        t = t + 1
      end
    end
    if sta == 3:
      reg({r + 3}, 5)
      rq_set("SET POS {pos} SPE {vel} FOR {fuerza} GTO 1")
      sleep(0.3)
      t = 0
      obj = 0
      while obj == 0 and t < 120:
        sleep(0.05)
        obj = rq_get("OBJ")
        t = t + 1
      end
    end"""
    return f"""def gripper_desde_pc():
  def reg(n, v):
    if v < 0:
      v = 65535
    end
    write_port_register(n, v)
  end
  def rq_set(cmd):
    socket_send_line(cmd, "rq")
    socket_read_byte_list(3, "rq", 1.0)
  end
  def rq_get(var):
    socket_send_line(str_cat("GET ", var), "rq")
    l = socket_read_line("rq", 1.0)
    i = str_find(l, " ")
    if i > 0:
      return to_num(str_sub(l, i + 1))
    end
    return -1
  end
{antes}  reg({r + 3}, 1)
  if socket_open("127.0.0.1", 63352, "rq"):
    reg({r + 3}, 2)
    sta = rq_get("STA"){accion}
    reg({r}, rq_get("STA"))
    reg({r + 1}, rq_get("POS"))
    reg({r + 2}, rq_get("OBJ"))
    socket_close("rq")
    reg({r + 3}, 3)
  else:
    reg({r + 3}, 9)
  end
  reg({r + 4}, {seq})
end
"""


class Lector(threading.Thread):
    """Conexión permanente al puerto 30002: guarda el último estado y permite enviar scripts."""

    def __init__(self, ip):
        super().__init__(daemon=True)
        self.ip = ip
        self.estado = None
        self.error = None
        self._sock = None
        self._lock = threading.Lock()
        self._fin = threading.Event()

    def run(self):
        while not self._fin.is_set():
            try:
                with socket.create_connection((self.ip, PUERTO), timeout=3) as s:
                    self._sock = s
                    try:
                        while not self._fin.is_set():
                            self.estado = leer_estado(s)
                            self.error = None
                    finally:
                        self._sock = None
            except (OSError, struct.error) as e:
                self.estado = None
                self.error = str(e) or e.__class__.__name__
                self._fin.wait(1.5)

    def enviar(self, script):
        with self._lock:
            s = self._sock
            if s is None:
                raise ConnectionError("sin conexión con el robot")
            s.sendall(script.encode("utf-8"))

    def detener(self):
        self._fin.set()


class LectorModbus(threading.Thread):
    """Lee cada 0,3 s los registros del gripper (128-132) por Modbus TCP."""

    def __init__(self, ip):
        super().__init__(daemon=True)
        self.ip = ip
        self.regs = None
        self._fin = threading.Event()

    def run(self):
        tid = 0
        while not self._fin.is_set():
            try:
                with socket.create_connection((self.ip, PUERTO_MODBUS), timeout=3) as m:
                    while not self._fin.is_set():
                        tid = (tid + 1) & 0xFFFF
                        m.sendall(struct.pack("!HHHBBHH", tid, 0, 6, 0, 3, REG_GRIPPER, 5))
                        _, _, _, _, fc, nb = struct.unpack("!HHHBBB", recibir(m, 9))
                        if fc & 0x80:
                            raise ConnectionError(f"Modbus error {nb}")
                        self.regs = struct.unpack(f"!{nb // 2}H", recibir(m, nb))
                        self._fin.wait(0.3)
            except (OSError, struct.error):
                self.regs = None
                self._fin.wait(2.0)

    def detener(self):
        self._fin.set()


# ----------------------------------------------------------------------------- piezas de interfaz

def boton(padre, texto, comando, bg=GRIS, fg=TEXTO, activo=None, fuente=(F, 10, "bold"), padx=14, pady=7):
    return tk.Button(padre, text=texto, command=comando, bg=bg, fg=fg, activebackground=activo or bg,
                     activeforeground=fg, disabledforeground=DESACT, relief="flat", bd=0, font=fuente,
                     padx=padx, pady=pady, cursor="hand2", takefocus=0, highlightthickness=0)


def tarjeta(padre, **kw):
    return tk.Frame(padre, bg=CARD, highlightbackground=BORDE, highlightthickness=1, padx=18, pady=16, **kw)


class Segmentado(tk.Frame):
    """Selector tipo «píldora» con varias opciones."""

    def __init__(self, padre, opciones, valor, al_cambiar, pequeno=False):
        super().__init__(padre, bg=GRIS, padx=3, pady=3)
        self.valor = valor
        self.al_cambiar = al_cambiar
        self.botones = {}
        for clave, texto in opciones:
            b = tk.Label(self, text=texto, font=(F, 9 if pequeno else 10, "bold"), cursor="hand2",
                         padx=10 if pequeno else 18, pady=3 if pequeno else 6)
            b.pack(side="left")
            b.bind("<Button-1>", lambda e, c=clave: self.elegir(c))
            self.botones[clave] = b
        self._pintar()

    def elegir(self, clave):
        if clave != self.valor:
            self.valor = clave
            self._pintar()
            self.al_cambiar(clave)

    def _pintar(self):
        for c, b in self.botones.items():
            sel = c == self.valor
            b.config(bg=CARD if sel else GRIS, fg=ACENTO if sel else SUAVE)


# ----------------------------------------------------------------------------- visor de cámara

class VentanaCamara:
    """Ventana con la imagen en vivo, seguimiento de mano (MediaPipe) y captura de fotos."""

    def __init__(self, app, camaras):
        self.app = app
        self.camaras = dict(camaras)
        self.hilo = None
        self.foto = None
        self.cruz = False
        self.indice = camaras[0][0]
        self.detector = DetectorManoMP()
        self.seguir = False
        self.enganchado = False         # la mano pasó por el centro tras activar / tras perderla
        self.espejo = True
        self.invertir = False
        self.pos = None                 # centro de la palma suavizado (0..1)

        w = self.win = tk.Toplevel(app.root)
        w.title("Cámara")
        w.configure(bg=BG)
        w.geometry("800x740")
        w.minsize(560, 520)
        w.protocol("WM_DELETE_WINDOW", self.cerrar)

        barra = tk.Frame(w, bg=CARD, highlightbackground=BORDE, highlightthickness=1)
        barra.pack(fill="x")
        tk.Label(barra, text="Cámara", bg=CARD, fg=TEXTO, font=(F, 13, "bold")).pack(side="left", padx=(16, 12), pady=10)
        if len(camaras) > 1:
            Segmentado(barra, [(str(i), n) for i, n in camaras], str(self.indice),
                       lambda c: self.abrir(int(c)), pequeno=True).pack(side="left")
        else:
            tk.Label(barra, text=camaras[0][1], bg=CARD, fg=SUAVE, font=(F, 10)).pack(side="left")
        self.lbl_fps = tk.Label(barra, text="", bg=CARD, fg=SUAVE, font=(F, 9))
        self.lbl_fps.pack(side="right", padx=16)

        self.lienzo = tk.Canvas(w, bg="#111827", highlightthickness=0)
        self.lienzo.pack(fill="both", expand=True, padx=16, pady=12)
        self.id_img = self.lienzo.create_image(0, 0, anchor="center")
        self.id_txt = self.lienzo.create_text(0, 0, text="Abriendo cámara…", fill="#9CA3AF", font=(F, 11))

        pie = tk.Frame(w, bg=BG)
        pie.pack(fill="x", padx=16, pady=(0, 14))
        self.btn_cruz = boton(pie, "Cruz central", self.alternar_cruz, bg=GRIS, fg=SUAVE)
        self.btn_cruz.pack(side="left")
        boton(pie, "Capturar foto", self.capturar, bg=ACENTO, fg="white", activo="#1D4ED8").pack(side="right")
        boton(pie, "Abrir carpeta", self.abrir_carpeta, fg=SUAVE).pack(side="right", padx=8)
        self.lbl_info = tk.Label(pie, text="Las fotos se guardan con la posición del robot (JSON).",
                                 bg=BG, fg=SUAVE, font=(F, 9))
        self.lbl_info.pack(side="left", padx=12)

        # Seguimiento de mano
        sm = tk.Frame(w, bg=CARD, highlightbackground=BORDE, highlightthickness=1, padx=14, pady=10)
        sm.pack(fill="x", padx=16, pady=(0, 14))
        f1 = tk.Frame(sm, bg=CARD)
        f1.pack(fill="x")
        tk.Label(f1, text="Seguir la mano", bg=CARD, fg=TEXTO, font=(F, 11, "bold")).pack(side="left", padx=(0, 12))
        self.btn_seguir = boton(f1, "", self.alternar_seguir, pady=5, padx=14)
        self.btn_seguir.pack(side="left")
        self.btn_invertir = boton(f1, "Invertir sentido", self.alternar_invertir, pady=5, padx=10)
        self.btn_invertir.pack(side="right")
        self.btn_espejo = boton(f1, "Espejo", self.alternar_espejo, pady=5, padx=10)
        self.btn_espejo.pack(side="right", padx=6)
        f2 = tk.Frame(sm, bg=CARD)
        f2.pack(fill="x", pady=(8, 0))
        opciones = ["Nada"] + NOMBRES
        tk.Label(f2, text="Izquierda / derecha mueve:", bg=CARD, fg=SUAVE, font=(F, 9)).pack(side="left")
        self.eje_x = ttk.Combobox(f2, values=opciones, state="readonly", width=10)
        self.eje_x.set("Base")
        self.eje_x.pack(side="left", padx=(4, 14))
        tk.Label(f2, text="Arriba / abajo mueve:", bg=CARD, fg=SUAVE, font=(F, 9)).pack(side="left")
        self.eje_y = ttk.Combobox(f2, values=opciones, state="readonly", width=10)
        self.eje_y.set("Nada")
        self.eje_y.pack(side="left", padx=4)
        self.lbl_mano = tk.Label(sm, bg=CARD, fg=SUAVE, font=(F, 9), anchor="w",
                                 text="Pulsa «Activar seguimiento», pon la mano en el centro y muévela a los lados.")
        self.lbl_mano.pack(fill="x", pady=(8, 0))
        if self.detector.error:
            self.lbl_mano.config(text=self.detector.error, fg=ROJO)
        self._pintar_botones_mano()

        self.abrir(self.indice)
        self.tick()

    def viva(self):
        try:
            return bool(self.win.winfo_exists())
        except tk.TclError:
            return False

    def abrir(self, indice):
        if self.hilo:
            self.hilo.detener()
        self.indice = indice
        self.hilo = HiloCamara(indice, self.detector if self.detector.det else None, self.espejo)
        self.hilo.start()

    def alternar_cruz(self):
        self.cruz = not self.cruz
        self.btn_cruz.config(bg=ACENTO if self.cruz else GRIS, fg="white" if self.cruz else SUAVE,
                             activebackground=ACENTO if self.cruz else GRIS)

    def tick(self):
        if not self.viva():
            return
        try:
            self._tick()
        except Exception as e:      # nunca dejar el visor congelado: mostrar y registrar el error
            registrar_error(e)
            self.lbl_mano.config(text=f"Error en el visor: {e} (detalles en errores_ur3e.log)", fg=ROJO)
        self.win.after(33, self.tick)

    def _tick(self):
        cw, ch = max(50, self.lienzo.winfo_width()), max(50, self.lienzo.winfo_height())
        vista, _original, mano = self.hilo.ultimo() if self.hilo else (None, None, None)
        if vista is not None:
            qd = self._procesar_mano(mano)
            fh, fw = vista.shape[:2]
            s = min(cw / fw, ch / fh)
            vis = vista
            if abs(s - 1) > 0.01:
                vis = cv2.resize(vista, (max(1, int(fw * s)), max(1, int(fh * s))),
                                 interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
            vh, vw = vis.shape[:2]
            cx, cy = vw // 2, vh // 2
            if self.cruz:
                cv2.line(vis, (0, cy), (vw, cy), (80, 220, 80), 1)
                cv2.line(vis, (cx, 0), (cx, vh), (80, 220, 80), 1)
                cv2.circle(vis, (cx, cy), max(10, vh // 12), (80, 220, 80), 1)
            self._dibujar_mano(vis, mano, qd)
            self.foto = frame_a_photo(vis)
            self.lienzo.itemconfig(self.id_img, image=self.foto)
            self.lienzo.itemconfig(self.id_txt, text="")
            self.lbl_fps.config(text=f"{fw}×{fh}  ·  {self.hilo.fps:.0f} fps")
        elif self.hilo and self.hilo.error:
            self.lienzo.itemconfig(self.id_txt, text=self.hilo.error)
        self.lienzo.coords(self.id_img, cw // 2, ch // 2)
        self.lienzo.coords(self.id_txt, cw // 2, ch // 2)

    # --- seguimiento de mano
    def _pintar_botones_mano(self):
        def estilo(b, activo, texto=None, color=ACENTO):
            bg, fg = (color, "white") if activo else (GRIS, SUAVE)
            b.config(bg=bg, fg=fg, activebackground=bg, activeforeground=fg, **({"text": texto} if texto else {}))
        if self.detector.error:
            estilo(self.btn_seguir, False, "Seguimiento no disponible")
            self.btn_seguir.config(state="disabled")
        elif self.seguir:
            estilo(self.btn_seguir, True, "Detener seguimiento", "#16A34A")
        else:
            estilo(self.btn_seguir, True, "Activar seguimiento")
        estilo(self.btn_espejo, self.espejo)
        estilo(self.btn_invertir, self.invertir)

    def alternar_espejo(self):
        self.espejo = not self.espejo
        if self.hilo:
            self.hilo.espejo = self.espejo
        self.pos = None
        self.enganchado = False
        self._pintar_botones_mano()

    def alternar_invertir(self):
        self.invertir = not self.invertir
        self._pintar_botones_mano()

    def alternar_seguir(self):
        if self.seguir:
            self.desactivar_seguimiento()
            self.lbl_mano.config(text="Seguimiento detenido.", fg=SUAVE)
            return
        if not self.app.robot_listo(self.app.est()):
            self.lbl_mano.config(text="El robot no está listo (estado Normal y modo Remoto).", fg=AMBAR)
            return
        ejes = [e for e in (self.eje_x.get(), self.eje_y.get()) if e != "Nada"]
        if not ejes:
            self.lbl_mano.config(text="Elige qué articulación mueve cada dirección.", fg=AMBAR)
            return
        if not messagebox.askyesno("Seguir la mano",
                                   f"El robot moverá {' y '.join(ejes)} siguiendo tu mano.\n"
                                   f"Velocidad máxima: {self.app.vel_giro():.0f} °/s (barra de velocidad).\n\n"
                                   "Para empezar, pon la mano en el centro de la imagen.\n"
                                   "Si la mano sale de la imagen, el robot se para.\n"
                                   "Espacio o PARAR lo detienen.\n\n¿Activar?", parent=self.win):
            return
        self.seguir = True
        self.enganchado = False
        self._pintar_botones_mano()

    def desactivar_seguimiento(self):
        self.seguir = False
        self.enganchado = False
        self.app.vel_mano = None
        if self.viva():
            self._pintar_botones_mano()

    def _procesar_mano(self, mano):
        """Suaviza la posición de la palma, gestiona el «embrague» y publica la velocidad en la App."""
        if mano is None:
            self.pos = None
            self.enganchado = False
        else:
            nx, ny = mano["x"], mano["y"]
            if self.pos is not None:
                nx, ny = 0.5 * self.pos[0] + 0.5 * nx, 0.5 * self.pos[1] + 0.5 * ny
            self.pos = (nx, ny)
            if not self.enganchado and abs(nx - 0.5) * 2 < ZONA_MUERTA and abs(ny - 0.5) * 2 < ZONA_MUERTA:
                self.enganchado = True
        qd = [0.0] * 6
        if self.pos is not None and self.enganchado:
            vx = fraccion_velocidad((self.pos[0] - 0.5) * 2)
            vy = -fraccion_velocidad((self.pos[1] - 0.5) * 2)       # mano arriba = positivo
            if self.invertir:
                vx, vy = -vx, -vy
            vmax = self.app.vel_giro()
            for combo, v in ((self.eje_x, vx), (self.eje_y, vy)):
                if combo.get() in NOMBRES:
                    qd[NOMBRES.index(combo.get())] += v * vmax
        if self.seguir:
            self.app.vel_mano = qd
            self.app.vel_mano_t = time.time()
            if mano is None:
                texto, color = "No se ve la mano: robot parado. Vuelve a ponerla en el centro.", AMBAR
            elif not self.enganchado:
                texto, color = "Pon la mano en el centro (rectángulo) para empezar.", AMBAR
            elif any(qd):
                texto = "Siguiendo · " + " · ".join(f"{NOMBRES[i]} {v:+.0f} °/s" for i, v in enumerate(qd) if v)
                color = ACENTO
            else:
                texto, color = "Mano en el centro: robot quieto. Muévela a los lados.", "#15803D"
            if self.lbl_mano.cget("text") != texto:
                self.lbl_mano.config(text=texto, fg=color)
        return qd

    def _dibujar_mano(self, vis, mano, qd):
        vh, vw = vis.shape[:2]
        cx, cy = vw // 2, vh // 2
        if self.seguir:
            color_zona = (80, 200, 80) if self.enganchado else (0, 170, 255)
        else:
            color_zona = (170, 170, 170)
        zx, zy = int(ZONA_MUERTA * vw / 2), int(ZONA_MUERTA * vh / 2)
        cv2.rectangle(vis, (cx - zx, cy - zy), (cx + zx, cy + zy), color_zona, 1, cv2.LINE_AA)
        if mano is not None:
            pts = [(int(x * vw), int(y * vh)) for x, y in mano["puntos"]]
            for a, b in CONEXIONES_MANO:
                cv2.line(vis, pts[a], pts[b], (235, 235, 235), 1, cv2.LINE_AA)
            for p in pts:
                cv2.circle(vis, p, 2, (235, 140, 40), -1, cv2.LINE_AA)
            if self.pos is not None:
                px, py = int(self.pos[0] * vw), int(self.pos[1] * vh)
                cv2.circle(vis, (px, py), 6, (235, 99, 37), -1, cv2.LINE_AA)
                if any(qd):
                    cv2.arrowedLine(vis, (cx, cy), (px, py), (60, 60, 255), 2, cv2.LINE_AA, tipLength=0.08)
        if self.detector.error:
            self._banda(vis, "Seguimiento de mano no disponible", (120, 120, 255))
        elif not self.seguir:
            self._banda(vis, "Pulsa 'Activar seguimiento' para mover el robot con la mano")
        elif mano is None:
            self._banda(vis, "No se ve la mano: robot parado", (0, 170, 255))
        elif not self.enganchado:
            self._banda(vis, "Pon la mano en el centro para empezar", (0, 170, 255))
        else:
            self._banda(vis, "Siguiendo la mano", (80, 200, 80))

    @staticmethod
    def _banda(vis, texto, color=(255, 255, 255)):
        """Franja oscura con un texto en la parte superior de la imagen."""
        alto = 34
        franja = vis[0:alto, :]
        vis[0:alto, :] = (franja * 0.35).astype(franja.dtype)
        cv2.putText(vis, texto, (12, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)

    def capturar(self):
        _vista, f, _mano = self.hilo.ultimo() if self.hilo else (None, None, None)
        if f is None:
            self.lbl_info.config(text="Todavía no hay imagen.")
            return
        os.makedirs(CARPETA_CAPTURAS, exist_ok=True)
        ahora = time.time()
        marca = time.strftime("%Y%m%d_%H%M%S", time.localtime(ahora)) + f"_{int(ahora % 1 * 1000):03d}"
        base = os.path.join(CARPETA_CAPTURAS, f"captura_{marca}")
        ok, png = cv2.imencode(".png", f)
        if not ok:
            self.lbl_info.config(text="No se pudo codificar la imagen.")
            return
        try:
            with open(base + ".png", "wb") as fp:
                fp.write(png.tobytes())
            meta = {"fecha": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ahora)),
                    "imagen": os.path.basename(base) + ".png", "camara": self.camaras.get(self.indice),
                    "ancho": int(f.shape[1]), "alto": int(f.shape[0]), "robot": self.app.estado_para_registro()}
            with open(base + ".json", "w", encoding="utf-8") as fp:
                json.dump(meta, fp, ensure_ascii=False, indent=2)
        except OSError as e:
            self.lbl_info.config(text=f"No se pudo guardar: {e}")
            return
        self.lbl_info.config(text=f"Guardada: capturas\\{os.path.basename(base)}.png")
        self.app.mensaje.set(f"Foto guardada en {base}.png (con la posición del robot en .json)")

    def abrir_carpeta(self):
        os.makedirs(CARPETA_CAPTURAS, exist_ok=True)
        if hasattr(os, "startfile"):
            os.startfile(CARPETA_CAPTURAS)

    def cerrar(self):
        self.seguir = False
        self.app.vel_mano = None
        if self.hilo:
            self.hilo.detener()
            self.hilo.join(timeout=1.0)
        self.detector.cerrar()
        try:
            self.win.destroy()
        except tk.TclError:
            pass


# ----------------------------------------------------------------------------- aplicación

class App:
    def __init__(self, root, ip):
        self.root = root
        self.ip = ip
        self.lector = None
        self.modbus = None
        self.sincronizado = False
        self.modo = "art"
        self.referencia = "base"
        self.teclado = False
        self.pulsadas = set()           # teclas de movimiento mantenidas
        self.raton = set()              # botones − / + mantenidos con el ratón
        self.teclas_unicas = set()      # O / C ya pulsadas (evita repetir al mantener)
        self.jog_en_curso = None        # None, "art" o "tcp"
        self.vel_mano = None            # velocidades articulares (°/s) que pide el seguimiento de mano
        self.vel_mano_t = 0.0
        self.vel_pct = tk.IntVar(value=20)
        self.fuerza = "Media"
        self.gripper_t = 0.0
        self.gripper_seq = int(time.time()) % 50000
        self.gripper_esperado = None    # nº de la última orden enviada al gripper
        self.gripper_reintentado = False
        self.gripper_consultado = False
        self.dialogo = None
        self.home, self.posiciones = cargar_posiciones()
        self.camaras = None             # None = buscando; [] = ninguna; [(índice, nombre)]
        self.ventana_cam = None
        self._buscar_camaras()

        root.title("Control UR3e")
        root.configure(bg=BG)
        root.minsize(940, 640)
        root.protocol("WM_DELETE_WINDOW", self.cerrar)

        self._cabecera()
        cuerpo = tk.Frame(root, bg=BG)
        cuerpo.pack(fill="both", expand=True, padx=18, pady=(16, 6))
        izq = tarjeta(cuerpo)
        izq.pack(side="left", fill="both", expand=True)
        der = tk.Frame(cuerpo, bg=BG)
        der.pack(side="left", fill="y", padx=(16, 0))
        self._panel_movimiento(izq)
        self._panel_gripper(der)
        self.mensaje = tk.StringVar(value="Conectando con el robot…")
        tk.Label(root, textvariable=self.mensaje, bg=BG, fg=SUAVE, font=(F, 9), anchor="w").pack(
            fill="x", padx=20, pady=(0, 10))

        root.bind_all("<KeyPress>", self.tecla_abajo)
        root.bind_all("<KeyRelease>", self.tecla_arriba)
        root.bind_all("<Button-1>", self.clic, add="+")
        root.bind("<FocusOut>", lambda e: self.pulsadas.clear())
        self.cambiar_modo("art")
        self.conectar()
        self.refrescar()
        self.tick_jog()

    # --- construcción
    def _cabecera(self):
        cab = tk.Frame(self.root, bg=CARD, highlightbackground=BORDE, highlightthickness=1)
        cab.pack(fill="x")
        tk.Label(cab, text="UR3e", font=(F, 15, "bold"), bg=CARD, fg=TEXTO).pack(side="left", padx=(18, 12), pady=12)
        self.pastilla = tk.Label(cab, text="● Conectando…", font=(F, 10, "bold"), padx=12, pady=4)
        self.pastilla.pack(side="left")
        boton(cab, "■  PARAR", self.parar, bg=ROJO, fg="white", activo=ROJO_OSCURO,
              fuente=(F, 11, "bold"), padx=22, pady=8).pack(side="right", padx=(8, 18), pady=8)
        boton(cab, "⚙", self.dialogo_ajustes, fuente=(F, 12), padx=10, pady=4).pack(side="right")
        boton(cab, "?", self.dialogo_atajos, fuente=(F, 12, "bold"), padx=12, pady=4).pack(side="right", padx=(0, 8))
        self.btn_teclado = boton(cab, "", self.alternar_teclado, padx=14)
        self.btn_teclado.pack(side="right", padx=8)
        self._pintar_teclado()
        self.btn_cam = boton(cab, "", self.abrir_camara, padx=14)
        self.btn_cam.pack(side="right")
        self._pintar_camara()

    def _panel_movimiento(self, p):
        arriba = tk.Frame(p, bg=CARD)
        arriba.pack(fill="x")
        Segmentado(arriba, [("art", "Articulaciones"), ("tcp", "Herramienta")], "art",
                   self.cambiar_modo).pack(side="left")
        self.marco_ref = tk.Frame(arriba, bg=CARD)
        tk.Label(self.marco_ref, text="Ejes de la", bg=CARD, fg=SUAVE, font=(F, 9)).pack(side="left", padx=(0, 6))
        Segmentado(self.marco_ref, [("base", "Base"), ("herramienta", "Herramienta")], "base",
                   self.cambiar_ref, pequeno=True).pack(side="left")

        self.aviso = tk.Label(p, text="", bg=CARD, fg=AMBAR, font=(F, 9), anchor="w", justify="left")
        self.aviso.pack(fill="x", pady=(10, 0))

        filas = tk.Frame(p, bg=CARD)
        filas.pack(fill="x", pady=(4, 0))
        filas.columnconfigure(3, weight=1)
        self.lbl_nombre, self.lbl_valor, self.lbl_unidad = [], [], []
        self.btn_menos, self.btn_mas = [], []
        for i, (mas, menos) in enumerate(TECLAS):
            nombre = tk.Label(filas, bg=CARD, fg=TEXTO, font=(F, 11), width=10, anchor="w")
            nombre.grid(row=i, column=0, sticky="w", pady=4)
            valor = tk.Label(filas, text="—", bg=CARD, fg=TEXTO, font=(F, 17, "bold"), width=7, anchor="e")
            valor.grid(row=i, column=1, sticky="e")
            unidad = tk.Label(filas, bg=CARD, fg=SUAVE, font=(F, 10), width=3, anchor="w")
            unidad.grid(row=i, column=2, sticky="w", padx=(4, 0))
            tk.Label(filas, text=menos.upper(), bg=CARD, fg=DESACT, font=("Consolas", 9, "bold")).grid(
                row=i, column=4, padx=(0, 6))
            b_menos = self._boton_mantener(filas, "−", menos)
            b_menos.grid(row=i, column=5, padx=2)
            b_mas = self._boton_mantener(filas, "+", mas)
            b_mas.grid(row=i, column=6, padx=2)
            tk.Label(filas, text=mas.upper(), bg=CARD, fg=DESACT, font=("Consolas", 9, "bold")).grid(
                row=i, column=7, padx=(6, 0))
            self.lbl_nombre.append(nombre)
            self.lbl_valor.append(valor)
            self.lbl_unidad.append(unidad)
            self.btn_menos.append(b_menos)
            self.btn_mas.append(b_mas)

        tk.Frame(p, bg=BORDE, height=1).pack(fill="x", pady=(14, 12))
        vel = tk.Frame(p, bg=CARD)
        vel.pack(fill="x")
        tk.Label(vel, text="Velocidad", bg=CARD, fg=TEXTO, font=(F, 10, "bold")).pack(side="left")
        tk.Scale(vel, from_=1, to=100, orient="horizontal", variable=self.vel_pct, showvalue=False, length=240,
                 width=12, sliderlength=26, bg=ACENTO, troughcolor="#E2E7EF", highlightthickness=0, bd=0,
                 sliderrelief="flat", activebackground="#1D4ED8", takefocus=0,
                 command=lambda _v: self._pintar_vel()).pack(side="left", padx=12)
        self.lbl_vel = tk.Label(vel, bg=CARD, fg=SUAVE, font=(F, 9))
        self.lbl_vel.pack(side="left")
        self._pintar_vel()

        pie = tk.Frame(p, bg=CARD)
        pie.pack(fill="x", pady=(14, 0))
        tk.Label(pie, text="Mantén pulsado − / + para mover · suelta para parar", bg=CARD, fg=SUAVE,
                 font=(F, 9)).pack(side="left")
        enlace = tk.Label(pie, text="Ir a una posición exacta…", bg=CARD, fg=ACENTO, font=(F, 9, "underline"),
                          cursor="hand2")
        enlace.pack(side="right")
        enlace.bind("<Button-1>", lambda e: self.dialogo_posicion())

    def _boton_mantener(self, padre, texto, tecla):
        b = tk.Label(padre, text=texto, font=(F, 16, "bold"), width=4, pady=1, cursor="hand2",
                     bg=GRIS, fg=DESACT)
        b.bind("<ButtonPress-1>", lambda e: self.raton.add(tecla))
        b.bind("<ButtonRelease-1>", lambda e: self.raton.discard(tecla))
        b.bind("<Leave>", lambda e: self.raton.discard(tecla))
        return b

    def _panel_gripper(self, der):
        g = tarjeta(der)
        g.pack(fill="x")
        tk.Label(g, text="Gripper", bg=CARD, fg=TEXTO, font=(F, 12, "bold")).pack(anchor="w")
        self.g_estado = tk.Label(g, text="—", bg=CARD, fg=TEXTO, font=(F, 15, "bold"))
        self.g_estado.pack(anchor="w", pady=(10, 0))
        self.g_detalle = tk.Label(g, text="", bg=CARD, fg=SUAVE, font=(F, 9), wraplength=230, justify="left")
        self.g_detalle.pack(anchor="w")
        self.g_barra = tk.Canvas(g, width=236, height=8, bg=CARD, highlightthickness=0)
        self.g_barra.pack(pady=(10, 14))
        bt = tk.Frame(g, bg=CARD)
        bt.pack(fill="x")
        self.btn_abrir = boton(bt, "Abrir", lambda: self.gripper(GRIPPER_ABIERTO), bg=ACENTO_SUAVE, fg=ACENTO,
                               activo="#CCDCFB", fuente=(F, 11, "bold"), pady=10)
        self.btn_abrir.pack(side="left", expand=True, fill="x", padx=(0, 4))
        self.btn_cerrar = boton(bt, "Cerrar", lambda: self.gripper(GRIPPER_CERRADO), bg=ACENTO, fg="white",
                                activo="#1D4ED8", fuente=(F, 11, "bold"), pady=10)
        self.btn_cerrar.pack(side="left", expand=True, fill="x", padx=(4, 0))
        teclas = tk.Frame(g, bg=CARD)
        teclas.pack(fill="x")
        tk.Label(teclas, text="O", bg=CARD, fg=DESACT, font=("Consolas", 9, "bold")).pack(side="left", expand=True)
        tk.Label(teclas, text="C", bg=CARD, fg=DESACT, font=("Consolas", 9, "bold")).pack(side="left", expand=True)
        tk.Label(g, text="Fuerza", bg=CARD, fg=TEXTO, font=(F, 10, "bold")).pack(anchor="w", pady=(12, 6))
        Segmentado(g, [(k, k) for k in FUERZAS], self.fuerza, lambda v: setattr(self, "fuerza", v),
                   pequeno=True).pack(anchor="w")

        self._panel_posiciones(der)

    def _panel_posiciones(self, der):
        c = tarjeta(der)
        c.pack(fill="both", expand=True, pady=(16, 0))
        tk.Label(c, text="Posiciones", bg=CARD, fg=TEXTO, font=(F, 12, "bold")).pack(anchor="w")
        self.btn_home = boton(c, "⌂   Ir a Home", self.ir_home, bg=ACENTO_SUAVE, fg=ACENTO, activo="#CCDCFB",
                              fuente=(F, 11, "bold"), pady=8)
        self.btn_home.pack(fill="x", pady=(10, 2))
        self.lbl_home = tk.Label(c, text="", bg=CARD, fg=SUAVE, font=(F, 8))
        self.lbl_home.pack(anchor="w", pady=(0, 10))

        marco = tk.Frame(c, bg=CARD, highlightbackground=BORDE, highlightthickness=1)
        marco.pack(fill="both", expand=True)
        self.lista = tk.Listbox(marco, height=4, font=(F, 10), relief="flat", bd=0, highlightthickness=0,
                                selectbackground=ACENTO_SUAVE, selectforeground=ACENTO, activestyle="none",
                                fg=TEXTO, bg=CARD, exportselection=False)
        self.lista.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        barra = tk.Scrollbar(marco, command=self.lista.yview, width=12)
        barra.pack(side="right", fill="y")
        self.lista.config(yscrollcommand=barra.set)
        self.lista.bind("<Double-Button-1>", lambda e: self.ir_guardada())
        self.lbl_vacia = tk.Label(c, text="Aún no hay posiciones guardadas.", bg=CARD, fg=SUAVE, font=(F, 9))

        fila = tk.Frame(c, bg=CARD)
        fila.pack(fill="x", side="bottom", pady=(10, 0))
        self.btn_guardar = boton(fila, "＋ Guardar actual", self.guardar_actual, pady=6, padx=10)
        self.btn_guardar.pack(side="left")
        boton(fila, "Borrar", self.borrar_guardada, pady=6, padx=10, fg=SUAVE).pack(side="right")
        self.btn_ir = boton(fila, "Ir", self.ir_guardada, bg=ACENTO, fg="white", activo="#1D4ED8", pady=6, padx=16)
        self.btn_ir.pack(side="right", padx=6)
        self._pintar_posiciones()

    # --- conexión y estado
    def conectar(self, ip=None):
        if ip:
            self.ip = ip
        for hilo in (self.lector, self.modbus):
            if hilo:
                hilo.detener()
        self.sincronizado = False
        self.lector = Lector(self.ip)
        self.modbus = LectorModbus(self.ip)
        self.lector.start()
        self.modbus.start()

    def est(self):
        return self.lector.estado if self.lector else None

    def robot_listo(self, est):
        return (est is not None and est.get("modo") == MODO_FUNCIONANDO and "tcp" in est
                and not est.get("emergencia") and not est.get("proteccion")
                and all(m == MODO_ARTICULACION_OK for m in est.get("modos_art", [])))

    def enviar(self, script):
        try:
            self.lector.enviar(script)
        except (OSError, AttributeError):
            enviar_script(self.ip, script)

    def refrescar(self):
        est = self.est()
        listo = self.robot_listo(est)
        if listo and not self.sincronizado:
            self.sincronizado = True
            self.mensaje.set("Robot listo. Mantén pulsado − / + o activa el teclado.")
            if not self.gripper_consultado:
                self._consultar_gripper()
        elif not listo:
            self.sincronizado = False

        # Pastilla de estado
        if est is None:
            tipo, texto = "error", "Sin conexión"
        elif est.get("emergencia"):
            tipo, texto = "error", "Parada de emergencia"
        elif est.get("proteccion"):
            tipo, texto = "error", "Parada de protección"
        elif est.get("modo") != MODO_FUNCIONANDO:
            tipo, texto = "aviso", MODOS_ROBOT.get(est.get("modo"), f"modo {est.get('modo')}")
        elif self.jog_en_curso or est.get("programa"):
            tipo, texto = "mov", "En movimiento"
        else:
            tipo, texto = "ok", "Listo"
        if est and est.get("velocidad_pct", 100) < 99.5:
            texto += f" · panel al {est['velocidad_pct']:.0f} %"
        fondo, color = PASTILLAS[tipo]
        self.pastilla.config(text=f"●  {texto}", bg=fondo, fg=color)

        # Valores
        for i in range(6):
            if est is None:
                v = "—"
            elif self.modo == "art":
                v = f"{est['q'][i]:.1f}"
            elif "tcp" in est:
                x = est["tcp"][i]
                v = f"{x * 1000:.1f}" if i < 3 else f"{math.degrees(x):.1f}"
            else:
                v = "—"
            if v == "-0.0":
                v = "0.0"
            self.lbl_valor[i].config(text=v, fg=TEXTO if listo else SUAVE)

        # Aviso contextual
        aviso = ""
        if est is None and self.lector and self.lector.error:
            aviso = f"No hay conexión con {self.ip}: {self.lector.error}"
        elif est is not None and not listo:
            aviso = "Para mover: brazo encendido, frenos liberados (estado Normal) y modo Remoto en PolyScope."
        elif listo and self.modo == "tcp":
            sing = aviso_singularidad(est["q"])
            if sing:
                aviso = (f"⚠  Cerca de una singularidad ({sing}): movimientos de la herramienta bloqueados. "
                         "Dobla el codo y aleja la Muñeca 2 de 0°/180° en Articulaciones.")
        self.aviso.config(text=aviso)

        for b in (self.btn_abrir, self.btn_cerrar, self.btn_home, self.btn_ir):
            b.config(state="normal" if listo else "disabled")
        self.btn_guardar.config(state="normal" if est is not None else "disabled")
        self._pintar_gripper(listo, est)
        self._pintar_camara()
        self.root.after(200, self.refrescar)

    # --- cámara
    def _buscar_camaras(self):
        if cv2 is None:
            self.camaras = []
            return
        self.camaras = None

        def buscar():
            self.camaras = detectar_camaras()
        threading.Thread(target=buscar, daemon=True).start()

    def _pintar_camara(self):
        abierta = self.ventana_cam is not None and self.ventana_cam.viva()
        if cv2 is None:
            texto, bg, fg = "◉  Cámara: falta OpenCV", GRIS, SUAVE
        elif self.camaras is None:
            texto, bg, fg = "◉  Buscando cámara…", GRIS, SUAVE
        elif not self.camaras:
            texto, bg, fg = "◉  Sin cámara", GRIS, SUAVE
        else:
            n = len(self.camaras)
            texto = "◉  Cámara abierta" if abierta else ("◉  Cámara" if n == 1 else f"◉  {n} cámaras")
            bg, fg = (ACENTO, "white") if abierta else (ACENTO_SUAVE, ACENTO)
        if self.btn_cam.cget("text") != texto:
            self.btn_cam.config(text=texto, bg=bg, fg=fg, activebackground=bg, activeforeground=fg)

    def abrir_camara(self):
        if cv2 is None:
            messagebox.showinfo("Cámara", "Para ver la cámara hace falta OpenCV.\n\n"
                                          "Instálalo con:\n    python -m pip install opencv-python")
        elif self.camaras is None:
            self.mensaje.set("Buscando cámaras, espera un momento…")
        elif not self.camaras:
            self.mensaje.set("No se encontró ninguna cámara. Buscando de nuevo…")
            self._buscar_camaras()
        elif self.ventana_cam is not None and self.ventana_cam.viva():
            self.ventana_cam.win.deiconify()
            self.ventana_cam.win.lift()
        else:
            self.ventana_cam = VentanaCamara(self, self.camaras)

    # --- gripper
    def _orden_gripper(self, pos=None, antes="", fuerza=None):
        self.gripper_seq = (self.gripper_seq + 1) % 60000
        self.enviar(script_gripper(self.gripper_seq, pos, VEL_GRIPPER,
                                   FUERZAS.get(fuerza or self.fuerza, FUERZAS["Media"]), antes))
        self.gripper_esperado = self.gripper_seq
        self.gripper_t = time.time()
        self.gripper_reintentado = pos is None and self.gripper_reintentado

    def gripper_pendiente(self):
        regs = self.modbus.regs if self.modbus else None
        return self.gripper_esperado is not None and (regs is None or regs[4] != self.gripper_esperado)

    def _consultar_gripper(self):
        try:
            self._orden_gripper(None)
            self.gripper_consultado = True
        except OSError:
            pass

    def _pintar_gripper(self, listo, est):
        regs = self.modbus.regs if self.modbus else None
        frac = None
        if regs is None:
            estado, detalle = "Sin datos", "No se puede leer el estado (Modbus, puerto 502)."
        else:
            sta, pos, obj, paso, _seq = regs
            pendiente = self.gripper_pendiente()
            if pendiente and time.time() - self.gripper_t > 15:
                if self.gripper_reintentado:
                    self.gripper_esperado = None   # no insistir más: mostrar lo último que dejó el robot
                    pendiente = False
                elif listo and not self.jog_en_curso and not est.get("programa"):
                    self.gripper_reintentado = True
                    self._consultar_gripper()      # la orden anterior se interrumpió: consultar una vez
            pct = round(100 * min(pos, 255) / 255)
            frac = min(pos, 255) / 255
            if pendiente:
                estado = {4: "Activando…", 5: "Moviendo…"}.get(paso, "Esperando al robot…")
                detalle = "Se abre y cierra una vez para calibrarse." if paso == 4 else ""
            elif not self.gripper_consultado or paso == 0:
                estado, detalle, frac = "Sin datos", "Se consultará cuando el robot esté listo.", None
            elif paso == 9:
                estado, detalle, frac = "No responde", "¿Está instalada la URCap de Robotiq y el gripper conectado?", None
            elif sta != 3:
                estado, detalle = "Sin activar", "Se activará al pulsar Abrir o Cerrar (hace un ciclo completo)."
            elif obj == 2:
                estado, detalle = "Sujetando objeto", f"Cerrado al {pct} %"
            elif obj == 1:
                estado, detalle = "Objeto detectado", f"Al abrir · {pct} %"
            elif pos <= 12:
                estado, detalle = "Abierto", f"Posición {pct} %"
            elif pos >= 215:
                estado, detalle = "Cerrado", f"Posición {pct} % · sin objeto"
            else:
                estado, detalle = f"Cerrado al {pct} %", ""
        self.g_estado.config(text=estado)
        self.g_detalle.config(text=detalle)
        self.g_barra.delete("all")
        self.g_barra.create_rectangle(0, 0, 236, 8, fill=GRIS, width=0)
        if frac:
            self.g_barra.create_rectangle(0, 0, max(4, 236 * frac), 8, fill=ACENTO, width=0)

    def gripper(self, pos):
        if not self.robot_listo(self.est()):
            self.mensaje.set("El robot no está listo para mover el gripper.")
            return
        self.pulsadas.clear()
        self.raton.clear()
        self.jog_en_curso = None        # el script del gripper sustituye a cualquier movimiento en curso
        try:
            self._orden_gripper(pos)
        except OSError as e:
            self.mensaje.set(f"No se pudo enviar la orden al gripper: {e}")
            return
        accion = "Cerrando" if pos >= 128 else "Abriendo"
        self.mensaje.set(f"{accion} gripper (fuerza {self.fuerza.lower()})…")

    # --- modos y opciones
    def cambiar_modo(self, modo):
        self.modo = modo
        self.pulsadas.clear()
        self.raton.clear()
        tcp = modo == "tcp"
        if tcp:
            self.marco_ref.pack(side="right")
        else:
            self.marco_ref.pack_forget()
        for i in range(6):
            self.lbl_nombre[i].config(text=NOMBRES_TCP[i] if tcp else NOMBRES[i])
            self.lbl_unidad[i].config(text=UNIDADES_TCP[i] if tcp else "°")
        self._pintar_vel()

    def cambiar_ref(self, ref):
        self.referencia = ref
        self.pulsadas.clear()
        self.raton.clear()

    def vel_giro(self):
        return max(1.0, VEL_JOG_MAX * self.vel_pct.get() / 100)

    def vel_lineal(self):
        return max(2.0, VEL_LIN_JOG_MAX * self.vel_pct.get() / 100)

    def _pintar_vel(self):
        if self.modo == "tcp":
            t = f"{self.vel_pct.get()} %  ·  {self.vel_lineal():.0f} mm/s, giro {self.vel_giro():.0f} °/s"
        else:
            t = f"{self.vel_pct.get()} %  ·  {self.vel_giro():.0f} °/s"
        self.lbl_vel.config(text=t)

    def alternar_teclado(self):
        self.teclado = not self.teclado
        self.pulsadas.clear()
        self.root.focus_set()
        self._pintar_teclado()
        self.mensaje.set("Teclado activado: mantén la tecla de cada fila para mover." if self.teclado
                         else "Teclado desactivado.")

    def _pintar_teclado(self):
        if self.teclado:
            self.btn_teclado.config(text="⌨  Teclado activado", bg=ACENTO, fg="white",
                                    activebackground="#1D4ED8", activeforeground="white")
        else:
            self.btn_teclado.config(text="⌨  Teclado desactivado", bg=GRIS, fg=SUAVE,
                                    activebackground=GRIS, activeforeground=SUAVE)

    # --- teclado y ratón
    @staticmethod
    def es_campo_texto(widget):
        return isinstance(widget, (tk.Entry, tk.Spinbox))

    def clic(self, ev):
        if not self.es_campo_texto(ev.widget):
            try:
                ev.widget.winfo_toplevel().focus_set()
            except (AttributeError, tk.TclError):
                pass

    def tecla_abajo(self, ev):
        k = ev.keysym.lower()
        campo = self.es_campo_texto(ev.widget)
        if k == "escape" or (k == "space" and not campo):
            self.parar()
            return "break"
        if campo or not self.teclado:
            return None
        if k in TECLAS_TODAS:
            self.pulsadas.add(k)
            return "break"
        if k in ("z", "x"):
            self.vel_pct.set(max(1, min(100, self.vel_pct.get() + (-10 if k == "z" else 10))))
            self._pintar_vel()
            return "break"
        if k in ("o", "c"):
            if k not in self.teclas_unicas:
                self.teclas_unicas.add(k)
                self.gripper(GRIPPER_ABIERTO if k == "o" else GRIPPER_CERRADO)
            return "break"
        return None

    def tecla_arriba(self, ev):
        k = ev.keysym.lower()
        self.pulsadas.discard(k)
        self.teclas_unicas.discard(k)

    def tick_jog(self):
        try:
            est = self.est()
            listo = self.sincronizado and self.robot_listo(est)
            tcp = self.modo == "tcp"
            activas = (self.pulsadas if self.teclado else set()) | self.raton
            dirs = [0] * 6
            ocupado = self.gripper_pendiente() and time.time() - self.gripper_t < 12
            if listo and ocupado and activas:
                self.mensaje.set("Espera a que el gripper termine su movimiento…")
            if listo and not ocupado:
                dirs = [(mas in activas) - (menos in activas) for mas, menos in TECLAS]
                if not tcp:
                    for i in range(6):
                        q = est["q"][i]
                        if (dirs[i] > 0 and q > LIMITE_GRADOS - MARGEN_LIMITE) or \
                                (dirs[i] < 0 and q < -LIMITE_GRADOS + MARGEN_LIMITE):
                            dirs[i] = 0
            # Seguimiento de mano: solo si no hay control manual en ese momento
            qd_mano = None
            if listo and not ocupado and not any(dirs) and self.vel_mano is not None \
                    and time.time() - self.vel_mano_t < VIDA_VEL_MANO:
                qd_mano = []
                for i, v in enumerate(self.vel_mano):
                    q = est["q"][i]
                    fuera = (v > 0 and q > LIMITE_GRADOS - MARGEN_LIMITE) or (v < 0 and q < -LIMITE_GRADOS + MARGEN_LIMITE)
                    qd_mano.append(0.0 if fuera or abs(v) < 0.05 else v)
                if not any(qd_mano):
                    qd_mano = None
            marcas = dirs if (qd_mano is None or tcp) else [(v > 0) - (v < 0) for v in qd_mano]
            for i in range(6):
                for b, activo in ((self.btn_menos[i], marcas[i] < 0), (self.btn_mas[i], marcas[i] > 0)):
                    if activo:
                        b.config(bg=ACENTO, fg="white")
                    elif listo:
                        b.config(bg=ACENTO_SUAVE, fg=ACENTO)
                    else:
                        b.config(bg=GRIS, fg=DESACT)

            if tcp and any(dirs) and aviso_singularidad(est["q"]):
                # En una singularidad, seguir una línea recta exige aceleraciones imposibles y el
                # robot se detiene con un fallo (C306A3). Bloqueamos y, si ya se movía, se frena.
                dirs = [0] * 6
                self.mensaje.set("Herramienta bloqueada: el robot está cerca de una singularidad. "
                                 "Dobla el codo y aleja la Muñeca 2 de 0°/180° en la pestaña Articulaciones.")
            tipo = ("tcp" if tcp else "art") if any(dirs) else ("art" if qd_mano else None)
            if self.jog_en_curso and tipo != self.jog_en_curso:
                self.enviar(SCRIPT_PARAR_L if self.jog_en_curso == "tcp" else SCRIPT_PARAR)
                self.jog_en_curso = None
            elif tipo == "art":
                qd = [d * self.vel_giro() for d in dirs] if any(dirs) else qd_mano
                self.enviar(script_speedj(qd, ACEL_JOG, VIDA_ORDEN_JOG))
                self.jog_en_curso = "art"
            elif tipo == "tcp":
                lin = self.vel_lineal() / 1000
                giro = math.radians(self.vel_giro())
                v_lin = [d * lin for d in dirs[:3]]
                v_rot = [d * giro for d in dirs[3:]]
                if self.referencia == "herramienta":
                    R = rotvec_a_matriz(est["tcp"][3:])
                    v_lin, v_rot = matvec(R, v_lin), matvec(R, v_rot)
                self.enviar(script_speedl(v_lin + v_rot, VIDA_ORDEN_JOG))
                self.jog_en_curso = "tcp"
        except (OSError, tk.TclError, ValueError) as e:
            self.jog_en_curso = None
            self.mensaje.set(f"Error al mover: {e}")
        self.root.after(PERIODO_JOG_MS, self.tick_jog)

    # --- movimientos a posición exacta
    def mover_art(self, objetivo, vel, nombre=None, gripper=None, fuerza=None):
        est = self.est()
        if not self.robot_listo(est):
            self.mensaje.set("El robot no está listo.")
            return False
        if self.gripper_pendiente() and time.time() - self.gripper_t < 12:
            self.mensaje.set("Espera a que el gripper termine su movimiento.")
            return False
        if any(abs(x) > LIMITE_GRADOS for x in objetivo):
            messagebox.showwarning("Fuera de rango", f"Las articulaciones van de −{LIMITE_GRADOS:.0f}° a {LIMITE_GRADOS:.0f}°.")
            return False
        saltos = [abs(o - a) for o, a in zip(objetivo, est["q"])]
        mayor = max(saltos)
        destino = f"«{nombre}»" if nombre else "la posición indicada"
        if mayor > AVISO_SALTO and not messagebox.askyesno(
                "Confirmar movimiento", f"Para ir a {destino}, {NOMBRES[saltos.index(mayor)]} va a girar "
                                        f"{mayor:.0f}°.\nComprueba que la zona está despejada.\n\n¿Mover el robot?"):
            return False
        vel = min(max(vel, 1.0), VEL_MAX)
        self.pulsadas.clear()
        self.raton.clear()
        self.jog_en_curso = None
        if gripper is None:
            self.enviar(script_movej(objetivo, vel, 2 * vel))
            extra = ""
        else:
            # Un solo programa en el robot: primero el brazo y, al llegar, el gripper
            linea_movej = script_movej(objetivo, vel, 2 * vel).splitlines()[1] + "\n"
            self._orden_gripper(gripper, antes=linea_movej, fuerza=fuerza)
            extra = f" y después gripper {texto_gripper(gripper)}"
        self.mensaje.set(f"Yendo a {destino} a {vel:.0f} °/s{extra}…  (Espacio = parar)")
        return True

    def objetivo_gripper_actual(self):
        """Estado del gripper para guardarlo: 0 abierto, 255 cerrado (o sujetando), otro valor = parcial.
        None si no se conoce."""
        regs = self.modbus.regs if self.modbus else None
        if regs is None or not self.gripper_consultado or self.gripper_pendiente():
            return None
        sta, pos, obj, paso, _seq = regs
        if sta != 3 or paso == 9 or pos > 255:
            return None
        if pos <= 12:
            return 0
        if obj == 2 or pos >= 215:
            return 255
        return int(pos)

    def estado_para_registro(self):
        """Estado del robot en un diccionario (para guardar junto a las capturas de cámara)."""
        est = self.est()
        if est is None:
            return None
        datos = {"articulaciones_grados": [round(x, 4) for x in est["q"]]}
        if "tcp" in est:
            t = est["tcp"]
            datos["herramienta_mm_grados"] = [round(c * 1000, 3) for c in t[:3]] + [round(math.degrees(c), 4) for c in t[3:]]
            datos["herramienta_m_rotvec_rad"] = [round(c, 6) for c in t]
        if "tcp_offset" in est:
            datos["tcp_offset_m_rotvec_rad"] = [round(c, 6) for c in est["tcp_offset"]]
        regs = self.modbus.regs if self.modbus else None
        if regs:
            datos["gripper"] = {"activado": regs[0] == 3, "posicion_0_255": regs[1], "objeto": regs[2]}
        return datos

    # --- posiciones guardadas
    def vel_movimiento(self):
        return max(5.0, VEL_MAX * self.vel_pct.get() / 100)

    def _pintar_posiciones(self):
        self.lbl_home.config(text="Home: " + texto_q(self.home))
        sel = self.lista.curselection()
        self.lista.delete(0, "end")
        for p in self.posiciones:
            g = self._gripper_de(p)
            self.lista.insert("end", "  " + p["nombre"] + (f"   ·  {texto_gripper(g)}" if g is not None else ""))
        if sel and sel[0] < len(self.posiciones):
            self.lista.selection_set(sel[0])
        if self.posiciones:
            self.lbl_vacia.pack_forget()
        else:
            self.lbl_vacia.pack(anchor="w", pady=(6, 0))

    def _guardar_archivo(self):
        err = guardar_posiciones(self.home, self.posiciones)
        if err:
            messagebox.showerror("No se pudo guardar", f"No se pudo escribir {ARCHIVO_POS}:\n{err}")
        self._pintar_posiciones()

    def ir_home(self):
        try:
            self.mover_art(self.home, self.vel_movimiento(), "Home")
        except OSError as e:
            self.mensaje.set(f"No se pudo enviar el movimiento: {e}")

    def guardar_actual(self):
        est = self.est()
        if est is None:
            self.mensaje.set("Sin conexión: no se puede leer la posición actual.")
            return
        q = [round(x, 3) for x in est["q"]]
        nombre = simpledialog.askstring("Guardar posición", "Nombre de la posición:", parent=self.root,
                                        initialvalue=f"Posición {len(self.posiciones) + 1}")
        if not nombre or not nombre.strip():
            return
        nombre = nombre.strip()
        nueva = {"nombre": nombre, "q": q}
        if "tcp" in est:
            t = est["tcp"]
            nueva["tcp_mm_grados"] = [round(c * 1000, 2) for c in t[:3]] + [round(math.degrees(c), 3) for c in t[3:]]
        g = self.objetivo_gripper_actual()
        if g is not None:
            nueva["gripper"] = g
            nueva["fuerza"] = self.fuerza
        existentes = [i for i, p in enumerate(self.posiciones) if p["nombre"].lower() == nombre.lower()]
        if existentes:
            if not messagebox.askyesno("Ya existe", f"Ya hay una posición llamada «{nombre}».\n¿Reemplazarla?"):
                return
            self.posiciones[existentes[0]] = nueva
        else:
            self.posiciones.append(nueva)
        self._guardar_archivo()
        self.mensaje.set(f"Posición «{nombre}» guardada" +
                         (f" con el gripper {texto_gripper(g)}." if g is not None
                          else " (sin estado del gripper: no se conocía en ese momento)."))

    @staticmethod
    def _gripper_de(p):
        g = p.get("gripper")
        return g if isinstance(g, int) and 0 <= g <= 255 else None

    def _seleccionada(self):
        sel = self.lista.curselection()
        if not sel or sel[0] >= len(self.posiciones):
            self.mensaje.set("Elige primero una posición de la lista.")
            return None
        return sel[0]

    def ir_guardada(self):
        i = self._seleccionada()
        if i is None:
            return
        p = self.posiciones[i]
        try:
            self.mover_art(p["q"], self.vel_movimiento(), p["nombre"],
                           gripper=self._gripper_de(p), fuerza=p.get("fuerza"))
        except OSError as e:
            self.mensaje.set(f"No se pudo enviar el movimiento: {e}")

    def borrar_guardada(self):
        i = self._seleccionada()
        if i is None:
            return
        nombre = self.posiciones[i]["nombre"]
        if messagebox.askyesno("Borrar posición", f"¿Borrar la posición «{nombre}»?"):
            del self.posiciones[i]
            self._guardar_archivo()
            self.mensaje.set(f"Posición «{nombre}» borrada.")

    def mover_tcp(self, p, rv_grados, vel):
        est = self.est()
        if not self.robot_listo(est):
            self.mensaje.set("El robot no está listo.")
            return False
        sing = aviso_singularidad(est["q"])
        if sing:
            messagebox.showwarning("Cerca de una singularidad",
                                   f"No se puede mover la herramienta en línea recta: {sing}.\n\n"
                                   "Primero, en la pestaña Articulaciones, dobla el codo (unos 90°) y aleja "
                                   "la Muñeca 2 de 0° y de 180° (por ejemplo, a -90°).")
            return False
        R = rotvec_a_matriz([math.radians(a) for a in rv_grados])
        off = est.get("tcp_offset", [0.0] * 6)
        alcance = LONG_BRAZO_MM + 1000 * math.sqrt(off[0] ** 2 + off[1] ** 2 + off[2] ** 2)
        if math.dist(p, HOMBRO_MM) > alcance:
            messagebox.showwarning("Fuera de alcance", f"Ese punto está a {math.dist(p, HOMBRO_MM):.0f} mm del hombro; "
                                                       f"el máximo con esta herramienta es {alcance:.0f} mm.")
            return False
        dist = math.dist(p, [c * 1000 for c in est["tcp"][:3]])
        giro = angulo_entre(rotvec_a_matriz(est["tcp"][3:]), R)
        if (dist > AVISO_DIST_MM or giro > AVISO_GIRO) and not messagebox.askyesno(
                "Confirmar movimiento", f"La herramienta va a desplazarse {dist:.0f} mm y girar {giro:.0f}°.\n"
                                        "Comprueba que la zona está despejada.\n\n¿Mover el robot?"):
            return False
        vel = min(max(vel, 1.0), VEL_LIN_MAX)
        self.enviar(script_movel(p, matriz_a_rotvec(R), vel, 4 * vel))
        self.mensaje.set(f"Moviendo la herramienta en línea recta a {vel:.0f} mm/s…")
        return True

    # --- ventanas secundarias
    def _ventana(self, titulo):
        d = tk.Toplevel(self.root)
        d.title(titulo)
        d.configure(bg=CARD, padx=22, pady=18)
        d.transient(self.root)
        d.resizable(False, False)
        return d

    def dialogo_posicion(self):
        if self.dialogo is not None and self.dialogo.winfo_exists():
            self.dialogo.destroy()
        tcp = self.modo == "tcp"
        d = self.dialogo = self._ventana("Ir a una posición exacta")
        tk.Label(d, text="Herramienta (línea recta)" if tcp else "Articulaciones", bg=CARD, fg=TEXTO,
                 font=(F, 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
        campos = []
        for i in range(6):
            tk.Label(d, text=NOMBRES_TCP[i] if tcp else NOMBRES[i], bg=CARD, fg=TEXTO, font=(F, 10)).grid(
                row=i + 1, column=0, sticky="w", pady=3)
            e = tk.Entry(d, width=10, font=(F, 11), relief="solid", bd=1, justify="right")
            e.grid(row=i + 1, column=1, padx=8)
            tk.Label(d, text=UNIDADES_TCP[i] if tcp else "°", bg=CARD, fg=SUAVE, font=(F, 10)).grid(
                row=i + 1, column=2, sticky="w")
            campos.append(e)
        tk.Label(d, text="Velocidad", bg=CARD, fg=TEXTO, font=(F, 10)).grid(row=7, column=0, sticky="w", pady=(10, 3))
        e_vel = tk.Entry(d, width=10, font=(F, 11), relief="solid", bd=1, justify="right")
        e_vel.insert(0, "50" if tcp else "20")
        e_vel.grid(row=7, column=1, padx=8, pady=(10, 3))
        tk.Label(d, text="mm/s" if tcp else "°/s", bg=CARD, fg=SUAVE, font=(F, 10)).grid(row=7, column=2, sticky="w", pady=(10, 3))
        if tcp:
            tk.Label(d, text="Giro X/Y/Z = vector de rotación (como en PolyScope), en grados.", bg=CARD, fg=SUAVE,
                     font=(F, 8)).grid(row=8, column=0, columnspan=3, sticky="w", pady=(6, 0))

        def usar_actual():
            est = self.est()
            if not est:
                return
            for i, e in enumerate(campos):
                if tcp and "tcp" in est:
                    x = est["tcp"][i]
                    v = x * 1000 if i < 3 else math.degrees(x)
                else:
                    v = est["q"][i]
                e.delete(0, "end")
                e.insert(0, f"{v:.2f}")

        def mover():
            try:
                vals = [float(e.get().replace(",", ".")) for e in campos]
                vel = float(e_vel.get().replace(",", "."))
            except ValueError:
                messagebox.showwarning("Valor no válido", "Revisa que todos los campos sean números.", parent=d)
                return
            try:
                if tcp:
                    self.mover_tcp(vals[:3], vals[3:], vel)
                else:
                    self.mover_art(vals, vel)
            except OSError as ex:
                self.mensaje.set(f"No se pudo enviar el movimiento: {ex}")

        bts = tk.Frame(d, bg=CARD)
        bts.grid(row=9, column=0, columnspan=3, sticky="ew", pady=(16, 0))
        boton(bts, "Usar posición actual", usar_actual).pack(side="left")
        boton(bts, "Mover", mover, bg=ACENTO, fg="white", activo="#1D4ED8").pack(side="right")
        usar_actual()

    def dialogo_ajustes(self):
        d = self._ventana("Conexión")
        tk.Label(d, text="IP del robot", bg=CARD, fg=TEXTO, font=(F, 10, "bold")).pack(anchor="w")
        e = tk.Entry(d, width=20, font=(F, 11), relief="solid", bd=1)
        e.insert(0, self.ip)
        e.pack(anchor="w", pady=(4, 10))

        def conectar():
            self.conectar(e.get().strip())
            self.mensaje.set(f"Conectando con {self.ip}…")
            d.destroy()

        boton(d, "Conectar", conectar, bg=ACENTO, fg="white", activo="#1D4ED8").pack(anchor="w")
        tk.Label(d, bg=CARD, fg=SUAVE, font=(F, 9), justify="left",
                 text="\nEn PolyScope:\n• Brazo encendido y frenos liberados (Normal)\n"
                      "• Control remoto activado y modo «Remoto»\n• URCap de Robotiq para el gripper").pack(anchor="w")

        tk.Frame(d, bg=BORDE, height=1).pack(fill="x", pady=14)
        tk.Label(d, text="Posición Home", bg=CARD, fg=TEXTO, font=(F, 10, "bold")).pack(anchor="w")
        lbl = tk.Label(d, text=texto_q(self.home), bg=CARD, fg=SUAVE, font=(F, 9))
        lbl.pack(anchor="w", pady=(2, 8))

        def home_actual():
            est = self.est()
            if est is None:
                messagebox.showwarning("Sin conexión", "No se puede leer la posición actual.", parent=d)
                return
            if messagebox.askyesno("Cambiar Home", "¿Usar la posición actual del robot como Home?", parent=d):
                self.home = [round(x, 3) for x in est["q"]]
                self._guardar_archivo()
                lbl.config(text=texto_q(self.home))

        def home_fabrica():
            self.home = list(HOME_FABRICA)
            self._guardar_archivo()
            lbl.config(text=texto_q(self.home))

        fila = tk.Frame(d, bg=CARD)
        fila.pack(anchor="w")
        boton(fila, "Usar posición actual", home_actual, pady=5, padx=10).pack(side="left")
        boton(fila, "Home de fábrica", home_fabrica, pady=5, padx=10, fg=SUAVE).pack(side="left", padx=6)

    def dialogo_atajos(self):
        d = self._ventana("Atajos de teclado")
        tk.Label(d, text="Atajos de teclado", bg=CARD, fg=TEXTO, font=(F, 12, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 4))
        tk.Label(d, text="Activa «Teclado» arriba y mantén pulsada la tecla.", bg=CARD, fg=SUAVE,
                 font=(F, 9)).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 10))
        filas = [("Q A · W S · E D", "mover cada fila (+ / −)"), ("R F · T G · Y H", ""),
                 ("Espacio · Esc", "parar"), ("Z · X", "menos / más velocidad"), ("O · C", "abrir / cerrar gripper")]
        for i, (tecla, accion) in enumerate(filas):
            tk.Label(d, text=tecla, bg=GRIS, fg=TEXTO, font=("Consolas", 10, "bold"), padx=8, pady=2).grid(
                row=i + 2, column=0, sticky="w", pady=3)
            tk.Label(d, text=accion, bg=CARD, fg=SUAVE, font=(F, 10)).grid(row=i + 2, column=1, sticky="w", padx=(12, 0))

    # --- comunes
    def parar(self):
        self.pulsadas.clear()
        self.raton.clear()
        self.jog_en_curso = None
        if self.ventana_cam is not None and self.ventana_cam.viva():
            self.ventana_cam.desactivar_seguimiento()
        self.vel_mano = None
        try:
            self.enviar(SCRIPT_PARAR)
            self.mensaje.set("Orden de PARAR enviada.")
        except OSError as e:
            self.mensaje.set(f"No se pudo enviar PARAR: {e}. ¡Usa la seta de emergencia!")

    def cerrar(self):
        try:
            if self.jog_en_curso:
                self.enviar(SCRIPT_PARAR)
        except OSError:
            pass
        for hilo in (self.lector, self.modbus):
            if hilo:
                hilo.detener()
        if self.ventana_cam is not None:
            self.ventana_cam.cerrar()
        self.root.destroy()


# ----------------------------------------------------------------------------- inicio

def modo_leer(ip):
    with socket.create_connection((ip, PUERTO), timeout=5) as s:
        est = leer_estado(s)
    print(f"Robot {ip}")
    print("Estado:", MODOS_ROBOT.get(est.get("modo"), est.get("modo")),
          "| emergencia:", est.get("emergencia"), "| protección:", est.get("proteccion"))
    for nombre, q, m in zip(NOMBRES, est["q"], est["modos_art"]):
        print(f"  {nombre:<9} {q:9.2f}°   (modo articulación {m})")
    if "tcp" in est:
        t = est["tcp"]
        print("Herramienta: X {:.1f}  Y {:.1f}  Z {:.1f} mm | RX {:.2f}  RY {:.2f}  RZ {:.2f} °".format(
            *(c * 1000 for c in t[:3]), *(math.degrees(c) for c in t[3:])))
    sing = aviso_singularidad(est["q"])
    if sing:
        print("Aviso singularidad:", sing)


def main():
    args = sys.argv[1:]
    if args and args[0] == "--leer":
        modo_leer(args[1] if len(args) > 1 else IP_POR_DEFECTO)
        return
    try:  # texto nítido en pantallas con escalado (Windows)
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (ImportError, AttributeError, OSError):
        pass
    root = tk.Tk()
    app = App(root, args[0] if args else IP_POR_DEFECTO)

    def error_tk(tipo, valor, traza):
        registrar_error(valor)
        try:
            app.mensaje.set(f"Error: {valor} (detalles en errores_ur3e.log)")
        except tk.TclError:
            pass
    root.report_callback_exception = error_tk
    root.mainloop()


if __name__ == "__main__":
    main()
