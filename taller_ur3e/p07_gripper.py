"""
Paso 7 - Controlar el gripper Robotiq.

La URCap de Robotiq (instalada en el robot) ofrece un servicio en el puerto 63352, pero el robot
solo lo deja usar DESDE DENTRO (127.0.0.1). Por eso:
    1. El PC envía un programa URScript que se conecta a 127.0.0.1:63352 y da la orden al gripper.
    2. Ese programa escribe el resultado en registros Modbus del robot (128 a 131).
    3. El PC lee esos registros por Modbus TCP (puerto 502).

Órdenes del protocolo del gripper (texto):
    SET ACT 1                       activar (calibra abriendo y cerrando una vez)
    SET POS 255 SPE 200 FOR 150 GTO 1   ir a posición 255 (cerrado) con velocidad y fuerza dadas
    GET STA / GET POS / GET OBJ     leer estado (3 = activo), posición (0-255), objeto detectado
    Las órdenes SET responden "ack" SIN salto de línea; las GET responden "POS 123" con salto de línea.

Uso:
    python p07_gripper.py estado
    python p07_gripper.py cerrar
    python p07_gripper.py abrir
"""

import socket
import struct
import sys
import time

import ur_basico as ur

IP = "192.168.0.15"
ORDEN = sys.argv[1] if len(sys.argv) > 1 else "estado"
REG = 128                    # primer registro Modbus que usamos (128 = STA, 129 = POS, 130 = OBJ, 131 = fin)


def script_gripper(posicion=None, velocidad=200, fuerza=150):
    """Construye el programa URScript que habla con el gripper desde dentro del robot.
    posicion=None solo consulta el estado; 0 = abierto, 255 = cerrado."""
    mover = ""
    if posicion is not None:
        mover = f"""
    if sta != 3:
      rq_set("SET ACT 1")
      t = 0
      while sta != 3 and t < 100:
        sleep(0.1)
        sta = rq_get("STA")
        t = t + 1
      end
    end
    rq_set("SET POS {posicion} SPE {velocidad} FOR {fuerza} GTO 1")
    sleep(0.3)
    t = 0
    obj = 0
    while obj == 0 and t < 120:
      sleep(0.05)
      obj = rq_get("OBJ")
      t = t + 1
    end"""
    return f"""def gripper():
  def rq_set(orden):
    socket_send_line(orden, "rq")
    socket_read_byte_list(3, "rq", 1.0)
  end
  def rq_get(variable):
    socket_send_line(str_cat("GET ", variable), "rq")
    linea = socket_read_line("rq", 1.0)
    i = str_find(linea, " ")
    if i > 0:
      return to_num(str_sub(linea, i + 1))
    end
    return 0
  end
  write_port_register({REG + 3}, 0)
  if socket_open("127.0.0.1", 63352, "rq"):
    sta = rq_get("STA"){mover}
    write_port_register({REG}, rq_get("STA"))
    write_port_register({REG + 1}, rq_get("POS"))
    write_port_register({REG + 2}, rq_get("OBJ"))
    socket_close("rq")
    write_port_register({REG + 3}, 1)
  else:
    write_port_register({REG + 3}, 9)
  end
end
"""


def leer_registros(ip, primero, cantidad):
    """Lee 'cantidad' registros Modbus a partir de 'primero' (función 3: Read Holding Registers).

    Petición Modbus TCP = cabecera MBAP (7 bytes) + código de función + dirección + cantidad.
    """
    with socket.create_connection((ip, 502), timeout=3) as m:
        peticion = struct.pack("!HHHBBHH",
                               1,          # identificador de transacción
                               0,          # protocolo (0 = Modbus)
                               6,          # bytes que siguen
                               0,          # unidad
                               3,          # función 3 = leer registros
                               primero, cantidad)
        m.sendall(peticion)
        cabecera = ur.recibir_exacto(m, 9)               # MBAP (7) + función (1) + nº de bytes (1)
        n_bytes = cabecera[8]
        return struct.unpack(f"!{n_bytes // 2}H", ur.recibir_exacto(m, n_bytes))


posiciones = {"estado": None, "abrir": 0, "cerrar": 255}
if ORDEN not in posiciones:
    sys.exit("Orden no válida: usa estado, abrir o cerrar.")
if ORDEN != "estado" and not ur.confirmar(f"El gripper va a {ORDEN}se. Mantén las manos fuera. ¿Continuar?"):
    sys.exit("Cancelado.")

ur.enviar_script(IP, script_gripper(posiciones[ORDEN]))
for _ in range(60):                                      # esperamos hasta 15 s
    time.sleep(0.25)
    sta, pos, obj, fin = leer_registros(IP, REG, 4)
    if fin in (1, 9):
        break
if fin == 9:
    print("No se pudo conectar con el gripper. ¿Está instalada la URCap de Robotiq?")
else:
    textos_obj = {0: "en movimiento", 1: "objeto detectado al abrir", 2: "objeto detectado al cerrar",
                  3: "llegó a la posición, sin objeto"}
    print(f"Activado: {'sí' if sta == 3 else 'no'} | Posición: {pos}/255 ({round(pos / 2.55)} % cerrado)"
          f" | {textos_obj.get(obj, obj)}")
