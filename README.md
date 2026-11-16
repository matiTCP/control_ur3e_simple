# control_ur3e_simple

Programa en Python con interfaz gráfica para controlar un brazo colaborativo **Universal Robots UR3e** (e-Series) por TCP/IP desde un PC con Windows: articulaciones, herramienta (TCP), gripper **Robotiq**, posiciones guardadas, cámara y seguimiento de mano con visión artificial.

> ⚠️ **Seguridad**: este programa mueve un robot real. Úsalo con la zona de trabajo despejada, a velocidad baja y con la parada de emergencia al alcance. El autor no se hace responsable de daños derivados de su uso (ver [LICENSE](LICENSE)).

## Funciones

- **Articulaciones**: lectura en tiempo real y movimiento mientras se mantiene pulsado − / + (ratón o teclado).
- **Herramienta (TCP)**: movimiento cartesiano X, Y, Z y giros, en ejes de la base o de la herramienta, con aviso de singularidades.
- **Ir a una posición exacta**: `movej` (articulaciones) o `movel` (línea recta), con confirmación en movimientos grandes.
- **Gripper Robotiq**: abrir, cerrar, fuerza (baja / media / alta) y estado (abierto, cerrado, sujetando objeto).
- **Posiciones**: guardar posiciones con nombre (incluido el estado del gripper), volver a ellas y botón **Home**.
- **Cámara**: visor en vivo, cruz central y captura de fotos con la posición del robot en un JSON (útil para datasets y calibración cámara-robot).
- **Seguimiento de mano** (MediaPipe): el robot sigue los movimientos de la mano delante de la cámara.
- **PARAR** siempre visible (también con Espacio o Esc).

## Requisitos

**Robot**
- UR3e e-Series con PolyScope 5.x (probado con 5.20).
- Brazo encendido y frenos liberados (estado *Normal*).
- **Control remoto** activado y modo **Remoto** seleccionado en PolyScope.
- Para el gripper: URCap de Robotiq instalada.

**PC**
- Windows 10/11 y Python 3.10 o superior (probado con 3.12).
- PC y robot en la misma red.
- Opcional: webcam para la cámara y el seguimiento de mano.

## Instalación

```bash
git clone https://github.com/matiTCP/control_ur3e_simple.git
cd control_ur3e_simple
python -m pip install -r requirements.txt
```

El control del robot y del gripper funciona **sin instalar nada**: solo usa la biblioteca estándar de Python. `requirements.txt` solo hace falta para la cámara y el seguimiento de mano.

Para el seguimiento de mano, descarga el modelo oficial de MediaPipe en la carpeta `modelos`:

```powershell
mkdir modelos
Invoke-WebRequest -Uri "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task" -OutFile "modelos\hand_landmarker.task"
```

## Uso

```bash
python control_ur3e.py                 # IP por defecto: 192.168.0.15
python control_ur3e.py 192.168.0.20    # otra IP
python control_ur3e.py --leer          # solo muestra la posición actual (no mueve nada)
```

La IP también se puede cambiar desde el botón ⚙ de la ventana.

### Teclado

Activa **Teclado** en la cabecera y mantén pulsada la tecla:

| Teclas | Acción |
|---|---|
| Q/A · W/S · E/D · R/F · T/G · Y/H | mover cada fila (+ / −) de la pestaña abierta |
| Espacio · Esc | **parar** |
| Z · X | menos / más velocidad |
| O · C | abrir / cerrar gripper |

### Seguimiento de mano

1. Abre **Cámara** → **Activar seguimiento**.
2. Pon la mano en el **centro** de la imagen (el rectángulo se pone verde).
3. Muévela a la izquierda o a la derecha: la articulación elegida (por defecto la Base) gira, más rápido cuanto más lejos del centro.
4. Si la mano sale de la imagen, el robot se para. Para seguir, vuelve a ponerla en el centro.

## Cómo funciona

| Puerto del robot | Uso |
|---|---|
| 30002 (interfaz secundaria) | lectura del estado y envío de URScript (`speedj`, `speedl`, `movej`, `movel`, `stopj`) |
| 502 (Modbus TCP) | lectura del estado del gripper (registros 128-132) |

- El movimiento continuo (teclado, ratón, mano) envía órdenes de velocidad cada 0,1 s que caducan a los 0,3 s: si el PC se cuelga o se corta la red, **el robot se detiene solo**.
- El gripper se controla con un script que se conecta desde dentro del robot al servicio de la URCap de Robotiq (`127.0.0.1:63352`) y deja el resultado en registros Modbus.

## Archivos que crea el programa

Junto a `control_ur3e.py` (no se suben al repositorio):

- `posiciones_ur3e.json`: posiciones guardadas y Home.
- `capturas/`: fotos de la cámara con su JSON de posición del robot.
- `errores_ur3e.log`: registro de errores, si los hay.

## Licencia

[MIT](LICENSE) © 2026 matiTCP. Puedes usar, modificar y redistribuir el código manteniendo el aviso de copyright.

### Terceros y marcas

- *Universal Robots*, *UR3e* y *PolyScope* son marcas de Universal Robots A/S. *Robotiq* es marca de Robotiq Inc. Este proyecto es independiente y **no está afiliado, patrocinado ni respaldado** por esas empresas.
- [MediaPipe](https://github.com/google-ai-edge/mediapipe) (incluido el modelo Hand Landmarker) y [OpenCV](https://opencv.org) se distribuyen bajo la licencia Apache 2.0 y se instalan aparte; no se incluyen en este repositorio.
