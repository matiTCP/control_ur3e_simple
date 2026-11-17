# control_ur3e_simple

Programa en Python con interfaz gráfica para controlar un brazo colaborativo **Universal Robots UR3e** (e-Series) por TCP/IP desde un PC con Windows: articulaciones, herramienta (TCP), gripper **Robotiq**, posiciones guardadas, cámara y seguimiento de mano con visión artificial.

> ⚠️ **Seguridad**: este programa mueve un robot real. Úsalo con la zona de trabajo despejada, a velocidad baja y con la parada de emergencia al alcance. El autor no se hace responsable de daños derivados de su uso (ver [LICENSE](LICENSE)).

## Funciones

- **Articulaciones**: lectura en tiempo real y movimiento mientras se mantiene pulsado − / + (ratón o teclado).
- **Herramienta (TCP)**: movimiento cartesiano X, Y, Z y giros, en ejes de la base o de la herramienta. Se bloquea cerca de una singularidad (codo o Muñeca 2 a menos de 15° de 0°/180°) para evitar fallos como C306A3.
- **Ir a una posición exacta**: `movej` (articulaciones) o `movel` (línea recta), con confirmación en movimientos grandes.
- **Gripper Robotiq**: abrir, cerrar, fuerza (baja / media / alta) y estado (abierto, cerrado, sujetando objeto).
- **Posiciones**: guardar posiciones con nombre (incluido el estado del gripper), volver a ellas y botón **Home**.
- **Cámara**: visor en vivo, cruz central y captura de fotos con la posición del robot en un JSON (útil para datasets y calibración cámara-robot).
- **Seguimiento de mano** (MediaPipe): el robot sigue los movimientos de la mano delante de la cámara.
- **PARAR** siempre visible (también con Espacio o Esc).

## Descargar instalador (Windows)

La forma más fácil de usarlo: **[descarga el instalador de la última versión](https://github.com/matiTCP/control_ur3e_simple/releases/latest)** (`Instalar_Control_UR3e_<versión>.exe`, en *Assets*). No hace falta Python ni permisos de administrador.

1. Abre el instalador. Si aparece «Windows protegió su PC», pulsa **Más información → Ejecutar de todas formas** (el instalador no está firmado).
2. Elige **Instalar solo para mí** y sigue el asistente.
3. Abre **Control UR3e** desde el menú Inicio o el escritorio.

Las posiciones, capturas y errores se guardan en `Documentos\Control UR3e`. Se desinstala desde Configuración → Aplicaciones.

Para usar o modificar el código fuente, sigue los pasos de [Instalación](#instalación).

## Requisitos

**Robot**
- UR3e e-Series con PolyScope 5.x (probado con 5.20).
- Brazo encendido y frenos liberados (estado *Normal*).
- **Control remoto** activado y modo **Remoto** seleccionado en PolyScope.
- Para el gripper: URCap de Robotiq instalada.

**PC**
- Windows 10/11 y Python 3.10 o superior (probado con 3.12). Con el instalador no hace falta Python.
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
python control_ur3e.py --diagnostico   # comprueba cámara y MediaPipe sin conectarse al robot (diagnostico.txt)
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

Junto a `control_ur3e.py` (no se suben al repositorio). Si el programa está empaquetado como `.exe` (PyInstaller), se guardan en `Documentos\Control UR3e`:

- `posiciones_ur3e.json`: posiciones guardadas y Home.
- `capturas/`: fotos de la cámara con su JSON de posición del robot.
- `errores_ur3e.log`: registro de errores, si los hay.
- `diagnostico.txt`: resultado de `--diagnostico`.

## Taller paso a paso (`taller_ur3e/`)

Versión didáctica del programa, dividida en 10 pasos cortos y comentados línea a línea, pensada para un taller de robótica de primer año de ingeniería. Cada paso se ejecuta por separado:

| Archivo | Qué hace |
|---|---|
| `ur_basico.py` | biblioteca mínima: conexión por el puerto 30002, lectura del estado y URScript |
| `p01_conexion.py` | comprobar la conexión con el robot |
| `p02_leer_estado.py` | leer articulaciones, herramienta y estado |
| `p03_movej.py` | primer movimiento con `movej` |
| `p04_speedj.py` | movimiento continuo y seguro con `speedj` (perro guardián) |
| `p05_interfaz.py` | primera interfaz gráfica (tkinter) con botones y teclado |
| `p06_herramienta.py` | mover la herramienta (TCP) con `movel` |
| `p07_gripper.py` | controlar el gripper Robotiq |
| `p08_posiciones.py` | guardar posiciones y volver a ellas (JSON) |
| `p09_camara.py` | ver la cámara con OpenCV y capturar fotos |
| `p10_seguir_mano.py` | seguir la mano con MediaPipe |
| `mano_robot.py` | **taller corto**: programa mínimo que mueve una articulación con la mano frente a la cámara |

```bash
cd taller_ur3e
python p01_conexion.py
```

Para `p10_seguir_mano.py` y `mano_robot.py`, copia la carpeta `modelos/` (con `hand_landmarker.task`) dentro de `taller_ur3e/`.

### Taller corto: mover el robot con la cámara

`mano_robot.py` es un programa corto para un taller de pocas horas. Si la mano está a la izquierda o a la derecha del centro de la imagen, una articulación gira en un sentido u otro. Si está en el centro o fuera de la imagen, el robot se detiene. Sin `--robot` funciona en **modo prueba** y no mueve nada:

```bash
python -m pip install mediapipe
cd taller_ur3e
python mano_robot.py            # modo prueba: solo muestra la velocidad
python mano_robot.py --robot    # mueve el robot (zona despejada y parada de emergencia al alcance)
```

Al inicio del programa se pueden cambiar `ARTICULACION` (0 Base … 5 Muñeca 3), `VEL_MAX` (°/s) y `ZONA` (tamaño de la zona central).

## Generar el instalador (`instalador/`)

El instalador de Windows se construye con [PyInstaller](https://pyinstaller.org) e [Inno Setup 6](https://jrsoftware.org/isinfo.php):

```powershell
winget install JRSoftware.InnoSetup          # solo la primera vez
powershell -ExecutionPolicy Bypass -File instalador\construir.ps1 -Version 1.0.1
```

El script crea un entorno de Python aparte en `%LOCALAPPDATA%\ControlUR3e-build` (con MediaPipe y PyInstaller), descarga el modelo de la mano si falta y deja el resultado en `instalador\salida\Instalar_Control_UR3e_<versión>.exe`. Ese archivo no se sube al repositorio: se publica como [Release](https://github.com/matiTCP/control_ur3e_simple/releases).

| Archivo | Qué es |
|---|---|
| `construir.ps1` | compila el programa y el instalador |
| `control_ur3e.iss` | configuración de Inno Setup (instalación por usuario, sin administrador) |
| `icono.ico` / `crear_icono.py` | icono del programa y el script que lo genera |

## Licencia

[MIT](LICENSE) © 2026 matiTCP. Puedes usar, modificar y redistribuir el código manteniendo el aviso de copyright.

### Terceros y marcas

- *Universal Robots*, *UR3e* y *PolyScope* son marcas de Universal Robots A/S. *Robotiq* es marca de Robotiq Inc. Este proyecto es independiente y **no está afiliado, patrocinado ni respaldado** por esas empresas.
- [MediaPipe](https://github.com/google-ai-edge/mediapipe) (incluido el modelo Hand Landmarker) y [OpenCV](https://opencv.org) se distribuyen bajo la licencia Apache 2.0. No se incluyen en este repositorio; el instalador de Windows sí los incluye, con sus licencias.
