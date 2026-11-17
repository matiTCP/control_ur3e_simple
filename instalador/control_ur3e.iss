; Instalador de «Control UR3e» (Inno Setup 6). Lo compila construir.ps1; no hace falta abrirlo a mano.
; Se instala para el usuario actual, SIN permisos de administrador (pensado para los PC del laboratorio).

#define AppNombre "Control UR3e"
#ifndef AppVersion
  #define AppVersion "1.0.0"
#endif

[Setup]
AppId={{6F3B2C1E-8A4D-4F7B-9C2E-5A3E00000001}
AppName={#AppNombre}
AppVersion={#AppVersion}
AppVerName={#AppNombre} {#AppVersion}
AppPublisher=Taller de Robótica
AppPublisherURL=https://github.com/matiTCP/control_ur3e_simple
DefaultDirName={autopf}\{#AppNombre}
DefaultGroupName={#AppNombre}
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=10.0
LicenseFile=..\LICENSE
SetupIconFile=icono.ico
UninstallDisplayIcon={app}\Control UR3e.exe
OutputDir=salida
OutputBaseFilename=Instalar_Control_UR3e_{#AppVersion}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern

[Languages]
Name: "es"; MessagesFile: "compiler:Languages\Spanish.isl"

[Tasks]
Name: "escritorio"; Description: "Crear un acceso directo en el escritorio"; GroupDescription: "Accesos directos:"

[Files]
Source: "{#DistDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autoprograms}\{#AppNombre}"; Filename: "{app}\Control UR3e.exe"
Name: "{autoprograms}\Carpeta de datos de {#AppNombre}"; Filename: "{userdocs}\Control UR3e"
Name: "{autodesktop}\{#AppNombre}"; Filename: "{app}\Control UR3e.exe"; Tasks: escritorio

[Dirs]
Name: "{userdocs}\Control UR3e"; Flags: uninsneveruninstall

[Run]
Filename: "{app}\Control UR3e.exe"; Description: "Abrir {#AppNombre} ahora"; Flags: nowait postinstall skipifsilent

[Messages]
es.WelcomeLabel2=Se instalará [name/ver] en este equipo.%n%nPrograma para controlar un brazo colaborativo Universal Robots UR3e por la red: articulaciones, herramienta, gripper Robotiq, posiciones, cámara y seguimiento de mano.%n%nSEGURIDAD: este programa mueve un robot real. Úsalo con la zona despejada, a velocidad baja y con la parada de emergencia al alcance.
es.FinishedLabel=Se instaló [name].%n%nLas posiciones guardadas, las capturas de la cámara y el registro de errores quedan en Documentos\Control UR3e.
