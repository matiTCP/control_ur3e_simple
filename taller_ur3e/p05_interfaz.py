"""
Paso 5 - Primera interfaz gráfica (tkinter) con botones y teclado.

Muestra los 6 ángulos del robot en tiempo real. Mientras mantienes pulsado un botón − / +
(o una tecla), la articulación se mueve; al soltar, se detiene.

Teclas: Q/A Base · W/S Hombro · E/D Codo · R/F Muñeca 1 · T/G Muñeca 2 · Y/H Muñeca 3
        Espacio = PARAR

¡ATENCIÓN! Este programa MUEVE el robot.
"""

import sys
import tkinter as tk          # biblioteca estándar para ventanas

import ur_basico as ur

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.15"
VELOCIDAD = 10.0              # °/s de cada articulación mientras se mantiene pulsada
TECLAS = [("q", "a"), ("w", "s"), ("e", "d"), ("r", "f"), ("t", "g"), ("y", "h")]   # (+, −)


class Aplicacion:
    def __init__(self, ventana):
        self.ventana = ventana
        self.robot = ur.ConexionUR(IP).iniciar()     # conexión permanente (lee en segundo plano)
        self.pulsadas = set()                        # teclas/botones que están pulsados ahora
        self.moviendo = False                        # ¿enviamos órdenes de velocidad?

        ventana.title("Taller UR3e - Paso 5")
        self.lbl_estado = tk.Label(ventana, text="Conectando…", font=("Segoe UI", 11, "bold"))
        self.lbl_estado.grid(row=0, column=0, columnspan=4, pady=8)

        self.lbl_angulos = []
        for i, nombre in enumerate(ur.NOMBRES_ARTICULACIONES):
            mas, menos = TECLAS[i]
            tk.Label(ventana, text=nombre, width=10, anchor="w").grid(row=i + 1, column=0, padx=8)
            lbl = tk.Label(ventana, text="—", width=8, font=("Consolas", 14))
            lbl.grid(row=i + 1, column=1)
            self.lbl_angulos.append(lbl)
            # Botones "mantener pulsado": usamos los eventos de pulsar y soltar del ratón
            for col, texto, tecla in ((2, f"−  ({menos.upper()})", menos), (3, f"+  ({mas.upper()})", mas)):
                b = tk.Button(ventana, text=texto, width=8)
                b.grid(row=i + 1, column=col, padx=4, pady=2)
                b.bind("<ButtonPress-1>", lambda ev, t=tecla: self.pulsadas.add(t))
                b.bind("<ButtonRelease-1>", lambda ev, t=tecla: self.pulsadas.discard(t))

        tk.Button(ventana, text="PARAR (Espacio)", bg="#c62828", fg="white",
                  command=self.parar).grid(row=7, column=0, columnspan=4, pady=10, sticky="ew", padx=8)

        # Teclado: guardamos qué teclas están pulsadas
        ventana.bind("<KeyPress>", self.tecla_abajo)
        ventana.bind("<KeyRelease>", lambda ev: self.pulsadas.discard(ev.keysym.lower()))
        ventana.bind("<FocusOut>", lambda ev: self.pulsadas.clear())   # si cambias de ventana, se para
        ventana.protocol("WM_DELETE_WINDOW", self.cerrar)

        self.refrescar()      # bucle de pantalla (cada 200 ms)
        self.bucle_mover()    # bucle de movimiento (cada 100 ms)

    def tecla_abajo(self, evento):
        tecla = evento.keysym.lower()
        if tecla == "space":
            self.parar()
        else:
            self.pulsadas.add(tecla)

    def refrescar(self):
        """Actualiza los números de la pantalla con el último estado leído."""
        e = self.robot.estado
        if e is None:
            self.lbl_estado.config(text=f"Sin conexión: {self.robot.error or 'conectando…'}", fg="red")
        else:
            listo = ur.robot_listo(e)
            self.lbl_estado.config(text="Listo para mover" if listo else "Robot no listo (Normal + Remoto)",
                                   fg="green" if listo else "orange")
            for lbl, angulo in zip(self.lbl_angulos, e["q"]):
                lbl.config(text=f"{angulo:7.1f}°")
        self.ventana.after(200, self.refrescar)

    def bucle_mover(self):
        """Cada 100 ms: calcula la velocidad de cada articulación según lo pulsado y la envía."""
        velocidades = [0.0] * 6
        if ur.robot_listo(self.robot.estado):
            for i, (mas, menos) in enumerate(TECLAS):
                # (True - False) = 1, (False - True) = -1, iguales = 0
                velocidades[i] = ((mas in self.pulsadas) - (menos in self.pulsadas)) * VELOCIDAD
        try:
            if any(velocidades):
                self.robot.enviar(ur.script_speedj(velocidades))
                self.moviendo = True
            elif self.moviendo:                  # se acaba de soltar todo: frenar
                self.robot.enviar(ur.SCRIPT_PARAR)
                self.moviendo = False
        except OSError as error:
            self.lbl_estado.config(text=f"Error al enviar: {error}", fg="red")
        self.ventana.after(100, self.bucle_mover)

    def parar(self):
        self.pulsadas.clear()
        self.moviendo = False
        try:
            self.robot.enviar(ur.SCRIPT_PARAR)
        except OSError:
            pass

    def cerrar(self):
        self.parar()
        self.robot.detener()
        self.ventana.destroy()


if __name__ == "__main__":
    raiz = tk.Tk()
    Aplicacion(raiz)
    raiz.mainloop()
