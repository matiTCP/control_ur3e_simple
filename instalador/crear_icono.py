"""Genera icono.ico (cuadrado rojo redondeado con un brazo robótico simple). Requiere Pillow."""
from PIL import Image, ImageDraw

N = 256
img = Image.new("RGBA", (N, N), (0, 0, 0, 0))
d = ImageDraw.Draw(img)
d.rounded_rectangle([8, 8, N - 8, N - 8], radius=48, fill=(214, 40, 40))
blanco = (255, 255, 255)
d.rectangle([60, 200, 196, 218], fill=blanco)                 # base
d.line([(128, 200), (100, 120)], fill=blanco, width=22)       # brazo
d.line([(100, 120), (176, 80)], fill=blanco, width=18)        # antebrazo
for x, y, r in [(128, 200, 18), (100, 120, 16), (176, 80, 13)]:
    d.ellipse([x - r, y - r, x + r, y + r], fill=blanco)       # articulaciones
d.line([(176, 80), (200, 66)], fill=blanco, width=8)          # pinza
d.line([(176, 80), (190, 100)], fill=blanco, width=8)
img.save("icono.ico", sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
img.save("icono_vista.png")
print("ok")
