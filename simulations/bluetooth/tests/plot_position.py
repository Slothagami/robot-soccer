from pygame.locals import QUIT
from recieve       import Port
from time          import sleep
import pygame as pg 
import sys

class Color:
    white  = (255, 255, 255)
    black  = (0, 0, 0)
    dwhite = (150, 150, 150)
    grey   = (32, 32, 32)
    dgrey   = (16, 16, 16)
    orange = (255, 100, 0)

def update(): 
    global pos

    pg.draw.rect(
        window, Color.black, 
        pg.Rect(
            width/2 - 100 * scale, height/2 - 100 * scale, 
            200 * scale, 200 * scale
            )
        )

    positions = port.recieve()
    positions = (msg for msg in positions)

    for msg in positions:
        if msg == "STUTTER":
            walls.append(next(positions))
        else: pos = msg

    for wall in walls: draw_point(wall, True)
    draw_point(pos)
    sleep(.25)

def draw_point(point, is_wall=False):
    point *= scale
    point = (point.imag + width/2, height/2-point.real)

    if is_wall:
        pg.draw.circle(window, Color.orange, point, 3)
    else: pg.draw.circle(window, Color.white, point, 4)

pg.init()
pg.display.set_caption("Hub position tracking")

width, height = 824, 550
window = pg.display.set_mode((width, height))
port   = Port("COM15")
pos    = complex(0)
walls  = []
scale = 2

while True:
    window.fill(Color.grey)
    update()
    pg.display.update()

    for e in pg.event.get():
        if e.type == QUIT:
            pg.quit()
            sys.exit()
