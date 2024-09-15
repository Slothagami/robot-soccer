from pygame.locals import QUIT
from recieve       import Port
from time          import sleep
import pygame as pg 
import sys

def average(list):
    return sum(list) / len(list)

class Color:
    white  = (255, 255, 255)
    black  = (0, 0, 0)
    dwhite = (150, 150, 150)
    grey   = (32, 32, 32)
    dgrey   = (16, 16, 16)
    orange = (255, 100, 0)

def update(): 
    global vector

    # draw axes
    pg.draw.line(window, Color.black, (width/2, 0), (width/2, height))
    pg.draw.line(window, Color.black, (0, height/2), (width, height/2))

    acceleraions = port.recieve()
    if acceleraions: 
        acceleraion = average(acceleraions)
    else: return

    pg.draw.line(
        window, Color.orange, 
        (width/2, height/2),
        (width/2 + acceleraion.real * scale, height/2 - acceleraion.imag * scale),
        5
    )
    sleep(1/24)

pg.init()
pg.display.set_caption("Hub position tracking")

width, height = 824, 550
window = pg.display.set_mode((width, height))
port   = Port("COM5")   
vector = complex(0)
scale = .2

while True:
    window.fill(Color.grey)
    update()
    pg.display.update()

    for e in pg.event.get():
        if e.type == QUIT:
            pg.quit()
            sys.exit()
