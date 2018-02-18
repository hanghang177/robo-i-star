import tkinter as tk
from PIL import Image,ImageTk
import sys
import math

WIDTH = 1280
HEIGHT = 720

# Grainger, MEL, MSEB, Engineering Hall, Everitt Labratory, Talbot Laboratory

coordnames = ["Grainger","MEL","MSEB","Engineering Hall","Everitt Labratory","Talbot Laboratory"]
pixcoords = [[242, 115, 458, 179], [411, 215, 555, 381], [431, 456, 554, 567], [259, 475, 400, 567], [16, 463, 162, 567],[18, 220, 160, 341]]

def click(event):
    x = event.x
    y = event.y
    for a in range(len(pixcoords)):
        pixcoord = pixcoords[a]
        xmin, ymin, xmax, ymax = pixcoord
        if (x >= xmin) and (x <= xmax) and (y >= ymin) and (y <= ymax):
            print coordnames[a]
            return a

class UI(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        mapimage = ImageTk.PhotoImage(Image.open("tests/map.jpg"))
        self.title("ROBO-I-STAR")
        # self.geometry("1280x720")
        panel = tk.Label(self, image = mapimage)
        panel.image = mapimage
        #panel.pack()
        panel.bind("<Button-1>",click)
        panel.pack(side="bottom", fill="both", expand="yes")


UI().mainloop()