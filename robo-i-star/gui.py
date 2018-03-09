import tkinter as tk
from PIL import Image,ImageTk
import sys
import math

WIDTH = 1280
HEIGHT = 720

# Grainger, MEL, MSEB, Engineering Hall, Everitt Labratory, Talbot Laboratory

coordnames = ["Grainger", "Talbot", "Everitt", "Engineering Hall", "Material Science Building", "Mechanical Engineering Lab"]
pixcoords = [[242, 115, 458, 179], [18, 220, 160, 341], [16, 463, 162, 567], [259, 475, 400, 567], [431, 456, 554, 567],[411, 215, 555, 381]]

class UI(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        mapimage = ImageTk.PhotoImage(Image.open("tests/map.jpg"))
        self.title("ROBO-I-STAR")
        self.location = tk.StringVar()
        self.locationindex = tk.IntVar()
        # self.geometry("1280x720")
        panel = tk.Label(self, image = mapimage)
        panel.image = mapimage
        #panel.pack()
        panel.bind("<Button-1>",self.click)
        panel.pack(side="bottom", fill="both", expand="yes")

    def click(self, event):
        x = event.x
        y = event.y
        for a in range(len(pixcoords)):
            pixcoord = pixcoords[a]
            xmin, ymin, xmax, ymax = pixcoord
            if (x >= xmin) and (x <= xmax) and (y >= ymin) and (y <= ymax):
                self.location.set(coordnames[a])
                self.locationindex.set(a)
                print(coordnames[a])
                self.destroy()
