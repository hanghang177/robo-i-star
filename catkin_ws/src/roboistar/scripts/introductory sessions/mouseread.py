import pygame
from pygame.locals import *

if __name__ == '__main__':
   pygame.init()  # Initialize the PyGame package
   pygame.display.set_mode((1000,600)) # Make the window 1000*600
   pygame.display.set_caption('Testing')  # Make the caption "Testing"
   running = True # Just a indicator of running
   while running: # While the program is running
      for event in pygame.event.get(): # Gets the events from PyGame
         if event.type == QUIT:  # If a button indicates quit
            running = False   # gets out of the loop (break works)
         if event.type == MOUSEMOTION: # If your mouse move
           print pygame.mouse.get_pos()[0]   # Print the position of mouse
   pygame.display.quit()   # Quit the display