import pygame
from pygame.locals import *

if __name__ == '__main__':
   pygame.init()
   pygame.display.set_mode((1000,600))
   pygame.display.set_caption('Testing')
   running = True
   while running:
      for event in pygame.event.get():
         if event.type == QUIT:
            running = False
         if event.type == MOUSEMOTION:
           print pygame.mouse.get_pos()[0]
   pygame.display.quit()