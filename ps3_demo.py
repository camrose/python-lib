#!/usr/bin/python
import pygame
import os
import time
#import usb

os.environ['SDL_VIDEO_CENTERED'] = '1'

if not pygame.display.get_init():
    pygame.display.init() 

if not pygame.font.get_init():
    pygame.font.init()

if not pygame.mixer.get_init():
    pygame.mixer.init()  

if not pygame.joystick.get_init():
    pygame.joystick.init()

width = 900
height = 560
path_image = "./ps3_controller.png"

class PS3_Controller:
  
  def __init__(self, surface):
    self.joystick = None
    self.buttons = None
    self.surface = surface
    self.t = 0
    self.time = time.time
    self.left_axis = None
    self.right_axis = None

  def update_axis(self):
    if self.joystick is not None:
      try:
        self.left_axis = [self.joystick.get_axis(0), self.joystick.get_axis(1)]
        self.right_axis =[ self.joystick.get_axis(2), self.joystick.get_axis(3)]
      except pygame.error:  
        print "Axis Error"

  def is_pressed(self, button_name):
    if self.buttons is not None:
      if button_name == 'left_stick':
        self.update_axis()
        if sum(self.left_axis) > 0 or sum(self.left_axis) < 0:
          return True
        else:
          return False
      elif button_name == 'right_stick':
        self.update_axis()
        if sum(self.right_axis) > 0 or sum(self.right_axis) < 0:
          return True
        else:
          return False
      else:
        self.update_buttons()
        if self.buttons[button_name] == 1:
          return True
        else:
          return False
    else:
      return False

  def update_buttons(self):
    if self.joystick is not None:
      pygame.event.pump()
      try:
        self.buttons = {   
          'left': self.joystick.get_button(7), 
          'right' : self.joystick.get_button(5),
          'up' : self.joystick.get_button(4), 
          'down': self.joystick.get_button(6),
          'square' : self.joystick.get_button(15),
          'x' : self.joystick.get_button(14),
          'circle' : self.joystick.get_button(13),
          'triangle' : self.joystick.get_button(12),
          'r1' : self.joystick.get_button(11),
          'r2' : self.joystick.get_button(9),
          'l1' : self.joystick.get_button(10),
          'l2' : self.joystick.get_button(8),
          'select' : self.joystick.get_button(0),
          'start' : self.joystick.get_button(3),
          'l3' : self.joystick.get_button(1),
          'r3' : self.joystick.get_button(2),
          'ps' : self.joystick.get_button(16),
        }
      except pygame.error:
        self.joystick = None
    else:
      self.buttons = None
    return

  #def check_if_connected(self):
  #  try:
  #    busses = usb.busses()
  #    for bus in busses:
  #        devices = bus.devices
  #        for dev in devices:
  #          if dev.idVendor == 1356:
  #            return True
  #    return False
  #  except usb.core.USBError:
  #    print "USB Disconnected"
  #    return False
    
  def check_status(self):
    self.update_buttons()
    #if self.check_if_connected():
    if 1: 
      if self.t == 0:
        print "Connected"
        self.t = 1
        pygame.joystick.quit()
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        while len(joysticks) <= 0:
          pygame.joystick.quit()
          pygame.joystick.init()
          joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        for joystick in joysticks:
          if 'playstation' in joystick.get_name().lower():
            print joystick.get_name()
            self.joystick = joystick
            self.joystick.init()
    
    else:
      print 'Disconnected'
      pygame.joystick.quit()
      self.t = 0
      self.joystick = None
      self.buttons = None

figures = { 'square' : ['circle', 31], 'x' : ['circle', 31], 'triangle' : ['circle', 31], 'ps' : ['circle', 31], 'circle' : ['circle', 31],
          'up' : ['polygon'], 'down' : ['polygon'], 'left' : ['polygon'], 'right' : ['polygon'], 'right_stick' : ['circle', 60],
          'left_stick' : ['circle', 60], 'select' : ['rectangle'], 'start' : ['polygon'], 'l3' : ['circle', 60], 'r3' : ['circle', 60],
          'r1' : ['rectangle'], 'r2' : ['rectangle'], 'l1' : ['rectangle'], 'l2' : ['rectangle'], 
          }

coords = { 'square': (651, 208), 'x' : (717, 275), 'circle' : (782, 208), 'triangle' : (716, 141), 'right_stick' : (578, 346),
        'left_stick' : (319, 342), 
        'up' : [(164, 135), (202, 135), (202, 169), (184, 187), (164, 169), (164, 135)],
        'right' : [(201, 206), (217, 184), (252, 184), (252, 224), (217, 225), (201, 206)],
         'down' : [(184, 224), (205, 241), (205, 272), (162, 272), (162, 241), (184, 224)],
        'left' : [(114, 186), (149, 186), (168, 204), (149, 224), (114, 224), (114, 186)],
        'select' : (349, 212, 38, 18), 'start' : [(511, 212), (539, 221), (511, 232), (511, 212)],
        'ps' : (454, 288), 'l3' : (319, 342), 'r3' : (578, 346), 'l1' : (115, 30, 138, 27), 'l2' : (115, 30, 138, 27),
        'r1' : (647, 30, 138, 27), 'r2' : (647, 30, 138, 27) }

if __name__ == "__main__":
  pygame.init()
  surface = pygame.display.set_mode((width,height))  
  pygame.display.set_caption('Pygame PS3 Demo')
  fondo = pygame.image.load(path_image)
  controller = PS3_Controller(surface)
  while True:
    surface.blit(fondo, (0,0))
    controller.check_status()
    for figure in figures:
      if controller.is_pressed(figure):
        if figures[figure][0] == 'circle':
          pygame.draw.circle(surface, (255, 0, 0), coords[figure], figures[figure][1], 0)
        elif figures[figure][0] == 'polygon':
          pygame.draw.polygon(surface, (255, 0, 0), coords[figure], 0)
        elif figures[figure][0] == 'rectangle':
          pygame.draw.rect(surface, (255, 0, 0), coords[figure], 0)
    pygame.display.update()