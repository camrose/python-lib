import subprocess
import pygame

pygame.init()
ps3 = pygame.joystick.Joystick(0)
ps3.init()

while True:
    try:
        pygame.event.pump()
        buttons = [ps3.get_button(0),ps3.get_button(1),ps3.get_button(2),ps3.get_button(3),ps3.get_button(4),ps3.get_button(5),ps3.get_button(6),ps3.get_button(7),ps3.get_button(8),ps3.get_button(9),ps3.get_button(10),ps3.get_button(11),ps3.get_button(12),ps3.get_button(13),ps3.get_button(14),ps3.get_button(15),ps3.get_button(16)]
        axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(2),ps3.get_axis(3)]
        print(buttons)
        #print(axis)
        #subprocess.call("clear")
    except KeyboardInterrupt:
        print("Exiting")
        raise
