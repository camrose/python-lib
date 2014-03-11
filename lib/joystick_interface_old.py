#!/usr/bin/env python
import time, sys, msvcrt
from quaternion import *
from struct import *
from serial import *
from math import *
from increment_counter import IncrementCounter
from dictionaries import *
import pygame

class JoystickInterface(object):
    
    def __init__(self, comm_interface = None, joyhandle = None):
        self.comm = comm_interface
        self.joyst = joyhandle
        self.elevator = IncrementCounter( start_value = 0.0, range = (1.0, -1.0), increment = 0.2 )
        self.start = False
    def process(self):
        pygame.event.pump()
        if self.joyst.get_button(11):
            self.start = 1
            self.comm.setRegulatorMode(RegulatorStates['Remote Control'])

        if self.joyst.get_button(8):
            self.start = 0
            self.comm.setRegulatorMode(RegulatorStates['Off'])
            
        if self.start:
            thrust = -self.joyst.get_axis(1)
            if thrust < 0.1:
                thrust = 0
                
            steer = -self.joyst.get_axis(2)
            
            elevdown = self.joyst.get_button(4)
            elevup = self.joyst.get_button(5)
            if elevdown:
                self.elevator.increase()
            elif elevup:
                self.elevator.decrease()

            self.comm.setRemoteControlValues( thrust, steer, self.elevator.value() );
