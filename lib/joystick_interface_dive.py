#!/usr/bin/env python
import time, sys, StringIO, msvcrt
from quaternion import *
from struct import *
from serial import *
from math import *
from increment_counter import IncrementCounter
from increment_counter import CircleCounter
from dictionaries import *
import pygame

class JoystickInterface(object):
    
    def __init__(self, comm_interface = None, hall_interface = None, keyboard_interface = None, joyhandle = None):
        self.comm = comm_interface
        self.hall = hall_interface
        self.joyst = joyhandle
        self.kbint = keyboard_interface
        self.buttons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
        self.start = True
        
    def process(self):
        pygame.event.pump()
        new_buttons = [self.joyst.get_button(0),self.joyst.get_button(1),0,self.joyst.get_button(3),self.joyst.get_button(4),0,self.joyst.get_button(6),0,0,0,self.joyst.get_button(10),self.joyst.get_button(11),self.joyst.get_button(12),self.joyst.get_button(13),self.joyst.get_button(14),self.joyst.get_button(15),0]


        c = [i for i, x in enumerate(new_buttons) if x == 0 and self.buttons[i] == 1]

        if self.start:
            for i in c:
                if i == 0:
                    self.start = 0
                    self.comm.setRegulatorMode(RegulatorStates['Off'])
                if i == 1:
                    self.yaw.set(0.0)
                    self.ref_changed = True
                elif i == 4:
                    self.hall.thrust.increase()
                    self.freq_changed = True
                elif i == 6:
                    self.hall.thrust.decrease()
                    self.freq_changed = True
                elif i == 10:
                    self.pitch.decrease()
                    self.ref_changed = True
                elif i == 11:
                    self.comm.startSensorDump(0)
                elif i == 12:
                    self.comm.setRegulatorMode(RegulatorStates['Stabilize'])
                elif i == 13:
                    self.comm.setRegulatorMode(RegulatorStates['Off'])
                elif i == 14:
                    self.kbint.pitch_coeffs = [ 0.0,    0.0,    3.0,   0.0,    0.2,    1.0,    1.0]
                    self.comm.setRegulatorPid( self.kbint.yaw_coeffs + self.kbint.pitch_coeffs + self.kbint.roll_coeffs )
                    self.comm.setRegulatorRef( eulerToQuaternionDeg( self.kbint.yaw.value(), 70.0, self.kbint.roll.value() ) )
                    #time.sleep(0.2)
                    self.comm.setRegulatorOffsets((self.kbint.steer.value(), self.kbint.elevator.value(), 1.0))
                elif i == 15:
                    self.kbint.pitch_coeffs = [ 0.0,    0.0,    0.0,   0.0,    0.0,    1.0,    1.0]
                    self.comm.setRegulatorPid( self.kbint.yaw_coeffs + self.kbint.pitch_coeffs + self.kbint.roll_coeffs )
                    self.comm.setRegulatorOffsets((self.kbint.steer.value(), self.kbint.elevator.value(), 0.0))

        self.buttons = new_buttons
