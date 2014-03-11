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
    
    def __init__(self, comm_interface = None, hall_interface = None, joyhandle = None):
        self.comm = comm_interface
        self.hall = hall_interface
        self.joyst = joyhandle
        self.buttons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
        self.pitch = IncrementCounter( start_value = 0.0, range = (90.0, -90.0), increment = 10.0 )
        self.thrust = IncrementCounter( start_value = 15.0, range = (17.0, 0.0), increment = 1.0 )
        self.yaw = IncrementCounter( start_value = 0.0, range = (180.0, -180.0), increment = 60.0 )
        
        self.yaw_coeffs = [ 0.0,    0.0,    2.0,   0.0,    0.4,    1.0,    1.0] # For steer Ki 0.8
        self.pitch_coeffs = [ 0.0,    0.0,    3.0,   0.0,    0.2,    1.0,    1.0] # For elevator
        self.roll_coeffs = [ 0.0,    0.0,    0.0,   0.0,    0.0,    1.0,    1.0] # For thrust 
        self.yaw_filter_coeffs = [ 3, 0, 0.0007, 0.0021, 0.0021, 0.0007, 1.0, 2.6861573965, -2.419655111, 0.7301653453]

        self.start = False
        self.ref_changed = False
        self.freq_changed = False
        
    def process(self):
        pygame.event.pump()
        new_buttons = [self.joyst.get_button(0),self.joyst.get_button(1),0,self.joyst.get_button(3),self.joyst.get_button(4),0,self.joyst.get_button(6),0,0,0,self.joyst.get_button(10),self.joyst.get_button(11),self.joyst.get_button(12),self.joyst.get_button(13),self.joyst.get_button(14),self.joyst.get_button(15),0]

        if new_buttons[3]==0 and self.buttons[3]:
            self.start = 1
            self.comm.setRegulatorMode(RegulatorStates['Track Hall'])
            self.buttons[3] = 0

        if new_buttons[12] == 0 and self.buttons[12]:
            self.comm.setRegulatorPid( self.yaw_coeffs + self.pitch_coeffs + self.roll_coeffs )                                
            self.comm.setRegulatorRateFilter( self.yaw_filter_coeffs )
            self.comm.setHallGains([5,0.2,100,0,900])
            self.comm.setTelemetrySubsample(1)
            self.buttons[12] = 0

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
                    self.pitch.increase()
                    self.ref_changed = True
                elif i == 13:
                    self.yaw.increase()
                    self.ref_changed = True
                elif i == 14:
                    self.comm.setSlewLimit(4.0)
                    self.comm.toggleFigureEight(1)
                elif i == 15:
                    self.yaw.decrease()
                    self.ref_changed = True

            if self.ref_changed:                    
                print   "Yaw, Pitch, Roll: " + str(self.yaw.value()) + " " + \
                    str(self.pitch.value()) + " " + str(0.0)
                self.comm.setRegulatorRef( eulerToQuaternionDeg( self.yaw.value(), self.pitch.value(), 0.0 ) )        
                self.ref_changed = False

            if self.freq_changed:
                print "Thrust: " + str(self.hall.thrust.value())
                self.hall.setVelProfile([90, 180, 270, 360], [25, 25, 25, 25], self.hall.thrust.value())
                self.comm.setVelProfile(self.hall.delta+self.hall.interval+self.hall.vel, self.hall.num_setpoints)
                self.comm.setHallThrust(self.hall.thrust.value(),1)
                self.freq_changed = False

        self.buttons = new_buttons
