#!/usr/bin/env python
import time, sys, msvcrt
from quaternion import *
from struct import *
from serial import *
from math import *

class IncrementCounter(object):
    
    def __init__(self, start_value = 0, range = (-1.0, 1.0), increment = 0.1):
        self.val = start_value
        self.lower_bound = min(range)
        self.upper_bound = max(range)
        self.increment = increment
        
    def value(self):
        return self.val
        
    def increase(self):
        self.val = self.val + self.increment
        self.__enforceRange()
        
    def decrease(self):
        self.val = self.val - self.increment
        self.__enforceRange()
        
    def set(self, val):
        self.val = val
        self.__enforceRange()
        
    def __enforceRange(self):
        if self.val > self.upper_bound:
            self.val = self.upper_bound
        elif self.val < self.lower_bound:
            self.val = self.lower_bound

class CircleCounter(object):
    
    def __init__(self, start_value = 0, increment = 0.1, modulo = 1):
        self.val = start_value % modulo
        self.increment = increment
        self.modulo = modulo
        
    def value(self):
        return self.val
        
    def increase(self):
        self.val = (self.val + self.increment) % self.modulo
        
    def decrease(self):
        self.val = (self.val - self.increment) % self.modulo
        
    def set(self, val):
        self.val = val % self.modulo
