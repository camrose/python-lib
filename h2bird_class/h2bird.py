import time, sys, msvcrt
from struct import *
from serial import *
from math import *
from xbee import XBee
from lib.hall_controller import HallController
from lib.command_interface import CommandInterface

class h2bird:
    
    def __init__(self,address,xb):
        self.DEST_ADDR = address
        self.xb = xb
        self.has_hall = False
        self.hall_controller = None
        self.comm = CommandInterface(address,self.tx)
        
    def tx(self,status,type,data):
        payload = chr(status) + chr(type) + ''.join(data)
        self.xb.tx(dest_addr = self.DEST_ADDR, data = payload)
        
    def enable_hall(self,num_setpoints):
        self.has_hall = enable
        self.hall_controller = HallController(num_setpoints)
        
    def setThrust(self,num):
        if self.has_hall == True:
            self.comm.setHallThrust(num, 1)
        else:
            self.comm.setRegulatorOffsets((self.steer.value(), self.elevator.value(), self.thrust.value()))
    
    def startRun(self):
        if self.has_hall == True:
            self.comm.setRegulatorMode(RegulatorStates['Track Hall'])
        else:
            self.comm.setRegulatorMode(RegulatorStates['Stabilize'])
            
    def setPID(self,gains):
        if gains == 0:
            self.comm.setRegulatorPid( self.yaw_coeffs + self.pitch_coeffs + self.roll_coeffs )                                
            self.comm.setRegulatorRateFilter( self.yaw_filter_coeffs )
            self.comm.setHallGains([5,0.2,100,0,900])
            self.comm.setTelemetrySubsample(1)
            
    
def setupSerial(COMPORT , BAUDRATE , timeout = 3, rtscts = 0):
    print "Setting up serial ..."
    try:
        ser = serial.Serial(port = COMPORT, baudrate = BAUDRATE, \
                    timeout=timeout, rtscts=rtscts)
    except serial.serialutil.SerialException:
        print "Could not open serial port:",shared.BS_COMPORT
        sys.exit(1)
    
    shared.ser = ser
    ser.flushInput()
    ser.flushOutput()
    return XBee(ser, callback = xbee_received)