import time, sys, msvcrt
from struct import *
from serial import *
from math import *
from xbee import XBee
from lib.dictionaries import *
from lib.command_interface import CommandInterface
from lib.telemetry_reader import TelemetryReader
from lib.network_coordinator import NetworkCoordinator
from lib.quaternion import *
from lib.keyboard_interface import KeyboardInterface
from lib.increment_counter import IncrementCounter
from lib.joystick_interface import JoystickInterface
import pygame

class HallController(object):
    def __init__(self, num_setpoints = 0):
        self.hall_gains = [0, 0, 0, 0, 0]
        self.duration = 0
        self.cycle = 56 #ms for 18Hz wing stroke (possibly roll this in to the thrust settings
        self.delta = [0] * num_setpoints
        self.interval = [0] * num_setpoints
        self.vel = [0] * num_setpoints
        self.counts = 45
        self.set = 0
        self.num_setpoints = num_setpoints
        self.thrust = IncrementCounter( start_value = 13.0, range = (18.0, 0.0), increment = 1.0 )
        print self.num_setpoints

    def setVelProfile(self, ang, percent_seg, thrust):
        if thrust == 0:
            self.cycle = 0
        else:
            self.cycle = int((1000/thrust))
        
        sum = 0
        self.delta[0] = (ang[0]*self.counts)/360
        sum = self.delta[0]
        for i in range(1,len(ang)-1):
            self.delta[i] = ((ang[i] - ang[i-1])*self.counts)/360
            sum = sum + self.delta[i]
        self.delta[len(ang)-1] = self.counts-sum

        sum = 0
        for i in range(0,len(percent_seg)):
            self.interval[i] = (percent_seg[i]*self.cycle)/100
            sum = sum + self.interval[i]
            self.vel[i] = (self.delta[i] << 8)/self.interval[i]
        self.interval[len(percent_seg)-1] = self.interval[len(percent_seg)-1] + self.cycle - sum

        self.checkParams()

    def setGains(self, gains):
        for i in range(0,len(gains)):
            self.hall_gains[i] = gains[i]

    def checkParams(self):
        if len(self.vel) == len(self.interval) == len(self.delta):
            self.set = 1

def txCallback(dest, packet):
    global xb
    xb.tx(dest_addr = dest, data = packet)
    
def rxCallback(packet):
    global telem, coord
    telem.processPacket(packet)
    coord.processPacket(packet)
    
def loop():

    global xb, telem, coord

    DEFAULT_COM_PORT = 'COM3'
    DEFAULT_BAUD_RATE = 57600
    DEFAULT_ADDRESS = '\x10\x21'
    DEFAULT_PAN = 0x1005
    
    if len(sys.argv) == 1:
        com = DEFAULT_COM_PORT
        baud = DEFAULT_BAUD_RATE
        addr = DEFAULT_ADDRESS
    elif len(sys.argv) == 4:
        com = sys.argv[1]
        baud = int(sys.argv[2])
        addr = pack('>H', int(sys.argv[3], 16))
    else:
        print "Wrong number of arguments. Must be: COM BAUD ADDR"
        sys.exit(1)
    
    ser = Serial(port = com, baudrate = baud) 
    xb = XBee(ser, callback = rxCallback)
    print "Setting PAN ID to " + hex(DEFAULT_PAN)
    xb.at(command = 'ID', parameter = pack('>H', DEFAULT_PAN))                 
    
    comm = CommandInterface(addr, txCallback)
    telem = TelemetryReader(addr, txCallback)
    kbint = KeyboardInterface(comm)
    coord = NetworkCoordinator(txCallback)
    hall = HallController(4)
    
    comm.enableDebug()
    telem.setConsoleMode(True)
    telem.setFileMode(True)
    telem.writeHeader()
    coord.resume()
    
    comm.setSlewLimit(3.0)

    try:
        pygame.init()
        j = pygame.joystick.Joystick(0)
        j.init()
        print j.get_name()
        print j.get_numaxes()
        print j.get_numballs()
        print j.get_numbuttons()
        print j.get_numhats()
        joyst = JoystickInterface(comm, hall, j)
    except Exception as e:
        print e
    
    while True:
        try:
            joyst.process()
            c = None
            if( msvcrt.kbhit() ):
               c = msvcrt.getch()
            kbint.process(c)
            time.sleep(0.01)
            #comm.sendPing()

        except Exception as e:
            print e
            break
            
    telem.close()                
    xb.halt()
    ser.close()
    

if __name__ == '__main__':
    
    try:        
        loop()
        
    except Exception as e:
        print e
        
    finally:
        #Add cleanup code!
        pass
