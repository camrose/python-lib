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


def txCallback(dest, packet):
	global xb
	xb.tx(dest_addr = dest, data = packet)

def rxCallback(packet):
    global telem, coord
    telem.processPacket(packet)
    coord.processPacket(packet)

if __name__ == '__main__':

    DEFAULT_COM_PORT = 'COM3'
    DEFAULT_BAUD_RATE = 57600
    DEFAULT_ADDRESS = '\x10\x21'
    DEFAULT_PAN = 0x1005
    DEFAULT_CHANNEL = 0x12

    yaw_coeffs = [ 0.0,    0.0,    2.0,   0.0,    0.4,    1.0,    1.0]
    pitch_coeffs = [ 0.0,    0.0,    3.0,   0.0,    0.2,    1.0,    1.0] #Kp = -0.8
    roll_coeffs = [ 0.0,    0.0,    -0.2,   0.0,    0.0,    1.0,    1.0]        
    yaw_filter_coeffs = [ 3, 0, 0.0007, 0.0021, 0.0021, 0.0007, 1.0, 2.6861573965, -2.419655111, 0.7301653453]
	
    if len(sys.argv) == 1:
        com = DEFAULT_COM_PORT
	baud = DEFAULT_BAUD_RATE
	addr = DEFAULT_ADDRESS
	chan = DEFAULT_CHANNEL

    comm = CommandInterface(addr, txCallback)
    telem = TelemetryReader(addr, txCallback)
    coord = NetworkCoordinator(txCallback)
    
    comm.enableDebug()
    telem.setConsoleMode(True)
    telem.setFileMode(True)
    telem.writeHeader()
    coord.resume()
    
    ser = Serial(port = com, baudrate = baud) 
    xb = XBee(ser, callback = rxCallback)
    print "Setting CHANNEL to " + hex(chan)
    xb.at(command = 'CH', parameter = pack('>H', chan))
    print "Setting PAN ID to " + hex(DEFAULT_PAN)
    xb.at(command = 'ID', parameter = pack('>H', DEFAULT_PAN))

    

    raw_input("Press any key to begin reading telemetry...")
    comm.requestDumpData(0x80 + 0, 0x80 + 600, 66)

    raw_input("Press key when finished receiving...")

    telem.close()                
    xb.halt()
    ser.close()
    
