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

hall_gains = [0, 0, 0, 0, 0]
duration = 0
cycle = 56 #ms for 18Hz wing stroke (possibly roll this in to the thrust settings
ang = [0, 0, 0, 0]
percent_int = [0, 0, 0, 0]
vel = [0, 0, 0, 0]
num_points = 4
thrust = 0

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
        self.thrust = IncrementCounter( start_value = 10.0, range = (18.0, 0.0), increment = 2.0 )
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
        
class KeyboardInterface(object):
    
    def __init__(self, comm_interface = None, hall_controller = None):
        self.comm = comm_interface
        self.hall = hall_controller
        # Attitude state
        self.yaw = IncrementCounter( start_value = 0.0, range = (180.0, -180.0), increment = 5.0 )
        self.pitch = IncrementCounter( start_value = 0.0, range = (90.0, -90.0), increment = 10.0 )    
        self.roll = IncrementCounter( start_value = 0.0, range = (180.0, -180.0), increment = 10.0)
        # Slew state
        self.yaw_rate = IncrementCounter( start_value = 0.0, range = (10.0, -10.0), increment = 0.25 )
        self.pitch_rate = IncrementCounter( start_value = 0.0, range = (10.0, -10.0), increment = 0.25 )
        self.roll_rate = IncrementCounter( start_value = 0.0, range = (10.0, -10.0), increment = 0.25 )    
        # Actuator state
        self.elevator = IncrementCounter( start_value = 0.0, range = (1.0, -1.0), increment = 0.2 )
        self.thrust = IncrementCounter( start_value = 0.0, range = (1.0, 0.0), increment = 0.05 )
        self.steer = IncrementCounter( start_value = 0.0, range = (1.0, -1.0), increment = 0.25 )        
        # PID constants        
        self.yaw_coeffs = [ 0.0,    0.0,    2.0,   0.0,    0.4,    1.0,    1.0] # For steer Ki 0.8
        self.pitch_coeffs = [ 0.0,    0.0,    3.0,   0.0,    0.2,    1.0,    1.0] # For elevator control
        #self.roll_coeffs = [ 0.0,    0.0,    -0.2,   0.0,    0.0,    1.0,    1.0] # For thrust control
        #self.pitch_coeffs = [ 0.0,    0.0,    0.0,   0.0,    0.0,    1.0,    1.0] # For elevator manual
        self.roll_coeffs = [ 0.0,    0.0,    0.0,   0.0,    0.0,    1.0,    1.0] # For thrust manual
        self.yaw_filter_coeffs = [ 3, 0, 0.0007, 0.0021, 0.0021, 0.0007, 1.0, 2.6861573965, -2.419655111, 0.7301653453]
        # self.yaw_filter_coeffs = [ 3, 0, 56.0701e-6, 168.2103e-6, 168.2103e-6, 56.0701e-6, 1, -2.8430, 2.6980, -0.8546]
        # State                               
        self.streaming = False
        self.rate_control = False
        self.rc_changed = False
        self.offsets_changed = False
        self.ref_changed = False
        self.rate_changed = False
        self.rot_changed = False
        self.freq_changed = False
        self.pinging = False        
        
    def process(self, c):
    
        if c != None:
            self.__handleKey(c)
            
        if self.pinging:
            self.comm.sendPing();                
            
    def __handleKey(self, c):
            
        # Reference commands
        if c == 'w':
            # self.elevator.increase()
            # self.offsets_changed = True 
            self.pitch.increase()
            # self.comm.rotateRefLocal(quatGenerate(radians(10), (0,1,0)))
            self.ref_changed = True
        elif c == 's':
            # self.elevator.decrease()
            # self.offsets_changed = True 
            self.pitch.decrease()
            #self.comm.rotateRefLocal(quatGenerate(radians(-10), (0,1,0)))
            self.ref_changed = True
        elif c == 'a':
            # self.yaw.decrease()
            # self.rot_changed = True
            # self.ref_changed = True
            # self.comm.rotateRefGlobal(quatGenerate(radians(-10), (0,0,1)))
            self.yaw_rate.decrease()                
            self.rate_changed = True
            
            # self.steer.decrease()
            # self.rc_changed = True
        elif c == 'd':
            # self.yaw.increase()
            # self.rot_changed = True
            # self.ref_changed = True
            # self.comm.rotateRefGlobal(quatGenerate(radians(10), (0,0,1)))
            self.yaw_rate.increase()            
            self.rate_changed = True
            
            # self.steer.increase()
            # self.rc_changed = True
        elif c == 'q':
            self.roll.increase()
            self.ref_changed = True
        elif c == 'e':
            self.roll.decrease()
            self.ref_changed = True
        # Telemetry commands        
        elif c == 'r':                
            self.comm.startSensorDump(1)          
        elif c == 'f':                
            self.comm.startSensorDump(0)
        elif c == 'v':
            self.comm.requestDumpData(0x80 + 0, 0x80 + 600, 72)            
        elif c == 't':
            self.comm.requestTelemetry()        
        elif c == 'y':
            self.streaming = not self.streaming
            telem.writeLine("-> Toggle Streaming\n")
            self.comm.toggleStreaming()
        # Regulator Modes
        elif c == '1':                
            self.comm.setRegulatorMode(RegulatorStates['Off'])
        elif c == '2':                
            self.comm.setRegulatorMode(RegulatorStates['Track Hall'])
        elif c == '3':                
            self.comm.setRegulatorMode(RegulatorStates['Remote Control'])                    
        elif c == '4':
            #self.comm.setRegulatorMode(RegulatorStates['Track Hall'])
            self.rate_control = not self.rate_control           
            self.comm.setRateMode(self.rate_control)
        elif c == '5':
            self.hall.setVelProfile([90, 180, 270, 360], [25, 25, 25, 25], 5)
            self.comm.setVelProfile(self.hall.delta+self.hall.interval+self.hall.vel, self.hall.num_setpoints)
            self.comm.setHallGains([5,0.2,100,0,900])
            self.comm.setHallThrust(5,10000)
            #self.comm.calibWings()
        elif c == '6':
            self.comm.hallPIDOn()
            #self.comm.setWingStop(1)
        elif c == '7':
            self.hall.setVelProfile([90, 180, 270, 360], [25, 25, 25, 25], 10)
            self.comm.setVelProfile(self.hall.delta+self.hall.interval+self.hall.vel, self.hall.num_setpoints)
            self.comm.setHallThrust(10,10000)
        elif c == '8':
            self.hall.setVelProfile([90, 180, 270, 360], [25, 25, 25, 25], 15)
            self.comm.setVelProfile(self.hall.delta+self.hall.interval+self.hall.vel, self.hall.num_setpoints)
            self.comm.setHallThrust(15,10000)
        elif c == '9':
            self.hall.setVelProfile([90, 180, 270, 360], [25, 25, 25, 25], 5)
            self.comm.setVelProfile(self.hall.delta+self.hall.interval+self.hall.vel, self.hall.num_setpoints)
            self.comm.setHallThrust(5,10000)
        elif c == '0':
            #self.pinging = not self.pinging
            self.comm.setRegulatorRef((1.0, 0.0, 0.0, 0.0))
            
        # Attitude
        elif c == 'c':
            self.comm.runGyroCalib(1000);            
        elif c == 'z':
            self.comm.zeroEstimate()                                  
        # Configuration
        elif c == 'p':                
            self.comm.setRegulatorPid( self.yaw_coeffs + self.pitch_coeffs + self.roll_coeffs )                                
            self.comm.setRegulatorRateFilter( self.yaw_filter_coeffs )
            self.comm.setHallGains([5,0.2,100,0,900])
            self.comm.setTelemetrySubsample(1)
        elif c == ']':
            self.hall.thrust.increase()
            self.freq_changed = True
            #self.thrust.increase()
            #self.rc_changed = True 
            #self.offsets_changed = True                        
        elif c == '[':
            self.hall.thrust.decrease()
            self.freq_changed = True
            #self.thrust.decrease()
            #self.rc_changed = True 
            #self.offsets_changed = True            
        elif c == '\x1b': #Esc key
            #break
            raise Exception('Exit')
        
        if self.rc_changed:        
            self.comm.setRemoteControlValues( self.thrust.value(), self.steer.value(), self.elevator.value() );             
            self.rc_changed = False
            
        if self.offsets_changed:
            self.comm.setRegulatorOffsets((self.steer.value(), self.elevator.value(), self.thrust.value()))
            self.offsets_changed = False
            
        if self.rate_changed:
            self.comm.setGlobalRateSlew( ( self.yaw_rate.value(), self.pitch_rate.value(), self.roll_rate.value() ) )
            self.rate_changed = False
        
        if self.ref_changed:                    
            print   "Yaw, Pitch, Roll: " + str(self.yaw.value()) + " " + \
                    str(self.pitch.value()) + " " + str(self.roll.value())
            self.comm.setRegulatorRef( eulerToQuaternionDeg( self.yaw.value(), self.pitch.value(), self.roll.value() ) )        
            self.ref_changed = False

        if self.rot_changed:
            print "Yaw Rot: " + str(self.yaw.value())
            self.comm.setTempRot(quatGenerate(radians(self.yaw.value()), (0,0,1)))
            self.rot_changed = False

        if self.freq_changed:
            print "Thrust: " + str(self.hall.thrust.value())
            self.hall.setVelProfile([90, 180, 270, 360], [25, 25, 25, 25], self.hall.thrust.value())
            self.comm.setVelProfile(self.hall.delta+self.hall.interval+self.hall.vel, self.hall.num_setpoints)
            self.comm.setHallThrust(self.hall.thrust.value(),1)
            self.freq_changed = False

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

    hall = HallController(num_points)
    #hall.setVelProfile(ang, percent_int, thrust)
    
    kbint = KeyboardInterface(comm,hall)
    coord = NetworkCoordinator(txCallback)
    
    comm.enableDebug()
    telem.setConsoleMode(True)
    telem.setFileMode(True)
    telem.writeHeader()
    coord.resume()
    
    comm.setSlewLimit(3.0)
    
    while True:

        try:
            c = None
            if( msvcrt.kbhit() ):
               c = msvcrt.getch()
            kbint.process(c)
            time.sleep(0.01)
            #comm.sendPing()

        except:        
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
