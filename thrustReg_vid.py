import threading, time, sys, msvcrt, datetime
from struct import *
from serial import *
from math import *
from xbee import XBee
from lib.dictionaries import *
from lib.command_interface import CommandInterface
from lib.telemetry_reader_line import TelemetryReader
from lib.network_coordinator import NetworkCoordinator
from lib.payload import Payload
from lib.quaternion import *
from Tkinter import *
from PIL import Image, ImageDraw
from ImageWin import Dib, HWND
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque

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

class WindowManager(threading.Thread):
    
    def __init__(self, targetWindow):
        threading.Thread.__init__(self)
        self.window = targetWindow
        
    def run(self):
        self.window.mainloop()
        
class ImageView(Frame):

    def __init__(self, master, **options):
        Frame.__init__(self, master, **options)
        self.pack()
        self.dib = None

    def setimage(self, image):
        self.config(bg="") # don't update the background
        self.dib = Dib(image)
        self.dib.expose(HWND(self.winfo_id()))
            
#end class

class VideoStreamer(object):
    
    DEFAULT_DESTINATION = '\x11\x01'    
    INDICATOR_DECAY = 30
    
    def __init__(self, callback):
        self.tx_callback = callback
        
        # Transmitter Parameters
        self.block_size = 75

        # Image Parameters
        self.capture_width = 128
        self.capture_height = 1
        self.frame_cnt = 0
        self.shown = 0
        
        self.scale = 1
        self.hardware_col_subsample = 1
        self.hardware_row_subsample = 1
        
        self.dx = self.scale*self.hardware_col_subsample;
        self.dy = self.scale;
        
        self.x_vals = deque([0.0]*128)
        self.y_vals = deque([0.0]*128)
        
        self.display_width = self.capture_width * self.scale        
        self.display_height = self.capture_height * self.scale        
             
        # Radio parameters
        self.endpoint_addr = VideoStreamer.DEFAULT_DESTINATION
        
        # Viewer window
        self.root = Tk()
        self.stream = ImageView(self.root, width = self.display_width, height = 128)
        self.manager = WindowManager(self.root)
        self.manager.start()
        self.line = []
        
        self.frame = Image.new('L',(self.display_width, 128))
        self.cframe = Image.new('L',(self.display_width, 128))
        self.frame_array = self.frame.load()
        self.cframe_array = self.cframe.load()
        self.frame_draw = ImageDraw.Draw(self.frame)

        # Other state
        self.lastFrame = 0		
        self.lastFrameTime = time.clock()
        plt.ion()
        #plt.show()
        
        self.et = IncrementCounter( start_value = 501, range = (0, 2000), increment = 5 )
        self.fs = IncrementCounter( start_value = 6000, range = (0, 100000), increment = 200 )
        
        
        print("capture width: " + str(self.capture_width) + " height: " + str(self.capture_height))
        print("Display width: " + str(self.display_width) + " height: " + str(self.display_height))
        print("Scale: " + str(self.scale))

    def save(self, filename):
        self.frame.save(filename + ".png", "PNG")
        
    def close(self):        
        pass
        
    def setEndpoint(self, addr):
        print "Setting endpoint address of: " + hex(unpack('>H', addr)[0])
        self.endpoint_addr = addr        
        
    def processPacket(self, packet):
        
        global dat, curr_line, line_num, st
        
        pld = Payload(packet.get('rf_data'))
        type = pld.type
        data = pld.data
        status = pld.status

        if type == Commands['LINE_FRAME_RESPONSE']:
            #print "Data len: " + str(len(data))
            data_flag = str((len(data)-6)) + 'B'
            raw = unpack('HHH' + data_flag, data)
            
            frame_num = raw[0]
            row = raw[1]*self.hardware_row_subsample
            col = raw[2]
            pixels = raw[3::]
            curr_time = time.time() - st
            
            #print 'Line:' + str(len(curr_line)) + ' ' + 'Pix:' + str(len(pixels)) + '\n'
            if len(curr_line) == 0 and len(pixels) == 75:
                curr_line.extend(pixels)
            elif len(curr_line) == 0 and len(pixels) == 53:
                curr_line = []
            elif len(curr_line) == 75 and len(pixels) == 75:
                curr_line = []
            elif len(curr_line) == 75 and len(pixels) == 53:
                curr_line.extend(pixels)
            else:
                print 'Fail'
                
            
            #print 'Pixels:' + str(len(pixels)) + '\n'
            #print str(pixels) + '\n'
            
            #if (col == 75):
            #    self.frame_cnt+=1
            
            #plt.plot(range(col,col+len(pixels)), pixels)
            #shift = self.frame_cnt*128
            ##if (self.shown == 0):
            ##    self.shown = 1
            ##    plt.show()
            #a = data[(x_start-x_start):(x_end-x_start)]
            #if(x_start == 0):
            #    #b = [0,0,0,0,0,0,0,0,0,0]
            #    #b.extend(a)
            #    b = a
            #    plt.plot(range(x_start+shift,x_end+shift), a)
            #    #plt.axis([x_start+shift-1024,x_end+shift+1024,0,255])
            #else:
            #    b = a
            #    plt.plot(range(x_start+shift,x_end+shift), a)
            #    #plt.axis([x_start+shift-1024,x_end+shift+1024,0,255])
            #plt.axis([0,2024,0,255])
            #plt.draw()
            
            
                                          
            ##print "Received row: " + str(row) + " col: " + str(col)
            #np.insert(self.px_vals,col,pixels)
            #print self.px_vals
            
            if len(curr_line) == 128:
                #print str(curr_line)
                print str(line_num)
                curr_line.extend([curr_time])
                dat.append(curr_line)
                self.writeBlock(row, 0, 128, curr_line)
                curr_line = []
                line_num = line_num + 1
        
        if type == Commands['LINE_EDGE_RESPONSE']:
            #print "Data len: " + str(len(data))
            data_flag = str((len(data)-6)) + 'B'
            raw = unpack('LH' + data_flag, data)
            
            timestamp = raw[0]
            frame_num = raw[1]
            locs = raw[2::]
            curr_time = time.time() - st
            
            print 'Locs:' + str(locs) + '\n'
            curr_line = [0] * 128
            for i in range(0,len(locs)):
                if locs[i] != 0:
                    curr_line[locs[i]] = 255
            
            print str(frame_num)
            self.writeBlock(0,0,128,curr_line)
            
            
        elif type == Commands['CENTROID_REPORT']:
            raw = unpack('4H2B', data)            
            self.mergeImages()
            self.drawCentroid(raw[0:2])
            self.drawMax(raw[2:4])
            print "Max lum: " + str(raw[4]) + " avg: " + str(raw[5])
            self.updateImage()           
            self.displayFrameRate()                              
                        
    def displayFrameRate(self):
        now = time.clock()
        rate = 1.0/(now - self.lastFrameTime)
        print "Framerate: " + str(rate) + " fps\n"
        self.lastFrameTime = now
        
    def writeBlock(self, row, col_start, to_go, data):
        #x = horizontal, y = vertical
        x_start = col_start*self.hardware_col_subsample*self.scale
        x_end = x_start + to_go*self.hardware_col_subsample*self.scale
        y_start = row*self.scale
        y_end = y_start + 127
        
        print str(y_start)
        print str(y_end)
        print str(x_start)
        print str(x_end)
        for y in range(y_start, y_end):
            for x in range(x_start, x_end):
                #print str([x,y])
                self.frame_array[x, y] = data[(x - x_start)/(self.hardware_col_subsample*self.scale)]
                
        
        
            
        
        #shift = self.frame_cnt*128 + self.frame_cnt*10
        #a = data[(x_start-x_start):(x_end-x_start)]
        #
        #if(x_start == 0):
        #    b = [0,0,0,0,0,0,0,0,0,0]
        #    a.extend(a)
        #    #b = a
        #    plt.plot(range(x_start+shift,x_end+shift+10), b)
        #    #plt.axis([x_start+shift-1024,x_end+shift+1024,0,255])
        #else:
        #    #b = [0,0,0,0,0,0,0,0,0,0]
        #    #b.extend(a)
        #    #b = a
        #    plt.plot(range(x_start+shift+10,x_end+shift+10), a)
        #    #plt.axis([x_start+shift-1024,x_end+shift+1024,0,255])
        #plt.axis([0,2024,0,255])
        #plt.draw()
        #
        #if (x_start == 75):
        #    self.frame_cnt+=1
        ## Write indicator pixels        
        ##for y in range(y_start, y_end):
        ##    for x in range(0, self.dx):
        ##        self.cframe_array[x, y] = 0x0000FF
            
    def drawCentroid(self, centroid):
        x_start = centroid[0]*self.dx;
        y_start = centroid[1]*self.dy*self.hardware_row_subsample;
        print "Centroid at: " + str(centroid)        
        for y in range(y_start, y_start + self.dy):            
            for x in range(x_start, x_start + self.dx):                
                self.cframe_array[x, y] = 0x0000FF
                
    def drawMax(self, max):
        x_start = max[0]*self.dx;
        y_start = max[1]*self.dy*self.hardware_row_subsample;
        print "Max at: " + str(max)        
        for y in range(y_start, y_start + self.dy):            
            for x in range(x_start, x_start + self.dx):                
                self.cframe_array[x, y] = 0x00FF00
    
                
    def mergeImages(self):
        self.cframe.paste(self.frame, [self.dx, 0, \
                    self.display_width + self.dx, self.display_height])        
            
    def updateImage(self):                
        self.stream.setimage(self.frame)
        #print self.frame_array[0]
        #plt.plot(range(0,128), self.line)
        #plt.axis([0,2024,0,255])
        #plt.draw()
        
              
    
    def decayIndicators(self):
        for y in range(0, self.display_height - 1):            
            for x in range(0, self.dx):            
                pixel = self.cframe_array[x, y];                
                if pixel >= (self.INDICATOR_DECAY,0,0):
                    self.cframe_array[x, y] = (pixel[0] - self.INDICATOR_DECAY, 0, 0)
                else:
                    self.cframe_array[x, y] = (0, 0, 0)
        self.updateImage()
        
    def showFrame(self):
        for y in range(0, self.display_height - 1):            
            for x in range(0, self.dx):            
                pixel = self.cframe_array[x, y];                
                if pixel >= (self.INDICATOR_DECAY,0,0):
                    self.cframe_array[x, y] = (pixel[0] - self.INDICATOR_DECAY, 0, 0)
                else:
                    self.cframe_array[x, y] = (0, 0, 0)
        self.updateImage()

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
        self.thrust = IncrementCounter( start_value = 11.0, range = (18.0, 0.0), increment = 0.5 )
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
        self.lineThrust = IncrementCounter( start_value = 0.0, range = (1.0, 0.0), increment = 0.05 )
        self.steer = IncrementCounter( start_value = 0.0, range = (1.0, -1.0), increment = 0.25 )        
        # PID constants        
        self.line_coeffs = [ 63.5,    0.0,    0.01,   0.0,    0.0002,    1.0,    1.0] # For steer Ki 0.8
        self.yaw_coeffs = [ 0.0,    0.0,    2.0,   0.0,    0.4,    1.0,    1.0]
        #self.yaw_coeffs = [ 0.0,    0.0,    0.0,   0.0,    0.0,    1.0,    1.0]
        self.pitch_coeffs = [ 0.0,    0.0,    3.0,   0.0,    0.2,    1.0,    1.0] # For elevator control
        #self.roll_coeffs = [ 0.0,    0.0,    -0.2,   0.0,    0.0,    1.0,    1.0] # For thrust control
        #self.pitch_coeffs = [ 0.0,    0.0,    0.0,   0.0,    0.0,    1.0,    1.0] # For elevator manual
        #self.roll_coeffs = [ 0.0,    0.0,    0.0,   0.0,    0.0,    1.0,    1.0] # For thrust manual
        self.roll_coeffs = [ 0.0,    0.0,    0.0,   0.0,    0.0,    1.0,    1.0] # For thrust with line track (ref in meters)
        self.yaw_filter_coeffs = [ 3, 0, 0.0007, 0.0021, 0.0021, 0.0007, 1.0, 2.6861573965, -2.419655111, 0.7301653453]
        self.pitch_filter_coeffs = [ 3, 1, 0.0007, 0.0021, 0.0021, 0.0007, 1.0, 2.6861573965, -2.419655111, 0.7301653453]
        self.line_filter_coeffs = [ 3, 3, 0.0007, 0.0021, 0.0021, 0.0007, 1.0, 2.6861573965, -2.419655111, 0.7301653453]
        # self.yaw_filter_coeffs = [ 3, 0, 56.0701e-6, 168.2103e-6, 168.2103e-6, 56.0701e-6, 1, -2.8430, 2.6980, -0.8546]
        
        self.strobe_params = [ 2, 0, 2, 2 ]
        self.line_height_coeffs = [1.6,  0.0,  0.0,  0.05,  0.0, 1.0, 1.0]
        self.thresh = (15,10)
        self.window = 7
        
        
        
        
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
            self.comm.requestDumpData(0x80 + 0, 0x80 + 7000, 76)            
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
            self.comm.setRegulatorMode(RegulatorStates['Stabilize'])
        elif c == '3':                
            self.comm.setRegulatorMode(RegulatorStates['Remote Control'])                    
        elif c == '4':
            #self.comm.setRegulatorMode(RegulatorStates['Track Hall'])
            self.rate_control = not self.rate_control           
            self.comm.setRateMode(self.rate_control)
        elif c == '5':
            self.comm.setLineRef((63.5,2.6))
        elif c == '6':
            self.comm.setExperiment(1)
        elif c == '7':
            #self.comm.toggleExperiment(1)
            #self.comm.setRegulatorOffsets((self.steer.value(), self.elevator.value(), 0.2))
            self.comm.setRegulatorOffsets((self.steer.value(), self.elevator.value(), 0.3))
        elif c == '8':
            #self.comm.toggleExperiment(0)
            #self.comm.setRegulatorOffsets((self.steer.value(), self.elevator.value(), self.thrust.value()))
            self.comm.setRegulatorOffsets((self.steer.value(), self.elevator.value(), 1.0))
        elif c == '9':
            #self.comm.setRegulatorOffsets((self.steer.value(), self.elevator.value(), 1.0))
            self.comm.setRegulatorRef( eulerToQuaternionDeg( 0.0, self.pitch.value(), self.roll.value() ) )
        elif c == '0':
            #self.comm.setRegulatorRef( eulerToQuaternionDeg( 180.0, self.pitch.value(), self.roll.value() ) )
            #self.pinging = not self.pinging
            self.comm.setSlewLimit(5.0)
            self.comm.toggleFigureEight(1)
            
        # Attitude
        elif c == 'c':
            self.comm.runGyroCalib(1000);            
        elif c == 'z':
            self.comm.zeroEstimate()                                  
        # Configuration
        elif c == 'p':                
            self.comm.setRegulatorPid( self.yaw_coeffs + self.pitch_coeffs + self.roll_coeffs )
            self.comm.setLinePid( self.line_coeffs + self.line_height_coeffs )                                
            self.comm.setRegulatorRateFilter( self.yaw_filter_coeffs )
            self.comm.setRegulatorRateFilter( self.pitch_filter_coeffs )
            self.comm.setRegulatorRateFilter( self.line_filter_coeffs )
            self.comm.setEmptyThreshold( self.thresh )
            self.comm.setHeightFilterWindow(self.window)
            #self.comm.setHallGains([5,0.2,100,0,900])
            self.comm.setTelemetrySubsample(1)
            self.comm.setExposure(351, 14000)
        elif c == ']':
            #self.hall.thrust.increase()
            #self.freq_changed = True
            self.thrust.increase()
            self.lineThrust.increase()
            self.rc_changed = True 
            self.offsets_changed = True                        
        elif c == '[':
            #self.hall.thrust.decrease()
            #self.freq_changed = True
            self.thrust.decrease()
            self.lineThrust.decrease()
            self.rc_changed = True 
            self.offsets_changed = True
        elif c == 'o':
            self.yaw.set(90.0)
            self.ref_changed = True           
        elif c == '\x1b': #Esc key
            #break
            raise Exception('Exit')
        
        if self.rc_changed:        
            self.comm.setRemoteControlValues( self.thrust.value(), self.steer.value(), self.elevator.value() );             
            self.rc_changed = False
            
        if self.offsets_changed:
            self.comm.setRegulatorOffsets((self.steer.value(), self.elevator.value(), self.thrust.value()))
            self.comm.setLineOffsets((0.0, self.lineThrust.value()))
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
    #streamer.processPacket(packet)
    telem.processPacket(packet)
    coord.processPacket(packet)
    
def loop():

    global xb, telem, coord, comm, dat, curr_line, line_num, st, streamer

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
    
    #streamer = VideoStreamer(txCallback)
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
    
    comm.setSlewLimit(0.0)
    
    #streamer.setEndpoint(addr)    
    time.sleep(0.25)
    
    prev_time = time.time()
    file_index = 0;
    start_stream = 0
    
    track_marker = 0
    
    today = datetime.datetime.today()
    d = str(today.year) + "_" + str(today.month) + "_" + str(today.day)
    t = str(today.hour) + "_" + str(today.minute) + "_" + str(today.second)
    fname = d + '-' + t + '-' + 'line' + '.txt'
    f = open(fname, 'w')
    
    dat = []
    curr_line = []
    line_num = 0
    
    st = time.time()
    
    while True:

        try:
            curr_time = time.time()
            if ((curr_time - prev_time) > 0.05) and start_stream:
                comm.requestLineFrames()
                #streamer.updateImage()
                comm.foundMarker()
                prev_time = curr_time;
            c = None
            if( msvcrt.kbhit() ):
                c = msvcrt.getch()
                if c == 'g':
                    comm.startLS(1)
                elif c == 'h':
                    comm.startLS(0)
                elif c == 'j':
                    if start_stream:
                        start_stream = 0
                    else:
                        start_stream = 1
                elif c == 'l':
                    if track_marker:
                        track_marker = 0
                        comm.trackMarker(0)
                    else:
                        track_marker = 1
                        comm.trackMarker(1)
                        comm.startStrobe(kbint.strobe_params)
                elif c == '\x1b': #Esc key
                    raise Exception('Exit')
            kbint.process(c)
            time.sleep(0.05)
            #comm.sendPing()

        except Exception as e:
            print e        
            break
            
    time.sleep(0.25)
    for row in dat:
        row_str = str(row)
        print row_str
        f.write(row_str.strip('[]') + "\n")
    f.close()
    telem.close()
    #streamer.close()                
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
