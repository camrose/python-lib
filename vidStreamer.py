import threading, time, sys, msvcrt, datetime
from struct import *
from Tkinter import *
from PIL import Image, ImageDraw
from ImageWin import Dib, HWND
from serial import *
from xbee import XBee
#from lib.basestation import BaseStation
from lib.payload import Payload
from lib.telemetry_reader import TelemetryReader
from lib.network_coordinator import NetworkCoordinator
from lib.dictionaries import *
from lib.command_interface import CommandInterface
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque

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
        
#end class

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
        
        self.et = IncrementCounter( start_value = 351, range = (0, 2000), increment = 5 )
        self.fs = IncrementCounter( start_value = 10000, range = (0, 100000), increment = 200 )
        
        
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
                print "Line:" + str(line_num)
                curr_line.extend([curr_time])
                dat.append(curr_line)
                self.writeBlock(row, 0, 128, curr_line)
                curr_line = []
                line_num = line_num + 1
        
        if type == Commands['LINE_EDGE_RESPONSE']:
            #print "Data len: " + str(len(data))
            data_flag = str((len(data)-16)) + 'B'
            #raw = unpack('2H', data)
            raw = unpack('2HL2f2B', data)
            print raw
            
            frame_num = raw[0]
            frame_num = raw[1]
            timestamp = raw[2]
            distance = raw[3]
            location = raw[4]
            locs = raw[5::]
            #locs = [0]
            curr_time = time.time() - st
            
            print 'Locs:' + str(locs) + '\n'
            curr_line = [0] * 128
            for i in range(0,len(locs)):
                if locs[i] != 0:
                    curr_line[locs[i]] = 255
            
            print str(frame_num)
            dat.append(locs)
            self.writeBlock(0,0,128,curr_line)
            
            
        elif type == Commands['CENTROID_REPORT']:
            raw = unpack('4H2B', data)            
            self.mergeImages()
            self.drawCentroid(raw[0:2])
            self.drawMax(raw[2:4])
            print "Max lum: " + str(raw[4]) + " avg: " + str(raw[5])
            self.updateImage()           
            self.displayFrameRate()
            
            
        else:
            print "Invdfhdfhalid command: " + str(type)                                
                        
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
        
def txCallback(dest, packet):
    global xb
    xb.tx(dest_addr = dest, data = packet)
    
if __name__ == '__main__':

    global xb, telem, coord, comm, dat, curr_line, line_num, st
    
    #DEFAULT_BAUD_RATE = 115200

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

    streamer = VideoStreamer(txCallback)
    ser = Serial(port = com, baudrate = baud)
    xb = XBee(ser, callback = streamer.processPacket)
    print "Setting PAN ID to " + hex(DEFAULT_PAN)
    xb.at(command = 'ID', parameter = pack('>H', DEFAULT_PAN))

    	
    telem = TelemetryReader(addr, txCallback)
    comm = CommandInterface(addr, txCallback)
    coord = NetworkCoordinator(txCallback)

    comm.enableDebug()
    telem.setConsoleMode(True)
    telem.setFileMode(True)
    telem.writeHeader()
    coord.resume()
    
    streamer.setEndpoint(addr)    
    time.sleep(0.25)    

    #raw_input("Press any key to capture background frame.")
    #coord.setBackgroundFrame()
    
    prev_time = time.time()
    file_index = 0;
    start_stream = 0
    
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
            if ((curr_time - prev_time) > 0.4) and start_stream:
                comm.requestLineFrames()
                #comm.requestLineEdges()
                streamer.updateImage()
                prev_time = curr_time;
                
            if msvcrt.kbhit():
                c = msvcrt.getch()
                if c == 'e':
                    print "Closing..."
                    break
                elif c == 'h':
                    comm.setHP(0)
                    print "High pass toggled."
                elif c == 'c':
                    streamer.save(str(file_index))
                    print "Image captured."
                    file_index = file_index + 1
                elif c == 'y':
                    comm.startLS(1)
                    #self.streaming = not self.streaming
                    #telem.writeLine("-> Toggle Streaming\n")
                    #self.comm.toggleStreaming()
                elif c == 'u':
                    comm.startLS(0)
                elif c == 'r':
                    if start_stream:
                        start_stream = 0
                    else:
                        start_stream = 1
                    #comm.requestLineFrames()
                    #streamer.decayIndicators()
                elif c == 's':
                    comm.setExposure(351, 12000)
                    #comm.setExposure(streamer.et.value(), streamer.fs.value())
                elif c == '1':
                    streamer.fs.decrease()
                    print "Freq: " + str(streamer.fs.value())
                elif c == '2':
                    streamer.fs.increase()
                    print "Freq: " + str(streamer.fs.value())
                elif c == '3':
                    streamer.et.decrease()
                    print "Exposure: " + str(streamer.et.value())
                elif c == '4':
                    streamer.et.increase()
                    print "Exposure: " + str(streamer.et.value())
                elif c == 'j':
                    comm.requestLineFrames()
                    streamer.updateImage()
                elif c == 'k':
                    comm.requestLineEdges()
                    streamer.updateImage()
                elif c == '\x1b': #Esc key
                    raise Exception('Exit')
                
            #time.sleep(0.1)
                
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
    streamer.close()
    xb.halt()
    ser.close()
