hall_gains = [0, 0, 0, 0, 0]
duration = 0
cycle = 56 #ms for 18Hz wing stroke (possibly roll this in to the thrust settings
ang = [0, 0, 0, 0]
percent_int = [0, 0, 0, 0]
vel = [0, 0, 0, 0]
num_points = 4
thrust = 0

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
        self.thrust = 0.0
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