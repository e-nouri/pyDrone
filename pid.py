from time import time,sleep


class pid(object):

    def __init__(self,kp,ki,kd,maxCorr=5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previousTime = 0.0
        self.I = 0
        self.P =0
        self.D = 0
        self.previousError = 0.0
        self.init=True
        self.maxCorr=maxCorr


    def calc(self, error):
        if self.init:
            #the first time do not calculate pid correction, but init the time data
            self.previousTime = time()
            self.init=False
            return 0
        else:
            currentTime = time()
            stepTime = currentTime - self.previousTime

            self.P = error * self.kp
            self.I += (error * stepTime) * self.ki
            self.D = (error - self.previousError) / stepTime * self.kd


            correction = self.P + self.I + self.D
            self.previousTime = currentTime
            self.previousError = error
            #since W is an integer, correction is rounded
            correction = round(correction)

            if correction>self.maxCorr:
                correction=self.maxCorr
            if correction<-self.maxCorr:
                correction=-self.maxCorr
            return correction, self.P, self.I, self.D
