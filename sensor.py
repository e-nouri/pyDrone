
import math
from MPU6050 import MPU6050
from time import time,sleep
import threading

class sensor(threading.Thread):

    #TODO gestire logger

    def __init__(self,address=0x68,updateoffset=True, cycletime=0.01, savelog=False):

        threading.Thread.__init__(self)

        self.address=address
        #MPU6050 is an  specific interface to the hw used.
        #if the imu is different from MPU6050 it is enough to call the
        #correct interface here
        self.IMU=MPU6050(address)
        print 'IMU calibrating...'
        if updateoffset:
            self.IMU.updateOffsets('IMU_offset.txt')
        self.IMU.readOffsets('IMU_offset.txt')

        self.roll=0
        self.pitch=0
        self.yaw=0
        self.x_acc=0
        self.y_acc=0
        self.z_acc=0
        self.r_rate=0
        self.p_rate=0
        self.y_rate=0
        self.cycling=True
        self.cycletime=cycletime
        self.savelog = savelog
        self.datalog=''


    def run(self):

        self.datalog=''
        self.datalog='|time|dt'
        self.datalog +='|roll|pitch|yaw'
        self.datalog +='|r_rate|p_rate|y_rate'
        self.datalog +='|x_acc|y_acc|z_acc'
        self.datalog +='\n'



        inittime=time()
        tottime=0
        print 'IMU running...'
        while self.cycling:
            tottime_old=tottime
            tottime=time()-inittime
            steptime=tottime-tottime_old
            self.update(steptime)
            if self.savelog is True:
                self.datalog +=self.getDataString(tottime,steptime)

            #comment this for cycling as fast as possible
            #if steptime<self.cycletime:
                #sleep(self.cycletime-steptime)


    def stop(self):
        try:
            print 'IMU stopping...'
            self.cycling = False
            if self.savelog is True:
                sleep(0.1)
                with open('sensor_data.txt', 'w+') as data_file:
                    data_file.write(self.datalog)
                    data_file.flush()
        except IOError:
            pass


    def update(self,dt):
        self.x_acc, self.y_acc, self.z_acc, self.r_rate, self.p_rate, self.y_rate= self.IMU.readSensors()
        self.getAngleCompl(dt)


    def getDataString(self,data1='',data2=''):
        "return all the data as string , usefull for logging"
        s=''
        s='|'+str(data1)+'|'+str(data2)
        s +='|'+str(self.roll) +'|'+str(self.pitch) +'|'+str(self.yaw)
        s +='|'+str(self.r_rate) +'|'+str(self.p_rate) +'|'+str(self.y_rate)
        s +='|'+str(self.x_acc) +'|'+str(self.y_acc) +'|'+str(self.z_acc)
        s +='\n'
        return s


    def getAngleGyro(self,dt):
        "return the angle calculated on the gyro.not used"
        new_r=self.roll+self.r_rate*dt
        new_p=self.pitch+self.p_rate*dt
        new_y=self.yaw+self.y_rate*dt
        return new_r,new_p,new_y


    def getAngleAcc(self):
        "return the angle calculated on the accelerometer."
        pi=3.141592
        #ATTENTION atan2(y,x) while in excel is atan2(x,y)
        r=math.atan2(self.y_acc,self.z_acc)*180/pi
        p=math.atan2(self.x_acc,self.z_acc)*180/pi
        #Note that yaw value is not calculable using acc info
        #function returns a value just for keep a consistent structure
        y=0

        return r,p,y

    def getAngleCompl(self,dt):
        "return the angle calculated applying the complementary filter."
        tau=0.2
        #tau is the time constant in sec
        #for time periods < tau the  gyro takes precedence
        #for time periods > tau the acc takes precedence

        new_r,new_p,new_y=self.getAngleAcc()
        a=tau/(tau+dt)
        self.roll=a*(self.roll+self.r_rate*dt)+(1-a)*new_r
        self.pitch=a*(self.pitch+self.p_rate*dt)+(1-a)*new_p
        #note the yaw angle can be calculated only using the
        # gyro data, so a=1 for yaw.(it means that yaw value is affected by drift)
        a=1
        self.yaw=a*(self.yaw+self.y_rate*dt)+(1-a)*new_y
        #Rounding the results
        self.roll=(round(self.roll,2))
        self.pitch=(round(self.pitch,2))
        self.yaw=(round(self.yaw,2))


