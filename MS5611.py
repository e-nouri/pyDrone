## I recieved my MS5611 Pressure/Barometer Sensor not knowing what I was doing.
## After much trial and error and pooring over the datasheet quite extensively I figured out how to communicate with it using Python
## Below are my poor attempts a script to get the Pressure and Tempurature data from the MS5611
## The datasheet that I used can be found here: http://www.embeddedadventures.com/datasheets/ms5611.pdf
## All of the following values and calculations were taken from that datasheet.

## Note this script does require Root to run.

## Created By: Jonny Fosnight
## Date: 2013-03-18
## Email: jfosnight@live.com

## Feel free to use the script however you want.  If you would like to use it for a commercial project please contact me at the listed email address.


import time

## Import Libraries that let python talk to I2C devices
from smbus import SMBus

## Initialize the bus.  (If you have a Rev 2 Raspberry Pi the 0 needs to be changed to a 1)
bus = SMBus(0)

## This writes the command to the Sensor to calculate the digital value of pressure.  (This is a raw value and will
#       be used in a calculation later on in the script to get the actual real pressure.)
bus.write_byte(0x76, 0x48)
time.sleep(0.05)

## This command reads the data that we just calculated on the sensor
##  I had to use the read_i2c_block_data rather than the read_word or read_byte because the values being returned by the
#       sensor were 24 bit and was being truncated by the other read methods.  Reading the whole data block was the only way
#       that I knew to get all of the data.
D1 = bus.read_i2c_block_data(0x76, 0x00)  
time.sleep(0.05)


## This is much like above only it is getting the raw temperature.
bus.write_byte(0x76, 0x58)
time.sleep(0.05)
D2 = bus.read_i2c_block_data(0x76, 0x00)
time.sleep(0.05)


## The data being read in blocks are in an array in 8 bit pieces, so we need to convert the first 24 bits into decimal,
#       which is what this statement does.
D1 = D1[0] * 65536 + D1[1] * 256.0 + D1[2]
D2 = D2[0] * 65536 + D2[1] * 256.0 + D2[2]


## The MS6511 Sensor stores 6 values in the EPROM memory that we need in order to calculate the actual temperature and pressure
## These values are calculated/stored at the factory when the sensor is calibrated.
##      I probably could have used the read word function instead of the whole block, but I wanted to keep things consistent.
C1 = bus.read_i2c_block_data(0x76, 0xA2) #Pressure Sensitivity
time.sleep(0.05)
C2 = bus.read_i2c_block_data(0x76, 0xA4) #Pressure Offset
time.sleep(0.05)
C3 = bus.read_i2c_block_data(0x76, 0xA6) #Temperature coefficient of pressure sensitivity
time.sleep(0.05)
C4 = bus.read_i2c_block_data(0x76, 0xA8) #Temperature coefficient of pressure offset
time.sleep(0.05)
C5 = bus.read_i2c_block_data(0x76, 0xAA) #Reference temperature
time.sleep(0.05)
C6 = bus.read_i2c_block_data(0x76, 0xAC) #Temperature coefficient of the temperature
time.sleep(0.05)

## Again here we are converting the 2 8bit packages into a single decimal
C1 = C1[0] * 256.0 + C1[1]
C2 = C2[0] * 256.0 + C2[1]
C3 = C3[0] * 256.0 + C3[1]
C4 = C4[0] * 256.0 + C4[1]
C5 = C5[0] * 256.0 + C5[1]
C6 = C6[0] * 256.0 + C6[1]

## These are the calculations provided in the datasheet for the sensor.
## TEMP is the actual temperature reported in Centigrade.  It is reported to the 1/100ths, so it needs to be divided by 100 to be used by us.
dT = D2 - C5 * 2**8
TEMP = 2000 + dT * C6 / 2**23

## I also included Fahrenheit for the rest of us normal people
TEMP_F = TEMP/100.0 * 9.0/5 + 32

print "Temperature: ", TEMP/100.0
print "Temperature F: ", round(TEMP_F, 2)

## These calculations are all used to produce the final pressure value (just as before the Pressure value needs to be divided by 100 to shift the decimal place where it belongs.)
OFF = C2 * 2**16 + (C4 * dT) / 2**7
SENS = C1 * 2**15 + (C3 * dT) / 2**8
P = (D1 * SENS / 2**21 - OFF) / 2**15


print "Pressure: ", P/100.0

## Now one curious thing I found was that the sensor needs to be adjusted to your current eleivation.  I couldn't find an automatic way to do it, but I did find
#       this website with a chart. http://www.novalynx.com/manuals/bp-elevation-correction-tables.pdf
#   Just replace the 55.5 below with whatever elevation is approapriate for you.
print "Pressure Adjusted: ", round(P/100.0 + 55.5, 2)
