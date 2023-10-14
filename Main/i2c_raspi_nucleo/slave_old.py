import time
import pigpio
import os
import sys


SDA=10
SCL=11

I2C_ADDR=0x48

goal = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "1", "2", "3"]


def i2c(id, tick):
   global pi
   i = 0
   s, b, d = pi.bsc_i2c(I2C_ADDR)
   if b :
        if d[0] == ord('y'):
            print(chr(d[0]))
            while (i<12):
                s, b, d = pi.bsc_i2c(I2C_ADDR, goal[i])
                i = i +1

pi = pigpio.pi()

if not pi.connected:
    exit()

pi.set_pull_up_down(SDA, pigpio.PUD_UP)
pi.set_pull_up_down(SCL, pigpio.PUD_UP)

e = pi.event_callback(pigpio.EVENT_BSC, i2c)

pi.bsc_i2c(I2C_ADDR) # Configure BSC as I2C slave

time.sleep(10)

e.cancel()

pi.bsc_i2c(0) # Disable BSC peripheral

print("test")

pi.stop()

exit()
