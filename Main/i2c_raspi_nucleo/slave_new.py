import time
import pigpio

SDA=10
SCL=11
pi=0
e=0
I2C_ADDR=0x48

#goal = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "1", "2", "3"]
#contoh
x=123
y=456
a=789
b=123
goal = ['0' for i in range(12)]

def int_to_char():
    global goal
    global x
    global y
    global a
    global b

    index=12
    while(x > 0):
        index -= 1
        goal[index]=chr(int(x%10 + 48))
    while(y > 0):
        index -= 1
        goal[index]=chr(int(x%10 + 48))
    while(a > 0):
        index -= 1
        goal[index]=chr(int(x%10 + 48))
    while(b > 0):
        index -= 1
        goal[index]=chr(int(x%10 + 48))
        
        
def i2c(id, tick):
    global pi
    global goal
    global I2C_ADDR

    i = 0
    s, b, d = pi.bsc_i2c(I2C_ADDR)
    if b :
        if d[0] == ord('y'):
            print(chr(d[0]))
            int_to_char()
            while (i<12):
                s, b, d = pi.bsc_i2c(I2C_ADDR, goal[i])
                i = i + 1

def i2c_initialize():
    global pi
    global I2C_ADDR

    pi = pigpio.pi()

    while not pi.connected:
        pi = pigpio.pi()

    pi.set_pull_up_down(SDA, pigpio.PUD_UP)
    pi.set_pull_up_down(SCL, pigpio.PUD_UP)

    e = pi.event_callback(pigpio.EVENT_BSC, i2c) # register for I2C callbacks

    pi.bsc_i2c(I2C_ADDR) # Configure BSC as I2C slave


def i2c_close():
    global pi
    global e

    e.cancel()

    pi.bsc_i2c(0) # Disable BSC peripheral

    print("i2c closed")

    pi.stop()
    exit()

