#include "mbed.h"

I2C i2c(PB_9, PB_8);
Serial pc(USBTX, USBRX, 9600);
DigitalOut myled(LED1);

const int addr8bit = 0x48 << 1; // 8bit I2C address, 7bit i2c address 0x48

int main()
{
    char x[3];
    char y[3];
    char a[3];
    char b[3];
    char temp[1];
    int i =0;
    int status = 0;
    int k_x;
    int k_y;
    int o_a;
    int o_b;
    while (i<12) {
        
        status = i2c.write(addr8bit, "y", 1, 0);
        if(status==0){
            pc.printf ("\nsent : ");
            pc.printf("y");
            myled = !myled;
            
            while (i<12){
                status = i2c.read(addr8bit, temp, 1, 0);
                if(status==0){
                    pc.printf ("\nreceived : ");
                    pc.printf("%s", temp);
                    if(i<3){
                        x[i%3]=temp[0];
                        i++;
                    }
                    else if (i<6){
                        y[i%3]=temp[0];
                        i++;
                    }
                    else if (i<9){
                        a[i%3]=temp[0];
                        i++;
                    }
                    else if (i<12){
                        b[i%3]=temp[0];
                        i++;
                    }
                }
            }
            
            k_x = atoi(x);
            k_y = atoi(y);
            o_a = atoi(a);
            o_b = atoi(b);            
        }  
    }
    return 0;
}