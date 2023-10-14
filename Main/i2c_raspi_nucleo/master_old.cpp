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
    while (i<12) {
        
        status = i2c.write(addr8bit, "y", 1, 0);
        if(status==0){
            pc.printf ("\nsent : ");
            pc.printf("y");
            myled = !myled;
            
            while (i<3){
                status = i2c.read(addr8bit, temp, 1, 0);
                if(status==0){
                    pc.printf ("\nreceived : ");
                    pc.printf("%s", temp);
                    x[i%3]=temp[0];
                    i++;
                }
            }
            while (i<6){
                status = i2c.read(addr8bit, temp, 1, 0);
                if(status==0){
                    pc.printf ("\nreceived : ");
                    pc.printf("%s", temp);
                    y[i%3]=temp[0];
                    i++;
                }
            }
            while (i<9){
                status = i2c.read(addr8bit, temp, 1, 0);
                if(status==0){
                    pc.printf ("\nreceived : ");
                    pc.printf("%s", temp);
                    a[i%3]=temp[0];
                    i++;
                }
            }
            while (i<12){
                status = i2c.read(addr8bit, temp, 1, 0);
                if(status==0){
                    pc.printf ("\nreceived : ");
                    pc.printf("%s", temp);
                    b[i%3]=temp[0];
                    i++;
                }
            }
    
            
            int k_x = atoi(x);
            int k_y = atoi(y);
            int o_a = atoi(a);
            int o_b = atoi(b);
            
            pc.printf("\n%d", k_x+1);
            pc.printf("\n%d", k_y+1);
            pc.printf("\n%d", o_a+1);
            pc.printf("\n%d", o_b+1);
            
        }  
    }
    return 0;
}