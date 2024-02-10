# Biderectional Serial Testing
## Between Raspberry Pi and Arudino

Hardware: 
- Raspberry Pi Model 4B 8GB RAM with Ubuntu Server 22.04
- Arduino Mega 2560

Software:
- Python 3.10
- PySerial 3.5

Expected Result: 
After 3 Seconds Starting the main.py program, check on Raspi terminal

> Serial Up  
[INFO] Sending Messages to Subscriber  
Hello there Micro 0  
[INFO] Sending Messages to Subscriber  
Hello there Micro 1

The number will increment until you give KeyboardInterrupt such as **CTRL+C**.  
**Note:** Make sure any other app is not using the serial other than the python program.