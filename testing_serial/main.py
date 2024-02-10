import serial
import time

# Use the correct Arduino Port on Raspberry Pi
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()
print("Serial Up")

# Testing Biderectional Serial between Raspberry Pi and Arduino
try:
	while True:
		time.sleep(1)
		print("[INFO] Sending Messages to Subscriber")
		ser.write("Hello there Micro\n".encode('utf-8'))
		while ser.in_waiting <= 0:
			time.sleep(0.01)
		resp = ser.readline().decode('utf-8').rstrip()
		print(resp)
except:
	print("[INFO] Closing Serial Communication.")
	ser.close()