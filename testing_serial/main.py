import serial
import time
import numpy as np


# Use the correct Arduino Port on Raspberry Pi
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()
print("Serial Up")

#Inisialisasi motor speed di array
MotorSpeed = []

# Testing Biderectional Serial between Raspberry Pi and Arduino
try:
	while True:
		time.sleep(1)
		print("[INFO] Sending Messages to Subscriber")
		ser.write("Hello there Micro\n".encode('utf-8'))
		while ser.in_waiting <= 0:
			time.sleep(0.01)
			resp = ser.readline().decode('utf-8').rstrip()
			if resp:
				print(f"Motor speed: {resp}")
				MotorSpeed.append(resp)
except KeyboardInterrupt:
	print("[INFO] Closing Serial Communication.")
	ser.close()

#Array NumPy
MotorArray = np.array(MotorSpeed, dtype=float)
