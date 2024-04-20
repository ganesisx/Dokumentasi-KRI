import cv2 as cv
import numpy as np
import math
import serial
import time

# Initialize serial connection (adjust 'COM3' and 9600 to match your setup)
ser = serial.Serial('COM3', 9600, timeout=1)

# Start capturing video from the webcam
cap = cv.VideoCapture(1)  # Change to 0 if camera 1 is not available

if not cap.isOpened():
    print("Cannot open camera")
    exit()

lower_white = np.array([0, 0, 0], dtype=np.uint8)
upper_white = np.array([79, 245, 255], dtype=np.uint8)
lower_green = np.array([35, 50, 50], dtype=np.uint8)
upper_green = np.array([85, 255, 255], dtype=np.uint8)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    hsv_img = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    final_mask = cv.inRange(hsv_img, lower_white, upper_white)
    # maskg = cv.inRange(hsv_img, lower_green, upper_green)

    # final_mask = cv.bitwise_and(cv.bitwise_not(maskg), mask)
    # Apply dilation
    kernel = np.ones((7,7), np.uint8)  # You can adjust the kernel size
    mask_dilated = cv.dilate(final_mask, kernel, iterations=2)  # Increase iterations for more dilation
    mask_eroded = cv.erode(mask_dilated, kernel, iterations=2)  # Adjust iterations as needed



    contours, hierarchy = cv.findContours(mask_eroded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        c = max(contours, key=cv.contourArea)
        M = cv.moments(c)
        if M["m00"] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            print(f"CX : {cx}  CY : {cy}")

            if cy >= 290:
                print("Turn Left")
            elif 190 < cy < 290:
                print("On Track!")
            elif cy <= 190:
                print("Turn Righ    t")

            cv.circle(frame, (cx, cy), 5, (255, 255, 255), -1)  # Draw circle at contour center

    cv.drawContours(frame, contours, -1, (0, 255, 0), 1)
    cv.imshow("Mask", mask_eroded)
    cv.imshow("Frame", frame)

    if cv.waitKey(1) == ord('q'):  # Press 'q' to quit
        break

# When everything is done, release the capture
cap.release()
cv.destroyAllWindows()
