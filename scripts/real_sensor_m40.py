import time
import serial

ser = serial.Serial('COM6', 19200, timeout=0.050)
count = 0


while 1:
    
    input()

    print(ser.readline())
    # time.sleep(1)
    # count += 1