#!/usr/bin/python
import serial
import math
import numpy as np
import cv2
import platform

# Some settings and variables
outfile = open("outfile.txt", "w+")
print("Start")
invalids = 0
valids = 0
weaks = 0
indexOffset = 0xa0
invalidDataBitMask = 0x80
inferiorStrengthDataBitMask = 0x40
rotationCounter = 0
measurements = np.zeros((360, 1), np.float64)
if(platform.system() == 'Windows'):
    serialPort = 'COM4'
elif(platform.system() == 'Linux'):
    serialPort = '/dev/ttyUSB0'
else:
    serialPort = 'some Mac address'
print(serialPort)
serialStream = serial.Serial(port=serialPort,
                             baudrate=115200,
                             parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE,
                             bytesize=serial.EIGHTBITS,
                             timeout=0)

def updatePlot(measurements):
    #print("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")
    img = np.zeros((500, 500, 3), dtype=np.uint8)

    for angle in range(0,360):
        x = int(img.shape[1] / 2 + (int(-measurements[angle] * math.sin(math.radians(angle))) / (6000 / img.shape[0])))
        y = int(img.shape[0] / 2 - (int(measurements[angle] * math.cos(math.radians(angle))) / (6000 / img.shape[1])))
        img[y, x] = [255, 255, 255]

    cv2.imshow("meas", img)
    cv2.waitKey(1)



def decodeSingleLine(dataline):
    global invalids
    global valids
    global weaks
    data = []

    for byte in dataline.strip("\n").split(":")[:21]:
        if byte != "":
            data.append(int(byte, 16))

    startByte = data[0]
    indexByte = data[1] - indexOffset
    speed = float(data[2] | (data[3] << 8)) / 64.0
    in_checksum = data[-2] | (data[-1] << 8)

    # first data package (4 bytes after header)
    angle = indexByte * 4 + 0
    angle_rad = angle * math.pi / 180.
    # Next line used 0x1f to get first 5 bits of data[5], but first 6 bits should have been used so 0x3f
    # distance_mm = data[4] | ((data[5] & 0x1f) << 8)
    distance_mm = data[4] | ((data[5] & 0x3f) << 8)
    signalStrength = data[6] | (data[7] << 8)

    if data[5] & invalidDataBitMask:
        invalids += 1
        #pass
        print("X - ", data[5])
    else:
        valids += 1
        #pass
        print("O - ",)
    if data[5] & inferiorStrengthDataBitMask:
        #pass
        print("inferior signal strength")
        weaks += 1
    print("Speed: ", speed, ", angle: ", angle, ", dist: ", distance_mm, ", Signal Strength: ", signalStrength)
    # print "Checksum: ", checksum(data), ", from packet: ", in_checksum
    outfile.write(dataline + "\n")
    # print("-----------")
    global rotationCounter
    rotationCounter += 1
    # print(f"####### rotations: {rotationCounter}")
    if (not (angle > 359 or angle < 0)):
        measurements[angle] = min(5999, int(distance_mm))

    if rotationCounter == 10:
        rotationCounter = 0
        updatePlot(measurements)
        print(f"valids {valids}, invalids {invalids}, weaks {weaks}")


numberBytesToRead = 1
singleByte = serialStream.read(numberBytesToRead)
started = False
dataline = "Start"
STARTBYTE = "fa:"
EMPTYBYTE = b''
while True:
    if singleByte != EMPTYBYTE:
        enc = (singleByte.hex() + ":")
        if enc == STARTBYTE:
            if started:
                try:
                    decodeSingleLine(dataline)
                except Exception as e :
                    print(e)

            started = True
            dataline = enc
        elif started:
            dataline += enc
        else:
            print("Waiting for start")

    singleByte = serialStream.read(1)
outfile.close()
print("End")