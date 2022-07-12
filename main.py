#!/usr/bin/python
import serial
import math
import numpy as np
import cv2
import platform

from LidarDataReader import LidarDataReader
from SerialReader import SerialReader

# Some settings and variables

outfile = open("outfile.txt", "w+")
print("Start")
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
    img = np.zeros((1506, 1506, 3), dtype=np.uint8)

    for angle in range(0,360):
        x = int(img.shape[1] / 2 + (int(-measurements[angle] * math.sin(math.radians(angle))) / (6000 / img.shape[0])))
        y = int(img.shape[0] / 2 - (int(measurements[angle] * math.cos(math.radians(angle))) / (6000 / img.shape[1])))
        img[y, x] = [255, 255, 255]

    cv2.imshow("meas", img)
    #cv2.waitKey(1)



def decodeSingleLine(dataline):
    data = []

    for byte in dataline.strip("\n").split(":")[:21]:
        if byte != "":
            data.append(int(byte, 16))

    start = data[0]
    idx = data[1] - 0xa0
    speed = float(data[2] | (data[3] << 8)) / 64.0
    in_checksum = data[-2] + (data[-1] << 8)

    # first data package (4 bytes after header)
    angle = idx * 4 + 0
    angle_rad = angle * math.pi / 180.
    dist_mm = data[4] | ((data[5] & 0x1f) << 8)
    quality = data[6] | (data[7] << 8)

    if data[5] & 0x80:
        #pass
        print("X - ",)
    else:
        #pass
        print("O - ",)
    if data[5] & 0x40:
        #pass
        print("NOT GOOD")
    print("Speed: ", speed, ", angle: ", angle, ", dist: ", dist_mm, ", quality: ", quality)
    # print "Checksum: ", checksum(data), ", from packet: ", in_checksum
    outfile.write(dataline + "\n")
    #rint("-----------")
    global rotationCounter
    rotationCounter += 1
    print(f"####### rotations: {rotationCounter}")
    if (not (angle > 359 or angle < 0)):
        measurements[angle] = min(5999, int(dist_mm))

    if rotationCounter == 10:
        rotationCounter = 0
        updatePlot(measurements)


numberBytesToRead = 1
singleByte = serialStream.read(numberBytesToRead)
started = False
dataline = "Start"
STARTBYTE = "fa:"
EMPTYBYTE = b''
serialReader = SerialReader()
lidarDataReader = LidarDataReader()

while True:
    dataLine = serialReader.readbytes(22)
    parsedDataLine = LidarDataReader.parseDataLine(dataLine)
    data, dataUnit = LidarDataReader.decodeLidarDataLine(parsedDataLine)

    if dataLine[0] != EMPTYBYTE:
        #print(byte)
        encodedFirstByte = (dataLine[0].hex() + ":")
        if encodedFirstByte == STARTBYTE:
            if started:
                try:
                    decodeSingleLine(dataline)
                except Exception as e :
                    print(e)

            started = True
            dataline = encodedFirstByte
        elif started:
            dataline += encodedFirstByte
        else:
            print("Waiting for start")

    singleByte = serialStream.read(1)
outfile.close()
print("End")