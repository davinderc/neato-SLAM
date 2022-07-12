#!/usr/bin/python
import math

from LidarDataUnit import LidarDataUnit


class LidarDataReader:
    def __init__(self):
        self.lineStart = "fa"
        self.separator = ":"

    def parseDataLine(self, dataLine):
        parsedData = []
        for byte in dataLine.strip("\n").split(self.separator)[:21]:
            if byte != "":
                parsedData.append(int(byte, 16))
        return parsedData

    def decodeLidarDataLine(self, dataLine):
        print(dataLine)

        parsedLine = self.parseDataLine(dataLine)

        start = parsedLine[0]
        idx = parsedLine[1] - 0xa0
        speed = float(parsedLine[2] | (parsedLine[3] << 8)) / 64.0
        in_checksum = parsedLine[-2] + (parsedLine[-1] << 8)

        # first data package (4 bytes after header)
        angle = idx * 4 + 0
        angle_rad = angle * math.pi / 180.
        dist_mm = parsedLine[4] | ((parsedLine[5] & 0x1f) << 8)
        quality = parsedLine[6] | (parsedLine[7] << 8)

        lidarDataUnit = LidarDataUnit(angle, dist_mm, quality)

        return parsedLine, lidarDataUnit

        # if parsedLine[5] & 0x80:
        #     print("X - ", )
        # else:
        #     print("O - ", )
        # if parsedLine[5] & 0x40:
        #     print("NOT GOOD")
        # # print("Speed: ", speed, ", angle: ", angle, ", dist: ", dist_mm, ", quality: ", quality)
        # # print "Checksum: ", checksum(data), ", from packet: ", in_checksum
        # outfile.write(dataLine + "\n")
        # print("-----------")
