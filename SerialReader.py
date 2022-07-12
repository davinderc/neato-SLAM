#!/usr/bin/python
import serial


class SerialReader:
    def __init__(self):
        self.port = "/dev/ttyUSB0"
        self.baudrate = 115200
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.bytesize = serial.EIGHTBITS
        self.timeout = 0
        self.measurementsStarted = False
        self.reader = serial.Serial(port=self.port,
                                    baudrate=self.baudrate,
                                    parity=self.parity,
                                    stopbits=self.stopbits,
                                    bytesize=self.bytesize,
                                    timeout=self.timeout)

    def readbytes(self):
        return self.reader.read(1)

    def readbytes(self, numberOfBytes):
        return self.reader.read(numberOfBytes)
