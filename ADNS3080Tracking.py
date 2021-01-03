#!/usr/bin/env python
#
# Copyright DIY Drone:
# https://github.com/diydrones/ardupilot/tree/5ddbcc296dd6dd9ac9ed6316ac3134c736ae8a78/libraries/AP_OpticalFlow
# Modified by Kristian Sloth Lauszus

# Modified on July 2019 by Jungwon Hwang jhwang59@rrc.ca
# Reading the raw data [x,dX y,dY Surface_Quality] from serial port

from serial import Serial, SerialException


class OpticalFlow:
    ser = None

    def __init__(self):
        self.open()

    def close(self):
        self.close_serial()

    def open(self):
        # Close the serial port
        self.close_serial()

        # Open the serial port
        try:
            self.ser = Serial(port='/dev/ttyUSB0', baudrate='115200', timeout=.1) 
            self.read_loop()  # Read from serial port
        except SerialException:
            print("Failed to open serial port")

    def close_serial(self):
        if self.ser and self.ser.isOpen():
            try:
                self.ser.close()
                print("Closed serial port")
            except SerialException:
                pass  # Do nothing

    def read_loop(self):
        if self.ser.isOpen():
            try:
                self.read_from_serial()
            except (IOError, TypeError):
                pass

    def read_from_serial(self):
        lines = [0, 0, 0, 0, 0]
        while self.ser and self.ser.isOpen() and self.ser.inWaiting() > 0:
            for counter in range(0,5):
                # Process the line read
                line = int(self.ser.readline())
                lines[counter] = line
            return lines
