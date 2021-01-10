# Copyright DIY Drone, ADNS3080ImageGrabber.py
# Modified by Jungwon Hwang (hjw0903@gmail.com) July, 2019
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
            pass  # Do nothing

    def close_serial(self):
        if self.ser and self.ser.isOpen():
            try:
                self.ser.close()
                print("Closed serial port")
            except SerialException:
                pass  

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
