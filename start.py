# -*- coding: utf-8 -*-

import json
import os
import sys
import termios
import time
import sched
from threading import Timer
import atexit

# sys.argv[0] ... Serial Port
# sys.argv[1] ... How long in seconds the sampling is performed, 10 seconds by default (optional)

# SerialPort class was imported from John Wiseman's https://github.com/wiseman/arduino-serial/blob/master/arduinoserial.py

# Map from the numbers to the termios constants (which are pretty much
# the same numbers).

BPS_SYMS = {
    4800:   termios.B4800,
    9600:   termios.B9600,
    19200:  termios.B19200,
    38400:  termios.B38400,
    57600:  termios.B57600,
    115200: termios.B115200
}


# Indices into the termios tuple.

IFLAG = 0
OFLAG = 1
CFLAG = 2
LFLAG = 3
ISPEED = 4
OSPEED = 5
CC = 6


def bps_to_termios_sym(bps):
    return BPS_SYMS[bps]

class SerialPort:

    def __init__(self, serialport, bps):
        """Takes the string name of the serial port
        (e.g. "/dev/tty.usbserial","COM1") and a baud rate (bps) and
        connects to that port at that speed and 8N1. Opens the port in
        fully raw mode so you can send binary data.
        """
        self.bps = bps
        self.fd = os.open(serialport, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
        attrs = termios.tcgetattr(self.fd)
        bps_sym = bps_to_termios_sym(bps)
        # Set I/O speed.
        attrs[ISPEED] = bps_sym
        attrs[OSPEED] = bps_sym

        # 8N1
        attrs[CFLAG] &= ~termios.PARENB
        attrs[CFLAG] &= ~termios.CSTOPB
        attrs[CFLAG] &= ~termios.CSIZE
        attrs[CFLAG] |= termios.CS8
        # No flow control
        attrs[CFLAG] &= ~termios.CRTSCTS

        # Turn on READ & ignore contrll lines.
        attrs[CFLAG] |= termios.CREAD | termios.CLOCAL
        # Turn off software flow control.
        attrs[IFLAG] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)

        # Make raw.
        attrs[LFLAG] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
        attrs[OFLAG] &= ~termios.OPOST

        # It's complicated--See
        # http://unixwiz.net/techtips/termios-vmin-vtime.html
        attrs[CC][termios.VMIN] = 0;
        attrs[CC][termios.VTIME] = 20;
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def read_async(self):
        n = os.read(self.fd, 100)
        if n != '':
            return bytearray(n)
        return None;

    def read_until(self, until):
        buf = ""
        done = False
        while not done:
            n = os.read(self.fd, 1)
            if n == '':
                # FIXME: Maybe worth blocking instead of busy-looping?
                time.sleep(0.01)
                continue
            buf = buf + n
            if n == until:
                done = True
        return buf

    def read_line(self):
        try:
            return self.read_until("\n").strip()
        except OSError:
            return None

    def write(self, str):
        os.write(self.fd, str)

    def write_byte(self, byte):
        os.write(self.fd, chr(byte))

class Sampler:

    def __init__(self, serial, inserval=1):
        self.serial = serial
        self.interval = 1
        self.count = 0
        self.duration = 0
        self.sum = 0
        self.running = False

    def start(self, expected_duration):
        self.running = True
        self.expected_duration = expected_duration
        def print_count():
            c = self.count
            self.count = 0
            print("%d : %s bytes/sec" % (time.time(), c))
            self.duration = self.duration + 1
            self.sum = self.sum + c
            if self.duration >= self.expected_duration:
                self.running = False
            if self.running:
                Timer(self.interval, print_count, ()).start()

        print('Enter Ctrl+C twice to exit!')
        Timer(self.interval, print_count, ()).start()
        while self.running:
            b = self.serial.read_async()
            if b != None:
                self.count = self.count + len(b)
            time.sleep(0.01)
        self.dump()

    def dump(self):
        time.sleep(1)
        print('========================================')
        print('Total Time in Seconds => %d' % self.duration)
        print('Total Received Bytes => %d' % self.sum)
        print('Mean Received Bytes Per Second => %.2f' % (float(self.sum) / self.duration))
        print('Baud Rate in Bit Per Second => %d' % self.serial.bps)
        print('========================================')

def main(serial_port, expected_duration):
    serial = SerialPort(serial_port, 115200)
    sampler = Sampler(serial)
    sampler.start(expected_duration)

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print("Set the serial port")
    else:
        if len(sys.argv) > 2:
            expected_duration = int(sys.argv[2])
        else:
            expected_duration = 10
        main(sys.argv[1], expected_duration)
