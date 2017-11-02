#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PyAX-12

# The MIT License
#
# Copyright (c) 2010,2015 Jeremie DECOCK (http://www.jdhp.org)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

''' 
install pyax-12 library from here: https://github.com/jeremiedecock/pyax12 
call this module via command line like python.exe .\move.py -p "COM16" -b 1000000 -i 254,
add -b 1000 (<- 1000 kbps = 1 000 000 bps), 
add -p "COM16" (Windows) or -p "/dev/..." (Linux),
add -i for the motor id (254 is the broadcast id),
this will move the adressed motors to position 30
'''

from pyax12.connection import Connection
from pyax12.argparse_default import common_argument_parser

import time

MAIN = 0
JIB = 1
RUDDER = 2

def set_position(motor_id, pos):
    """
	moves the motor with the given motor_id to the given position
    """

    # Parse options
    parser = common_argument_parser(desc=set_position.__doc__)
    args = parser.parse_args()

    # Connect to the serial port
    serial_connection = Connection(port=args.port,  # specified with -p
                                   baudrate=args.baudrate, # specified with -b
                                   timeout=args.timeout,
                                   rpi_gpio=args.rpi)

    dynamixel_id = args.dynamixel_id
    address = 0x12  # Activate the Alarm Shutdown
    data = 0x24  # when overheat or overload (AX-12)
    serial_connection.write_data(dynamixel_id, address, data)
    
    # Print the control table of the specified Dynamixel unit
    serial_connection.pretty_print_control_table(dynamixel_id)
    
    # move motor to given position
    serial_connection.goto(dynamixel_id, pos, speed=512, degrees=True)
    time.sleep(1)    # Wait 1 second

    # Close the serial connection
    serial_connection.close()

if __name__ == '__main__':
	for motor_id in range(0,2):
		pos = 30
		set_position(motor_id, pos)
