#!/usr/bin/env python3
import logging
import os
import sys

# import evdev
import rpyc
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor


# Config
REMOTE_HOST = '10.42.0.3'

# Setup logging
os.system('setfont Lat7-Terminus12x6')
logging.basicConfig(level=logging.DEBUG, stream=sys.stdout,
                    format='%(asctime)s %(levelname)-8s %(message)s')
logger = logging.getLogger(__name__)


def reset_motors():
    """ reset motor positions to default """
    logger.info("Resetting motors...")
    waist_motor.reset()
    shoulder_motor1.reset()
    shoulder_motor2.reset()
    elbow_motor.reset()
    roll_motor.reset()
    pitch_motor.reset()
    spin_motor.reset()
    grabber_motor.reset()


# Initial setup

# RPyC
# Setup on slave EV3: https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/rpyc.html
# Create a RPyC connection to the remote ev3dev device.
# Use the hostname or IP address of the ev3dev device.
# If this fails, verify your IP connectivty via ``ping X.X.X.X``
logger.info("Connecting RPyC to {}...".format(REMOTE_HOST))
# change this IP address for your slave EV3 brick
conn = rpyc.classic.connect(REMOTE_HOST)
remote_motor = conn.modules['ev3dev2.motor']

# Primary EV3 - motors
waist_motor = LargeMotor(OUTPUT_A)
shoulder_motor1 = LargeMotor(OUTPUT_B)
shoulder_motor2 = LargeMotor(OUTPUT_C)
elbow_motor = LargeMotor(OUTPUT_D)

# Secondary EV3 - motors
roll_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_A)
pitch_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_B)
spin_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_C)
grabber_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_D)

reset_motors()
