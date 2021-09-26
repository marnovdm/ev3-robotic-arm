#!/usr/bin/env python3
# ev3-robot-arm 6dof, originally by Nino Guba.
# v2 improved by Marno van der Molen;
# - bugfixes
# - don't require grabber attachment to run
# - more debug output for troubleshooting
# - improved gamepad responsiveness
# - proportional control for some motors
# - code cleanup / simplify
# v2.1 refinements by Marno van der Molen:
# - simlify code
# - allow changing speed of movement by holding d-pad up/down
# - optionally support a color sensor to align waist by pressing d-pad left/right
# v2.2 minor improvements by Marno van der Molen
# - maintain grabber grip during spin
# - increase joystick deadzone a bit to prevent unintended movement while pressing L3/R3
# - start work on calibration support using touch sensors
# - prevent calculate_speed() returning values over 100 which causes exceptions

__author__ = 'Nino Guba'

import logging
import os
import sys
import threading
import time

import evdev
import rpyc
from signal import signal, SIGINT
from ev3dev2 import DeviceNotFound
from ev3dev2.led import Leds
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor, MoveTank
from ev3dev2.power import PowerSupply

from evdev import InputDevice

from math_helper import scale_stick
from motor_thread import MotorThread
from waist_align_thread import WaistAlignThread

# Config
REMOTE_HOST = '10.42.0.3'
JOYSTICK_DEADZONE = 20

# Setup logging
os.system('setfont Lat7-Terminus12x6')
logging.basicConfig(level=logging.INFO, stream=sys.stdout,
                    format='%(message)s')
logger = logging.getLogger(__name__)


# Initial setup

# RPyC
# Setup on slave EV3: https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/rpyc.html
# Create a RPyC connection to the remote ev3dev device.
# Use the hostname or IP address of the ev3dev device.
# If this fails, verify your IP connectivty via ``ping X.X.X.X``
logger.info("Connecting RPyC to {}...".format(REMOTE_HOST))
# change this IP address for your slave EV3 brick
conn = rpyc.classic.connect(REMOTE_HOST)
remote_power_mod = conn.modules['ev3dev2.power']
remote_motor = conn.modules['ev3dev2.motor']
remote_led = conn.modules['ev3dev2.led']
logger.info("RPyC started succesfully")

# Gamepad
# If bluetooth is not available, check https://github.com/ev3dev/ev3dev/issues/1314
logger.info("Connecting wireless controller...")
gamepad = InputDevice(evdev.list_devices()[0])
if gamepad.name != 'Wireless Controller':
    logger.error('Failed to connect to wireless controller')
    sys.exit(1)

# LEDs
leds = Leds()
remote_leds = remote_led.Leds()

# Power
power = PowerSupply(name_pattern='*ev3*')
remote_power = remote_power_mod.PowerSupply(name_pattern='*ev3*')

# Primary EV3
# Sensors
try:
    color_sensor = ColorSensor(INPUT_1)
    color_sensor.mode = ColorSensor.MODE_COL_COLOR
    logger.info("Color sensor detected!")
except DeviceNotFound:
    logger.info("Color sensor not detected (primary EV3, input 1) - running without it...")
    color_sensor = False

try:
    shoulder_touch = TouchSensor(INPUT_3)
    logger.info("Shoulder touch sensor detected!")
except DeviceNotFound:
    logger.info("Shoulder touch sensor not detected (primary EV3, input 3) - running without it...")
    shoulder_touch = False

try:
    elbow_touch = TouchSensor(INPUT_4)
    logger.info("Elbow touch sensor detected!")
except DeviceNotFound:
    logger.info("Elbow touch sensor not detected (primary EV3, input 4) - running without it...")
    elbow_touch = False

# Motors
waist_motor = LargeMotor(OUTPUT_A)
shoulder_motors = MoveTank(OUTPUT_B, OUTPUT_C)
elbow_motor = LargeMotor(OUTPUT_D)

# Secondary EV3
# Motors
roll_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_A)
pitch_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_B)
pitch_motor.stop_action = remote_motor.MediumMotor.STOP_ACTION_COAST
spin_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_C)

try:
    grabber_motor = remote_motor.MediumMotor(remote_motor.OUTPUT_D)
    grabber_motor.stop_action = remote_motor.MediumMotor.STOP_ACTION_COAST
    logger.info("Grabber motor detected!")
except DeviceNotFound:
    logger.info("Grabber motor not detected (secondary EV3, port D) - running without it...")
    grabber_motor = False


devices = {
    'color_sensor': color_sensor, 
    'shoulder_touch': shoulder_touch, 
    'elbow_touch': elbow_touch, 
    'waist_motor': waist_motor, 
    'shoulder_motors': shoulder_motors, 
    'elbow_motor': elbow_motor, 
    'roll_motor': roll_motor, 
    'pitch_motor': pitch_motor, 
    'spin_motor': spin_motor, 
    'grabber_motor': grabber_motor,
    'leds': leds,
    'remote_leds': remote_leds,
}

state = {
    'shoulder_speed': 0, 
    'elbow_speed': 0, 
    'waist_left': False, 
    'waist_right': False, 
    'roll_left': False, 
    'roll_right': False, 
    'pitch_up': False, 
    'pitch_down': False, 
    'spin_left': False, 
    'spin_right': False, 
    'grabber_open': False, 
    'grabber_close': False, 
    'speed_modifier': 0, 
    'running': True, 
    'waist_target_color': 0, 
    'aligning_waist': False
}


def reset_motors():
    """ reset motor positions to default """
    logger.info("Resetting motors...")
    waist_motor.reset()
    shoulder_motors.reset()
    elbow_motor.reset()
    roll_motor.reset()
    pitch_motor.reset()
    spin_motor.reset()
    if grabber_motor:
        grabber_motor.reset()


def log_power_info():
    logger.info('Local battery power: {}V / {}A'.format(round(power.measured_volts, 2), round(power.measured_amps, 2)))
    logger.info('Remote battery power: {}V / {}A'.format(round(remote_power.measured_volts, 2), round(remote_power.measured_amps, 2)))


def clean_shutdown(signal_received=None, frame=None):
    """ make sure all motors are stopped when stopping this script """
    logger.info('Shutting down...')

    global state
    state['running'] = False

    logger.info('waist..')
    waist_motor.stop()
    logger.info('shoulder..')
    shoulder_motors.stop()
    logger.info('elbow..')
    elbow_motor.stop()
    logger.info('pitch..')
    # For some reason the pitch motor sometimes gets stuck here, and a reset helps?
    # pitch_motor.reset()
    pitch_motor.stop()
    logger.info('roll..')
    roll_motor.stop()
    logger.info('spin..')
    spin_motor.stop()

    if grabber_motor:
        logger.info('grabber..')
        grabber_motor.stop()

    # See https://github.com/gvalkov/python-evdev/issues/19 if this raises exceptions, but it seems
    # stable now.
    gamepad.close()

    logger.info('Shutdown completed.')
    sys.exit(0)


# Ensure clean shutdown on CTRL+C
signal(SIGINT, clean_shutdown)

# Not sure why but resetting all motors before doing anything else seems to improve reliability
reset_motors()

log_power_info()

# Main motor control thread
motor_thread = MotorThread(logger=logger, devices=devices, state=state)
motor_thread.setDaemon(True)
motor_thread.start()

# We only need the WaistAlignThread if we detected a color sensor
if color_sensor:
    waist_align_thread = WaistAlignThread(logger=logger, devices=devices, state=state)
    waist_align_thread.setDaemon(True)
    waist_align_thread.start()

# Handle gamepad input
for event in gamepad.read_loop():  # this loops infinitely
    if event.type == 3:  # stick input
        if event.code == 0:  # Left stick X-axis
            state['shoulder_speed'] = scale_stick(event.value, deadzone=JOYSTICK_DEADZONE, invert=True)
        elif event.code == 3:  # Right stick X-axis
            state['elbow_speed'] = scale_stick(event.value, deadzone=JOYSTICK_DEADZONE)
        elif event.code == 17:  # dpad up/down
            state['speed_modifier'] = event.value
        elif event.code == 16:  # dpad left/right
            state['waist_target_color'] = event.value

    elif event.type == 1:  # button input

        if event.code == 310:  # L1
            if event.value == 1:
                state['waist_right'] = False
                state['waist_left'] = True
            elif event.value == 0:
                state['waist_left'] = False

        elif event.code == 311:  # R1
            if event.value == 1:
                state['waist_left'] = False
                state['waist_right'] = True
            elif event.value == 0:
                state['waist_right'] = False

        elif event.code == 308:  # Square
            if event.value == 1:
                state['roll_right'] = False
                state['roll_left'] = True
            elif event.value == 0:
                state['roll_left'] = False

        elif event.code == 305:  # Circle
            if event.value == 1:
                state['roll_left'] = False
                state['roll_right'] = True
            elif event.value == 0:
                state['roll_right'] = False

        elif event.code == 307:  # Triangle
            if event.value == 1:
                state['pitch_down'] = False
                state['pitch_up'] = True
            elif event.value == 0:
                state['pitch_up'] = False

        elif event.code == 304:  # X
            if event.value == 1:
                state['pitch_up'] = False
                state['pitch_down'] = True
            elif event.value == 0:
                state['pitch_down'] = False

        elif event.code == 312:  # L2
            if event.value == 1:
                state['spin_right'] = False
                state['spin_left'] = True
            elif event.value == 0:
                state['spin_left'] = False

        elif event.code == 313:  # R2
            if event.value == 1:
                state['spin_left'] = False
                state['spin_right'] = True
            elif event.value == 0:
                state['spin_right'] = False

        elif event.code == 317:  # L3
            if event.value == 1:
                state['grabber_close'] = False
                state['grabber_open'] = True
            elif event.value == 0:
                state['grabber_open'] = False

        elif event.code == 318:  # R3
            if event.value == 1:
                state['grabber_open'] = False
                state['grabber_close'] = True
            elif event.value == 0:
                state['grabber_close'] = False

        elif event.code == 314 and event.value == 1:  # Share
            # debug info
            log_power_info()

        elif event.code == 315 and event.value == 1:  # Options
            # debug info
            logger.info('Elbow motor state: {}'.format(elbow_motor.state))
            logger.info('Elbow motor duty cycle: {}'.format(elbow_motor.duty_cycle))
            logger.info('Elbow motor speed: {}'.format(elbow_motor.speed))

        elif event.code == 316 and event.value == 1:  # PS
            # stop control loop
            state['running'] = False

            # Move motors to default position
            # motors_to_center()

            # sound.play_song((('E5', 'e'), ('C4', 'e')))
            leds.set_color("LEFT", "BLACK")
            leds.set_color("RIGHT", "BLACK")
            remote_leds.set_color("LEFT", "BLACK")
            remote_leds.set_color("RIGHT", "BLACK")

            time.sleep(1)  # Wait for the motor thread to finish
            break

clean_shutdown()
