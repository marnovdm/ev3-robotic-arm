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
import csv
import os
import sys
import time
from signal import SIGINT, signal

# import evdev
import rpyc
from ev3dev2 import DeviceNotFound
from ev3dev2.led import Leds
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor
from ev3dev2.power import PowerSupply
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
# from evdev import InputDevice

# from math_helper import scale_stick
from smart_motor import (ColorSensorMotor, LimitedRangeMotor, StaticRangeMotor,
                         TouchSensorMotor, TouchSensorMotorSet)

# Config
REMOTE_HOST = '10.42.0.3'
JOYSTICK_DEADZONE = 20

# Define speeds
FULL_SPEED = 100
FAST_SPEED = 75
NORMAL_SPEED = 50
SLOW_SPEED = 25
VERY_SLOW_SPEED = 10

# Setup logging
os.system('setfont Lat7-Terminus12x6')
logging.basicConfig(level=logging.DEBUG, stream=sys.stdout,
                    format='%(asctime)s %(levelname)-8s %(message)s')
logger = logging.getLogger(__name__)


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


# Initial setup

# RPyC
# Setup on slave EV3: https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/rpyc.html
# Create a RPyC connection to the remote ev3dev device.
# Use the hostname or IP address of the ev3dev device.
# If this fails, verify your IP connectivty via ``ping X.X.X.X``
logger.info("Connecting RPyC to {}...".format(REMOTE_HOST))
# change this IP address for your slave EV3 brick
conn = rpyc.classic.connect(REMOTE_HOST)
# remote_ev3 = conn.modules['ev3dev.ev3']
remote_power_mod = conn.modules['ev3dev2.power']
remote_motor = conn.modules['ev3dev2.motor']
remote_led = conn.modules['ev3dev2.led']
remote_sensor = conn.modules['ev3dev2.sensor']
remote_sensor_lego = conn.modules['ev3dev2.sensor.lego']
logger.info("RPyC started succesfully")

# LEDs
leds = Leds()
remote_leds = remote_led.Leds()


def set_led_colors(color):
    leds.set_color("LEFT", "BLACK")
    leds.set_color("RIGHT", "BLACK")
    remote_leds.set_color("LEFT", "BLACK")
    remote_leds.set_color("RIGHT", "BLACK")
    # sound.play_song((('C4', 'e'), ('D4', 'e'), ('E5', 'q')))
    leds.set_color("LEFT", color)
    leds.set_color("RIGHT", color)
    remote_leds.set_color("LEFT", color)
    remote_leds.set_color("RIGHT", color)


set_led_colors("YELLOW")

# Power
power = PowerSupply(name_pattern='*ev3*')
remote_power = remote_power_mod.PowerSupply(name_pattern='*ev3*')

# Sound
# sound = Sound()

# Primary EV3
# Sensors
# try:
color_sensor = ColorSensor(INPUT_1)
color_sensor.mode = ColorSensor.MODE_COL_COLOR
logger.info("Color sensor detected!")

shoulder_touch = TouchSensor(INPUT_3)
logger.info("Shoulder touch sensor detected!")

elbow_touch = TouchSensor(INPUT_4)
logger.info("Elbow touch sensor detected!")

roll_touch = remote_sensor_lego.TouchSensor(remote_sensor.INPUT_1)
logger.info("Roll touch sensor detected!")


def motors_to_center():
    """ move all motors to their default position """

    shoulder_motors.on_to_position(
        SLOW_SPEED, shoulder_motors.center_position, True, True)
    elbow_motor.on_to_position(SLOW_SPEED, elbow_motor.center_position, True, True)

    roll_motor.on_to_position(NORMAL_SPEED, roll_motor.center_position, True, False)
    pitch_motor.on_to_position(NORMAL_SPEED, 0, True, False)
    spin_motor.on_to_position(NORMAL_SPEED, spin_motor.center_position, True, False)

    if grabber_motor:
        grabber_motor.on_to_position(
            NORMAL_SPEED, grabber_motor.center_position, True, True)

    waist_motor.on_to_position(FAST_SPEED, waist_motor.center_position, True, True)


# Motors
waist_motor = ColorSensorMotor(LargeMotor(
    OUTPUT_A), speed=NORMAL_SPEED, name='waist', sensor=color_sensor, color=5)  # 5 = red
shoulder_motors = TouchSensorMotorSet(
    [LargeMotor(OUTPUT_B), LargeMotor(OUTPUT_C)], speed=30, name='shoulder', sensor=shoulder_touch, max_position=-1000)
elbow_motor = TouchSensorMotor(LargeMotor(OUTPUT_D), speed=30, name='elbow', sensor=elbow_touch, max_position=-800)

# Secondary EV3
# Motors
# roll_motor = LimitedRangeMotor(remote_motor.MediumMotor(
#     remote_motor.OUTPUT_A), speed=30, name='roll')
roll_motor = TouchSensorMotor(remote_motor.MediumMotor(
    remote_motor.OUTPUT_A), speed=30, name='roll', sensor=roll_touch, max_position=-1000)
pitch_motor = LimitedRangeMotor(remote_motor.MediumMotor(
    remote_motor.OUTPUT_B), speed=10, name='pitch')
pitch_motor.stop_action = remote_motor.MediumMotor.STOP_ACTION_COAST
spin_motor = StaticRangeMotor(remote_motor.MediumMotor(
   remote_motor.OUTPUT_C), max_position=14 * 360, speed=20, name='spin')
# spin_motor = LimitedRangeMotor(remote_motor.MediumMotor(
#    remote_motor.OUTPUT_C), speed=20, name='spin')
# spin_motor = TouchSensorMotor(remote_motor.MediumMotor(
#     remote_motor.OUTPUT_C), speed=20, name='spin', sensor=spin_touch, max_position=500)

try:
    grabber_motor = LimitedRangeMotor(
        remote_motor.MediumMotor(remote_motor.OUTPUT_D), speed=10, name='grabber')
    grabber_motor.stop_action = remote_motor.MediumMotor.STOP_ACTION_COAST
    logger.info("Grabber motor detected!")
except DeviceNotFound:
    logger.info("Grabber motor not detected - running without it...")
    grabber_motor = False


def calibrate_motors():
    set_led_colors("ORANGE")
    logger.info('Calibrating motors...')

    # Note that the order here matters. We want to ensure the shoulder is calibrated first so the elbow can
    # reach it's full range without hitting the floor.
    shoulder_motors.calibrate()
    logger.debug(shoulder_motors)

    elbow_motor.calibrate(to_center=False)
    logger.debug(elbow_motor)

    # roll first, because otherwise we might not be able to move elbow all the way
    roll_motor.calibrate(timeout=5000)
    logger.debug(roll_motor)

    # shoulder_motors.to_position(50, wait=False)
    elbow_motor.to_position(50)

    # The waist motor has to be calibrated after calibrating the shoulder/elbow parts to ensure we're not
    # moving around with fully extended arm (which the waist motor gearing doesn't like)
    waist_motor.calibrate()
    logger.debug(waist_motor)

    # not strong enough yet :(
    pitch_motor.calibrate(timeout=5000)  # needs to be more robust, gear slips now instead of stalling the motor
    logger.debug(pitch_motor)

    if grabber_motor:
        grabber_motor.calibrate(to_center=False, timeout=5000)
        logger.debug(grabber_motor)

    # roll & spin motor are still missing here - spin motor can move indefinitely though

    set_led_colors("AMBER")


# Not sure why but resetting all motors before doing anything else seems to improve reliability
reset_motors()


def log_power_info():
    logger.info('Local battery power: {}V / {}A'.format(round(power.measured_volts, 2), round(power.measured_amps, 2)))
    logger.info('Remote battery power: {}V / {}A'.format(round(remote_power.measured_volts, 2), round(remote_power.measured_amps, 2)))


def clean_shutdown(signal_received=None, frame=None):
    """ make sure all motors are stopped when stopping this script """
    logger.info('Shutting down...')

    # This seems to help?!
    reset_motors()

    logger.info('waist..')
    waist_motor.stop()
    logger.info('shoulder..')
    shoulder_motors.stop()
    logger.info('elbow..')
    elbow_motor.stop()
    logger.info('roll..')
    roll_motor.reset()
    roll_motor.stop()
    logger.info('spin..')
    spin_motor.stop()
    logger.info('pitch..')
    # For some reason the pitch motor sometimes gets stuck here, and a reset helps?
    pitch_motor.reset()
    pitch_motor.stop()

    if grabber_motor:
        logger.info('grabber..')
        grabber_motor.stop()

    # See https://github.com/gvalkov/python-evdev/issues/19 if this raises exceptions, but it seems
    # stable now.
    # if gamepad:
    #     gamepad.close()

    leds.reset()
    remote_leds.reset()
    logger.info('Shutdown completed.')
    sys.exit(0)


# Ensure clean shutdown on CTRL+C
signal(SIGINT, clean_shutdown)
log_power_info()
calibrate_motors()

# contains only calibrated motors for now
motors = {
    'waist': waist_motor,
    'shoulder': shoulder_motors,
    'elbow': elbow_motor,
    'roll': roll_motor,
    # 'pitch': pitch_motor,
    # 'spin': spin_motor,
    # 'grabber': grabber_motor,
}


def moves_from_file(command_file):
    logger.info('Reading moves from {}...'.format(command_file))
    with open(command_file, newline='') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:

            for motor, position in row.items():
                if position is None:
                    logger.debug('Skip invalid position {} for motor {}'.format(position, motor))
                    continue

                try:
                    int(position)
                except ValueError:
                    logger.debug('Skip invalid row, got position {} for motor {}'.format(position, motor))
                    break

                if motor in motors:
                    motors[motor].to_position(int(position), wait=False)

            # testing...
            # time.sleep(1)
            # for motor in motors:
            #     print('Motor {} is running: {}'.format(motor._name, motor.is_running))
            while any((motor.is_running for name, motor in motors.items())):
                for name, motor in motors.items():
                    if motor.is_running:
                        logger.info('Waiting for {}...'.format(name))
                time.sleep(0.5)

            # while any((motor.is_running for name, motor in motors.items())):
            #     time.sleep(0.5)


moves_from_file('commands.csv')

# moves_from_file('waist.csv')
clean_shutdown()
