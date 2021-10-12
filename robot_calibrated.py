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
# v3 - major changes by Marno van der Molen
# - implement SmartMotor classes which abstracts the differences
# - support calibration using some touch/color sensors
# - perform moves from CSV files
__author__ = 'Nino Guba'

import logging
import csv
import os
import sys
import time
from signal import SIGINT, signal

# import evdev
import rpyc
from ev3dev2.led import Leds
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor
from ev3dev2.power import PowerSupply
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, TouchSensor

from smart_motor import (CalibrationError, ColorSensorMotor, LimitedRangeMotor, StaticRangeMotor,
                         TouchSensorMotor, TouchSensorMotorSet, MotorMoveError)

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


def set_led_colors(color, animate=False):
    """ helper to set LED color easily on both EV3 bricks """

    if animate:
        # stop any running animations
        leds.animate_stop()
        remote_leds.animate_stop()
        leds.animate_flash(color, groups=('LEFT', 'RIGHT'), sleeptime=0.5, duration=None, block=False)
    else:
        leds.set_color("LEFT", color)
        leds.set_color("RIGHT", color)
        remote_leds.set_color("LEFT", color)
        remote_leds.set_color("RIGHT", color)


# Start initialization of devices...
set_led_colors("YELLOW", animate=True)

# Power
power = PowerSupply(name_pattern='*ev3*')
remote_power = remote_power_mod.PowerSupply(name_pattern='*ev3*')

# Primary EV3 - sensors
waist_color_sensor = ColorSensor(INPUT_1)
waist_color_sensor.mode = ColorSensor.MODE_COL_COLOR
logger.info("Waist color sensor detected!")

front_color_sensor = ColorSensor(INPUT_2)
front_color_sensor.mode = ColorSensor.MODE_COL_COLOR
logger.info("Front color sensor detected!")

shoulder_touch = TouchSensor(INPUT_3)
logger.info("Shoulder touch sensor detected!")

elbow_touch = TouchSensor(INPUT_4)
logger.info("Elbow touch sensor detected!")

roll_touch = remote_sensor_lego.TouchSensor(remote_sensor.INPUT_1)
logger.info("Roll touch sensor detected!")

# Primary EV3 - motors
waist_motor = ColorSensorMotor(LargeMotor(
    OUTPUT_A), speed=NORMAL_SPEED, name='waist', sensor=waist_color_sensor, color=5)  # 5 = red
shoulder_motors = TouchSensorMotorSet(
    [LargeMotor(OUTPUT_B), LargeMotor(OUTPUT_C)], speed=30, name='shoulder', sensor=shoulder_touch, max_position=-1000)
elbow_motor = TouchSensorMotor(LargeMotor(OUTPUT_D), speed=30, name='elbow', sensor=elbow_touch, max_position=-750)

# Secondary EV3 - motors
roll_motor = TouchSensorMotor(remote_motor.MediumMotor(
    remote_motor.OUTPUT_A), speed=30, name='roll', sensor=roll_touch, max_position=-1000)
pitch_motor = LimitedRangeMotor(remote_motor.MediumMotor(
    remote_motor.OUTPUT_B), speed=20, name='pitch', max_position=-800)
pitch_motor.stop_action = remote_motor.MediumMotor.STOP_ACTION_COAST
spin_motor = ColorSensorMotor(remote_motor.MediumMotor(
    remote_motor.OUTPUT_C), name='spin', speed=25, sensor=front_color_sensor, color=5, max_position=2605)  # 5 = red
grabber_motor = LimitedRangeMotor(
   remote_motor.MediumMotor(remote_motor.OUTPUT_D), speed=80, name='grabber', max_position=-2000)
grabber_motor.stop_action = remote_motor.MediumMotor.STOP_ACTION_COAST


# contains only calibrated motors for now
motors = {
    'waist': waist_motor,
    'shoulder': shoulder_motors,
    'elbow': elbow_motor,
    'roll': roll_motor,
    'pitch': pitch_motor,
    # 'spin': spin_motor,
    'grab': grabber_motor,
}

def wait_for_motors(motor_wait_max=10, max_stalls=2):
    """ max wait in seconds """
    motor_wait_start = time.time()
    stalls = 0
    while any((motor.is_running for name, motor in motors.items())):
        for name, motor in motors.items():
            if motor.is_running and (motor.is_stalled or motor.is_overloaded):
                logger.error('Waiting for motor {}, currently at position {}, state {}'.format(name, motor.current_position, motor.state))
                stalls += 1
                if stalls >= max_stalls:
                    clean_shutdown()
                # raise MotorMoveError('stalled/overloaded while waiting for move to complete')
            elif motor.is_running:
                logger.info('Waiting for motor {}, currently at position {}, state {}'.format(name, motor.current_position, motor.state))

        if time.time() > (motor_wait_start + motor_wait_max):
            raise MotorMoveError('timed out while waiting for move to complete')

        time.sleep(0.25)  # used to be 0.5


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
    roll_motor.calibrate()
    logger.debug(roll_motor)

    # shoulder_motors.to_position(50, wait=False)
    elbow_motor.to_position(50)
    
    # grabber calibration before pitch, so we can move it to position 20 to prevent
    # blocking pitch motor.
    grabber_motor.calibrate(timeout=7000, to_center=False)
    logger.debug(grabber_motor)
    grabber_motor.to_position(20)

    pitch_motor.calibrate()
    logger.debug(pitch_motor)
    wait_for_motors(10)

    
    # The waist motor has to be calibrated after calibrating the shoulder/elbow parts to ensure we're not
    # moving around with fully extended arm (which the waist motor gearing doesn't like)
    waist_motor.calibrate(timeout=10000)
    logger.debug(waist_motor)

    waist_motor.to_position(50)
    shoulder_motors.to_position(88)
    elbow_motor.to_position(0)
    pitch_motor.to_position(95)
    grabber_motor.to_position(30)
    spin_motor.calibrate(timeout=5000, to_center=False)
    logger.debug(spin_motor)
    spin_motor.to_position(25)

    wait_for_motors(4)
    set_led_colors("GREEN")


# Not sure why but resetting all motors before doing anything else seems to improve reliability
reset_motors()


def log_power_info():
    logger.info('Local battery power: {}V / {}A'.format(round(power.measured_volts, 2), round(power.measured_amps, 2)))
    logger.info('Remote battery power: {}V / {}A'.format(round(remote_power.measured_volts, 2), round(remote_power.measured_amps, 2)))


def clean_shutdown(signal_received=None, frame=None):
    """ make sure all motors are stopped when stopping this script """
    logger.info('Shutting down...')
    set_led_colors("RED")
    # This seems to help?!
    # reset_motors()
    
    # Let's see if this helps with hanging stop() commands...
    # for motor in motors:
    #    motors[motor].stop_action = 'coast'

    if waist_motor.is_running:
        logger.info('waist..')
        waist_motor.stop()
    
    if shoulder_motors.is_running:
        logger.info('shoulder..')
        shoulder_motors.stop()
    
    if elbow_motor.is_running:
        logger.info('elbow..')
        elbow_motor.stop()
    
    if roll_motor.is_running:
        logger.info('roll..')
        # roll_motor.reset()
        roll_motor.stop()
    
    if spin_motor.is_running:
        logger.info('spin..')
        spin_motor.stop()
    
    if pitch_motor.is_running:
        logger.info('pitch..')
        pitch_motor.stop()
    
    if grabber_motor.is_running:
        logger.info('grabber..')
        grabber_motor.stop()

    # leds.reset()
    # remote_leds.reset()

    logger.info('Shutdown completed.')
    sys.exit(0)


def execute_move(moves, motor_wait_time=5):
    for motor, position in moves.items():
        if position is None or position == '':
            continue

        try:
            int(position)
        except ValueError:
            logger.debug('Skip invalid row, got position "{}" for motor {}'.format(position, motor))
            break

        if motor in motors:
            motors[motor].to_position(int(position), wait=False)
    
    wait_for_motors(motor_wait_time)

def moves_from_file(command_file, wait_between_steps=True):
    logger.info('Reading moves from {}...'.format(command_file))
    with open(command_file, newline='', encoding='utf-8') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:

            execute_move(row)

            # sleep time between rows
            if wait_between_steps:
                time.sleep(1)


def moves_from_userinput():
    """ moves from user input """
    logger.info('Reading moves from user input...')
    while True:
        user_input = input('CSV line: ')
        user_input_items = user_input.split(',')
        user_input_dict = {
            'waist': user_input_items[0],
            'shoulder': user_input_items[1],
            'elbow': user_input_items[2],
            'roll': user_input_items[3],
            'pitch': user_input_items[4],
            'spin': user_input_items[5],
            'grab': user_input_items[6]
        }
        execute_move(user_input_dict)


# Ensure clean shutdown on CTRL+C
signal(SIGINT, clean_shutdown)

if __name__ == "__main__":
    log_power_info()
    
    try:
        calibrate_motors()
    except CalibrationError:
        logger.error('Calibration error :(')
        clean_shutdown()
        raise
    
    
    # for cmd_file in ['pickup.csv']:  # ['commands.csv']:  # , 'waist.csv']:
    #     moves_from_file(cmd_file, wait_between_steps=False)
    
    moves_from_userinput()
    
    # failsafe
    clean_shutdown()