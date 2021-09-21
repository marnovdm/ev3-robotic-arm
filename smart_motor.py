#!/usr/bin/env python3
import time


class SmartMotorBase:
    """ base class for handling motors """
    _motor = None
    _speed = None
    _name = None
    _min_position = None
    _max_position = None
    _motorPadding = 10

    def __init__(self, motor, name, speed=30, padding=10, inverted=False, debug=False):
        self._motor = motor
        self._name = name
        self._speed = speed
        self._padding = padding
        self._inverted = inverted
        self._debug = debug

    def calibrate(self, to_center=True):
        print('Calibrating {}...'.format(self._name))

    @property
    def max_position(self):
        return self._max_position

    @property
    def min_position(self):
        return self._min_position

    @property
    def center_position(self):
        return (self._max_position - self._min_position) / 2

    @property
    def current_position(self):
        return self._motor.position

    def __getattr__(self, name):
        return getattr(self._motor, name)

    def __str__(self):
        return "Motor: {}, min: {}, center: {}, max: {}, current: {}".format(self._motor, self.min_position, self.center_position, self.max_position, self.current_position)

class StaticRangeMotor(SmartMotorBase):
    def __init__(self, motor, name, speed=30, padding=10, inverted=False, debug=False, max_position=None):
        # let's assume we're in center upon init and fake min and max to allow moving both ways on start
        self._max_position = max_position / 2
        self._min_position = (max_position / 2) * -1
        super().__init__(motor, name, speed, padding, inverted, debug)

    def calibrate(self, to_center=True):
        raise NotImplementedError


class LimitedRangeMotor(SmartMotorBase):
    """ handle motors with a limited range of valid movements, using stall detection to determine usable range """

    def calibrate(self, to_center=True):
        super().calibrate(to_center)
        self._motor.on(self._speed, False)

        def checkMotorState(state):
            print(state)
            if 'overloaded' in state or 'stalled' in state:
                return True

            return False

        # self._motor.wait(checkMotorState, 10000)
        self._motor.wait_until('stalled')
        self._motor.reset()  # sets 0 point
        self._min_position = self._motor.position + self._motorPadding

        self._motor.on(-self._speed, False)
        self._motor.wait_until('stalled')

        # self._motor.wait(checkMotorState, 10000)
        self._motor.stop()
        self._max_position = self._motor.position - self._motorPadding

        if to_center:
            self._motor.on_to_position(self._speed, self.center_position, True, True)

        print('Motor {} found max {}'.format(self._name, self._max_position))


class MotorSetBase:
    def on_to_position(self, speed, position, brake, wait):
        for motor in self._motor:
            if motor == self._motor[-1]:
                # if last motor in this set, honor wait variable
                motor.on_to_position(speed, position, brake, wait)
            else:
                # if not last motor in this set, don't ever consider waiting
                # even though it may have been requested
                motor.on_to_position(speed, position, brake, False)

    def reset(self):
        for motor in self._motor:
            motor.reset()

    def stop(self):
        for motor in self._motor:
            motor.stop()

    def on(self, speed):
        for motor in self._motor:
            motor.on(speed)

    @property
    def is_running(self):
        return self._motor[0].is_running

    @property
    def current_position(self):
        return self._motor[0].position

    def __str__(self):
        return "Motor: {}, min: {}, center: {}, max: {}, current: {}".format(self._motor, self.min_position, self.center_position, self.max_position, self.current_position)



class LimitedRangeMotorSet(MotorSetBase, SmartMotorBase):
    """ handle a set of motors with limited range of valid movements, using stall detection to determine usable range """

    def calibrate(self, to_center=True):
        super().calibrate(to_center)
        for motor in self._motor:
            motor.on(self._speed, False)

        def checkMotorState(state):
            print(state)
            if 'overloaded' in state or 'stalled' in state:
                return True

            return False

        self._motor[1].wait_until('stalled')
        # self._motor[1].wait(checkMotorState, 10000)
        for motor in self._motor:
            motor.reset()  # sets 0 point

        self._min_position = self._motor[1].position + self._motorPadding

        for motor in self._motor:
            motor.on(-self._speed, False)

        # self._motor[1].wait(checkMotorState, 10000)
        self._motor[1].wait_until('stalled')
        for motor in self._motor:
            motor.stop()

        self._max_position = self._motor[1].position - self._motorPadding

        if to_center:
            for motor in self._motor:
                if motor == self._motor[-1]:
                    # wait on motor completion if last motor in set
                    motor.on_to_position(self._speed, self.center_position, True, True)
                else:
                    # don't wait for completion if not last motor in set
                    motor.on_to_position(self._speed, self.center_position, True, False)

        print('Motor {} found max {}'.format(self._name, self._max_position))


class ColorSensorMotor(SmartMotorBase):
    """ handle motors which initialize valid range of movement using a color sensor """
    _sensor = None
    _color = None

    def __init__(self, motor, name, speed=10, padding=10, inverted=False, debug=False, sensor=None, color=None):
        self._sensor = sensor
        self._color = color
        super().__init__(motor, name, speed, padding, inverted, debug)

    def calibrate(self, to_center=True):
        super().calibrate(to_center)
        if self._sensor.color != self._color:
            self._motor.on(self._speed, False)
            while self._sensor.color != self._color:
                time.sleep(0.1)

            self._motor.stop()

        self._motor.reset()
        print('found min position')
        self._min_position = 0
        
        # determine full circle rotation length
        while self._sensor.color == self._color:
            self._motor.on(self._speed, False)
            time.sleep(0.1)

        # wait to go full circle
        if self._sensor.color != self._color:
            self._motor.on(self._speed, False)
            while self._sensor.color != self._color:
                time.sleep(0.1)

            self._motor.stop()
        
        print('found max position')
        self._max_position = self._motor.position

        if to_center:
            self._motor.on_to_position(self._speed, self.center_position, True, True)


class TouchSensorMotor(SmartMotorBase):
    """
    Moves the motor until the related touch sensor is pressed and set's that position to 0.
    The max_position argument is used to determine the max valid position.
    """
    _sensor = None

    def __init__(self, motor, name, speed=10, padding=10, inverted=False, debug=False, sensor=None, max_position=None):
        if not sensor:
            raise ValueError('missing sensor argument')

        self._sensor = sensor
        self._max_position = max_position
        self._min_position = 0
        super().__init__(motor, name, speed, padding, inverted, debug)

    def calibrate(self, to_center=True):
        super().calibrate(to_center)
        if not self._sensor.is_pressed:
            self._motor.on(self._speed, False)
            self._sensor.wait_for_pressed()

        self._motor.reset()

        if to_center:
            self._motor.on_to_position(self._speed, self.center_position, True, True)


class TouchSensorMotorSet(MotorSetBase, SmartMotorBase):
    _sensor = None

    def __init__(self, motor, name, speed=10, padding=10, inverted=False, debug=False, sensor=None, max_position=None):
        if not sensor:
            raise ValueError('missing sensor argument')

        self._sensor = sensor
        self._max_position = max_position
        self._min_position = 0
        super().__init__(motor, name, speed, padding, inverted, debug)

    def calibrate(self, to_center=True):
        super().calibrate(to_center)
        if not self._sensor.is_pressed:
            for motor in self._motor:
                if self._inverted:
                    motor.on(-self._speed, False)
                else:
                    motor.on(self._speed, False)

            self._sensor.wait_for_pressed()

        for motor in self._motor:
            motor.reset()

        if to_center:
            for motor in self._motor:
                if motor == self._motor[-1]:
                    # wait on motor completion if last motor in set
                    motor.on_to_position(self._speed, self.center_position, True, True)
                else:
                    # don't wait for completion if not last motor in set
                    motor.on_to_position(self._speed, self.center_position, True, False)
