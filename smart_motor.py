#!/usr/bin/env python3
import time


class SmartMotorBase:
    """ base class for handling motors """
    _motor = None
    _speed = None
    _name = None
    _minPos = -5000  # @TODO revert to None or 5..?
    _maxPos = 5000  # @TODO revert to None
    _motorPadding = 10

    def __init__(self, motor, speed=10, name=None):
        self._motor = motor
        self._speed = speed
        self._name = name

    def calibrate(self, to_center=True):
        print('Calibrating {}...'.format(self._name))

    @property
    def maxPos(self):
        return self._maxPos

    @property
    def minPos(self):
        return self._minPos

    @property
    def centerPos(self):
        return (self._maxPos - self._minPos) / 2

    def __getattr__(self, name):
        return getattr(self._motor, name)


class StaticRangeMotor(SmartMotorBase):
    def __init__(self, motor, maxPos, speed=10, name=None):
        # let's assume we're in center upon init and fake min and max to allow moving both ways on start
        self._maxPos = maxPos / 2
        self._minPos = (maxPos / 2) * -1
        super().__init__(motor, speed, name)

    def calibrate(self, to_center=True):
        raise NotImplementedError


class LimitedRangeMotor(SmartMotorBase):
    """ handle motors with a limited range of valid movements """

    def calibrate(self, to_center=True):
        super().calibrate()
        self._motor.on(-self._speed, False)

        def checkMotorState(state):
            print(state)
            if 'overloaded' in state or 'stalled' in state:
                return True

            return False

        # self._motor.wait(checkMotorState, 10000)
        self._motor.wait_until('stalled')
        self._motor.reset()  # sets 0 point
        self._minPos = self._motor.position + self._motorPadding

        self._motor.on(self._speed, False)
        self._motor.wait_until('stalled')

        # self._motor.wait(checkMotorState, 10000)
        self._motor.stop()
        self._maxPos = self._motor.position - self._motorPadding

        if to_center:
            self._motor.on_to_position(self._speed, self.centerPos, True, True)
        
        print('Motor {} found max {}'.format(self._name, self._maxPos))


class LimitedRangeMotorSet(LimitedRangeMotor):
    """ handle a set of motors with limited range of valid movements """

    def calibrate(self, to_center=True):
        # super().calibrate()
        for motor in self._motor:
            motor.on(-self._speed, False)

        def checkMotorState(state):
            print(state)
            if 'overloaded' in state or 'stalled' in state:
                return True

            return False
        
        self._motor[1].wait_until('stalled')
        # self._motor[1].wait(checkMotorState, 10000)
        for motor in self._motor:
            motor.reset()  # sets 0 point
        
        self._minPos = self._motor[1].position + self._motorPadding

        for motor in self._motor:
            motor.on(self._speed, False)

        # self._motor[1].wait(checkMotorState, 10000)
        self._motor[1].wait_until('stalled')
        for motor in self._motor:
            motor.stop()

        self._maxPos = self._motor[1].position - self._motorPadding
        
        if to_center:
            for motor in self._motor:
                motor.on_to_position(self._speed, self.centerPos, True, False)

        # cant wait here because of motor set, so let's at least give it some time
        time.sleep(1)
        print('Motor {} found max {}'.format(self._name, self._maxPos))

    def on_to_position(self, speed, position, brake, wait):
        for motor in self._motor:
            # @TODO hardcoded no-waiting because of dual motor setup
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


class ColorSensorMotor(SmartMotorBase):
    """ handle motors which initialize valid range of movement using a color sensor """
    _sensor = None
    _color = None

    def __init__(self, motor, speed=10, name=None, sensor=None, color=None):
        self._sensor = sensor
        self._color = color
        super().__init__(motor, speed, name)

    def calibrate(self):
        super().calibrate()
        if self._sensor.color != self._color:
            # TODO: non hardcoded negative speed here to reverse direction
            self._motor.on(-self._speed, False)
            while self._sensor.color != self._color:
                time.sleep(0.1)

        self._motor.reset()

    @property
    def centerPos(self):
        return 0


class TouchSensorMotor(SmartMotorBase):
    _sensor = None

    def __init__(self, motor, speed=10, name=None, sensor=None, max=None):
        self._sensor = sensor
        self._maxPos = max
        super().__init__(motor, speed, name)

    def calibrate(self):
        super().calibrate()
        if not self._sensor.is_pressed:
            self._motor.on(-self._speed, False)
            self._sensor.wait_for_pressed()

        self._motor.reset()
