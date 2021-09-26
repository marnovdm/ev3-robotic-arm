import threading
import time
from ev3dev2.sensor.lego import ColorSensor


class WaistAlignThread(threading.Thread):
    devices = None
    state = None
    logger = None
    
    # Define speeds
    FULL_SPEED = 100
    FAST_SPEED = 75
    NORMAL_SPEED = 50
    SLOW_SPEED = 25
    VERY_SLOW_SPEED = 10

    def __init__(self, logger, devices, state):
        threading.Thread.__init__(self)
        self.logger = logger
        for key in devices:
            setattr(self, key, devices[key])
    
        self.state = state

    def run(self):
        self.logger.info("WaistAlignThread running!")
        while self.state['running']:
            if self.state['waist_target_color'] != 0 and not self.state['aligning_waist']:
                self._align_waist_to_color(self.state['waist_target_color'])
            time.sleep(2)  # prevent performance impact, drawback is you need to hold the button for a bit before it registers

        self.logger.info("WaistAlignThread stopping!")

    def _align_waist_to_color(self, waist_target_color):
        if waist_target_color == -1:
            target_color = ColorSensor.COLOR_RED
        elif waist_target_color == 1:
            target_color = ColorSensor.COLOR_BLUE
        else:
            # if someone asks us to move to an unknown/unmapped
            # color, just make this a noop.
            return

        # Set a flag for the MotorThread to prevent stopping the waist motor while
        # we're trying to align it
        self.state['aligning_waist'] = True

        # If we're not on the correct color, start moving but make sure there's a
        # timeout to prevent trying forever.
        if self.color_sensor.color != target_color:
            self.logger.info('Moving to color {}...'.format(target_color))
            self.waist_motor.on(self.NORMAL_SPEED)

            max_iterations = 100
            iterations = 0
            while self.color_sensor.color != target_color:
                # wait a bit between checks. Ideally there would be a wait_for_color()
                # method or something, but as far as I know that's not possible with the
                # current libraries, so we do it like this.
                time.sleep(0.1)

                # prevent running forver
                iterations += 1
                if iterations >= max_iterations:
                    self.logger.info('Failed to align waist to requested color {}'.format(target_color))
                    break

            # we're either aligned or reached a timeout. Stop moving.
            self.waist_motor.stop()

        # update flag for MotorThead so waist control works again.
        self.state['aligning_waist'] = False