import threading


class MotorThread(threading.Thread):
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

    def _calculate_speed(self, speed, max=100):
        if self.state['speed_modifier'] == 0:
            return min(speed, max)
        elif self.state['speed_modifier'] == -1:  # dpad up
            return min(speed * 1.5, max)
        elif self.state['speed_modifier'] == 1:  # dpad down
            return min(speed / 1.5, max)

    def run(self):
        self.logger.info("MotorThread running!")
        # os.system('setfont Lat7-Terminus12x6')
        self.leds.set_color("LEFT", "BLACK")
        self.leds.set_color("RIGHT", "BLACK")
        self.remote_leds.set_color("LEFT", "BLACK")
        self.remote_leds.set_color("RIGHT", "BLACK")
        # sound.play_song((('C4', 'e'), ('D4', 'e'), ('E5', 'q')))
        self.leds.set_color("LEFT", "GREEN")
        self.leds.set_color("RIGHT", "GREEN")
        self.remote_leds.set_color("LEFT", "GREEN")
        self.remote_leds.set_color("RIGHT", "GREEN")

        self.logger.info("Starting main loop...")
        while self.state['running']:
            # Proportional control
            if self.state['shoulder_speed'] != 0:
                self.shoulder_motors.on(self.state['shoulder_speed'], self.state['shoulder_speed'])
            elif self.shoulder_motors.is_running:
                self.shoulder_motors.stop()

            # Proportional control
            if self.state['elbow_speed'] != 0:
                self.elbow_motor.on(self.state['elbow_speed'])
            elif self.elbow_motor.is_running:
                self.elbow_motor.stop()

            # on/off control
            if not self.state['aligning_waist']:
                if self.state['waist_left']:
                    self.waist_motor.on(self._calculate_speed(-self.SLOW_SPEED))
                elif self.state['waist_right']:
                    self.waist_motor.on(self._calculate_speed(self.SLOW_SPEED))
                elif self.waist_motor.is_running:
                    self.waist_motor.stop()

            # on/off control
            if self.state['roll_left']:
                self.roll_motor.on(self._calculate_speed(-self.SLOW_SPEED))
            elif self.state['roll_right']:
                self.roll_motor.on(self._calculate_speed(self.SLOW_SPEED))
            elif self.roll_motor.is_running:
                self.roll_motor.stop()

            # on/off control
            #
            # Pitch affects grabber as well, but to a lesser degree. We could improve this
            # in the future to adjust grabber based on pitch movement as well.
            if self.state['pitch_up']:
                self.pitch_motor.on(self._calculate_speed(self.VERY_SLOW_SPEED))
            elif self.state['pitch_down']:
                self.pitch_motor.on(self._calculate_speed(-self.VERY_SLOW_SPEED))
            elif self.pitch_motor.is_running:
                self.pitch_motor.stop()

            # on/off control
            #
            # If we keep spinning, the grabber motor can get stuck because it remains stationary
            # but is forced to move around the worm gear. We need to adjust it while spinning.
            #
            # spin motor: 7:1 (=23.6RPM)
            # grabber motor: 1:1 (=165RPM) untill the worm gear which we need to keep steady
            #
            # So, I think the grabber_motor needs to move 7 times slower than the spin_motor
            # to maintain it's position.
            #
            # NOTE: I'm using knob wheels to control the grabber, which is not smoothly rotating
            # at these low speeds. Therefor the grabber has to move a bit quicker for me, but I
            # think when using regular gears the 7 ratio should be sufficient.
            # NOTE: Yes, with regular gears the calculated ratio is correct!
            GRABBER_SPIN_RATIO = 7
            if self.state['spin_left']:
                spin_motor_speed = self._calculate_speed(-self.SLOW_SPEED)
                self.spin_motor.on(spin_motor_speed)
                if self.grabber_motor:
                    # determine grabber_motor speed based on spin_motor speed & invert
                    grabber_spin_sync_speed = (spin_motor_speed / GRABBER_SPIN_RATIO) * -1
                    self.grabber_motor.on(grabber_spin_sync_speed, False)
                    # logger.info('Spin motor {}, grabber {}'.format(spin_motor_speed, grabber_spin_sync_speed))
            elif self.state['spin_right']:
                spin_motor_speed = self._calculate_speed(self.SLOW_SPEED)
                self.spin_motor.on(spin_motor_speed)
                if self.grabber_motor:
                    # determine grabber_motor speed based on spin_motor speed & invert
                    grabber_spin_sync_speed = (spin_motor_speed / GRABBER_SPIN_RATIO) * -1
                    self.grabber_motor.on(grabber_spin_sync_speed, False)
                    # logger.info('Spin motor {}, grabber {}'.format(spin_motor_speed, grabber_spin_sync_speed))
            elif self.spin_motor.is_running:
                self.spin_motor.stop()
                if self.grabber_motor:
                    self.grabber_motor.stop()

            # on/off control - can only control this directly if we're not currently spinning
            elif self.grabber_motor:
                if self.state['grabber_open']:
                    self.grabber_motor.on(self._calculate_speed(self.NORMAL_SPEED), False)
                elif self.state['grabber_close']:
                    self.grabber_motor.on(self._calculate_speed(-self.NORMAL_SPEED), False)
                elif self.grabber_motor.is_running:
                    self.grabber_motor.stop()

        self.logger.info("MotorThread stopping!")
