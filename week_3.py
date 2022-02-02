import sys
sys.path.insert(0, './lib')
import logging
from adc import ADC
from servo import Servo
from pwm import PWM
from pin import Pin
from utils import reset_mcu
import numpy as np
import atexit
import time

import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np


DARK = 1
LIGHT = -1

class PhotoSensor:
    """ Reads pins A0 through A2 using the ADC class """
    def __init__(
            self,
    ):
        # Create ADC objects to read pins A0->A2
        self.pinreaders = [ADC(x) for x in ["A0", "A1", "A2"]]

    def read(self):
        """ Read each of the ADC pins and return a list of their values """
        return [p.read() for p in self.pinreaders]

class PhotoInterpreter:
    """ Interprets values from the PhotoSensor class """
    def __init__(
            self,
            sensitivity: float = 10,
    ):
        # Save our sensitivity parameter
        self.sensitivity = sensitivity

    def interpret(self, x: list, polarity = DARK):
        """ Interpret the newest sensor reading of x """
        l, m, r = x

        value = polarity * np.clip(((m - l) + (r - m))/2 / self.sensitivity, -1, 1)
        logging.info(f"Line relative position is {value:3.1f}")
        return value

class LineController:
    def __init__(self, steering_gain: float = 1):

        # Values and pin numbers taken from picar-x
        self.steering_servo = Servo(PWM('P2'))
        self.left_motor_speed = PWM("P13")
        self.right_motor_speed = PWM("P12")
        self.left_motor_dir = Pin("D4")
        self.right_motor_dir = Pin("D5")
        self.steering_angle = 0
        self.steering_gain = steering_gain

        for p in [self.left_motor_speed, self.right_motor_speed]:
            p.period(4095)
            p.prescaler(10)

        atexit.register(self.cleanup)

    def set_motor_speed(self, motor, speed):
        motor_spd = [self.left_motor_speed, self.right_motor_speed]
        motor_dir = [self.left_motor_dir, self.right_motor_dir]

        direction = np.sign(speed)
        speed = abs(speed)
        if speed != 0:
            speed = int(speed / 2) + 50
        try:
            if direction < 0:
                motor_dir[motor-1].high()
                motor_spd[motor-1].pulse_width_percent(speed)
            else:
                motor_dir[motor-1].low()
                motor_spd[motor-1].pulse_width_percent(speed)
        except:
            traceback.print_exc()
            logging.error(f"Motor {motor-1} is not responding")

    def forward(self, speed):
        current_angle = self.steering_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > 40:
                abs_current_angle = 40
            power_scale = (100 - abs_current_angle) / 100.0
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, speed) # fast wheel
                self.set_motor_speed(2, -1*speed * power_scale) # slow wheel
            else:
                self.set_motor_speed(1, speed * power_scale) #slow wheel
                self.set_motor_speed(2, -1*speed ) #fast wheel
        else:
            #if no turn angle is commanded, spin both motors at the same speed
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)

    def steer(self, angle):
        self.steering_angle = angle
        self.steering_servo.angle(angle)

    def follow_line(self, line_value):
        self.forward(0.5)
        self.steer(line_value * self.steering_gain)

    def cleanup(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

class CameraLineDetector:
    def __init__(self, camera):
        self.camera = camera
        self.position = 0

    def estimate(self, frame, polarity: int = cv2.THRESH_BINARY_INV, alpha: float = 0.3):
        # Crop the image to remove the irrelevant upper half
        crop = frame[frame.shape[0]//2:]

        # Apply a filter which supposedly keeps edges sharp while removing noise
        # (important because my apartment is carpeted and all of these techniques
        # HATE carpet)
        output = cv2.bilateralFilter(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), 9, 75, 75)

        # Threshold the image using Otsu's method (adaptive or something)
        _, thresh = cv2.threshold(output, 0, 255, polarity | cv2.THRESH_OTSU)

        # Find the contour
        cnts, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # If we detected any contours, get the largest one and get its mean x position
        x_pos = 0
        if len(cnts) > 0:
            largest_contour = np.argmax([cv2.contourArea(c) for c in cnts])
            x, _, w, _ = cv2.boundingRect(cnts[largest_contour])

            # Get the x position in the range [-1, 1]
            x_pos = ((x + w/2) - camera.resolution[0]/2) / camera.resolution[0]
            cv2.rectangle(output, (x, 0), (x+w, 100), (0, 255, 0), 2)
        return x_pos



if __name__ == '__main__':
    reset_mcu()
    use_camera = True

    controller = LineController(steering_gain=30)
    if use_camera:
        camera = PiCamera()
        camera.resolution = (640,480)
        camera.framerate = 30
        rawCapture = PiRGBArray(camera, size=camera.resolution)  

        camera_detector = CameraLineDetector(camera)
        for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
            position = camera_detector.estimate(frame.array)
            controller.follow_line(position)
            time.sleep(0.01)
            rawCapture.truncate(0)   # Release cache
           
    else:
        sensor = PhotoSensor()
        interpreter = PhotoInterpreter(sensitivity=100)

        while True:
            position = interpreter.interpret(sensor.read())
            controller.follow_line(position)
            time.sleep(0.01)

