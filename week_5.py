import sys
sys.path.insert(0, './lib')
sys.path.insert(0, './RossROS')

import rossros

from ultrasonic import Ultrasonic
from utils import reset_mcu
from sensor import Ultrasonic
from pin import Pin

from week_3 import PhotoSensor, PhotoInterpreter, LineController

class UltrasonicSensor:
    def __init__(self):
        trig = Pin('D8')
        echo = Pin('D9')
        self.sensor = Ultrasonic(trig, echo)
    
    def read(self):
        return self.sensor.read()

class UltrasonicInterpreter:
    def __init__(self):
        self.stop_at = 25 #cm

    def interpret(self, sensor_reading):
        if sensor_reading < stop_at:
            return 1 # Robot should stop if we can't find distance or are below threshold
        else:
            return 0 # Robot should keep going if the distance is above threshold

class WallStoppingLineController(LineController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def follow_line(self, line_value, wall_value):
        if wall_value == 1:
            self.set_motor_speed(1, 0)
            self.set_motor_speed(2, 0)
            self.steer(0)
        else:
            self.forward(0.5)
            self.steer(line_value * self.steering_gain)

if __name__ == '__main__':
    reset_mcu()

    photosensor = PhotoSensor()
    photointerp = PhotoInterpreter()

    ultrasonicsensor = UltrasonicSensor()
    ultrasonicinterp = UltrasonicInterpreter()

    controller = WallStoppingLineController()

    def clean():
        LineController().cleanup()
    atexit.register(clean)

    dt = 1e-1

    threads = []

    photosensor_bus = rossros.Bus(name='photosensorbus')
    threads += [rossros.Producer(photosensor.read,
                                 photosensor_bus,
                                 delay=dt,
                                 name='photosensor')]

    photointerp_bus = rossros.Bus(name='photointerpbus')
    threads += [rossros.ConsumerProducer(photointerp.interpret,
                                         photosensor_bus,
                                         photointerp_bus,
                                         delay=dt,
                                         name='photointerp')]

    ultrasonicsensor_bus = rossros.Bus(name='ultrasonicsensorbus')
    threads += [rossros.Producer(ultrasonicsensor.read,
                                 ultrasonicsensor_bus,
                                 delay=dt,
                                 name='sensor')]

    ultrasonicinterp_bus = rossros.Bus(name='ultrasonicinterpbus')
    threads += [rossros.ConsumerProducer(ultrasonicinterp.interpret,
                                         ultrasonicsensor_bus,
                                         ultrasonicinterp_bus,
                                         delay=dt,
                                         name='interp')]

    threads += [rossros.Consumer(controller.follow_line,
                                 (photointerp_bus, ultrasonicinterp_bus),
                                 delay=dt,
                                 name='control')]
    rossros.runConcurrently(threads)
