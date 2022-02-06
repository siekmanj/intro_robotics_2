import atexit
import sys
sys.path.insert(0, './lib')
sys.path.insert(0, './RossROS')
import traceback

import rossros

from ultrasonic import Ultrasonic
from utils import reset_mcu
from ultrasonic import Ultrasonic
from pin import Pin
import time

from week_3 import PhotoSensor, PhotoInterpreter, LineController

class UltrasonicSensor:
    def __init__(self):
        trig = Pin('D2')
        echo = Pin('D3')
        self.sensor = Ultrasonic(trig, echo)
    
    def read(self):
        val = self.sensor.read()
        return val

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
        try:
            if wall_value == 1:
                self.set_motor_speed(1, 0)
                self.set_motor_speed(2, 0)
            else:
                self.forward(0.5)
            self.steer(line_value * self.steering_gain)
            time.sleep(0.05)

        except:
            traceback.print_exc()

if __name__ == '__main__':

    reset_mcu()
    photosensor = PhotoSensor()
    photointerp = PhotoInterpreter(sensitivity=100)

    ultrasonicsensor = UltrasonicSensor()
    ultrasonicinterp = UltrasonicInterpreter()

    controller = WallStoppingLineController(steering_gain=30)

    def clean():
        LineController().cleanup()
        reset_mcu()
    atexit.register(clean)

    dt = 1e-1

    threads = []
    timer_bus = rossros.Bus(name="timer")


    photosensor_bus = rossros.Bus(name='photosensorbus')
    threads += [rossros.Producer(photosensor.read,
                                 photosensor_bus,
                                 termination_busses=(timer_bus,),
                                 delay=dt,
                                 name='photosensor')]

    photointerp_bus = rossros.Bus(name='photointerpbus')
    threads += [rossros.ConsumerProducer(photointerp.interpret,
                                         photosensor_bus,
                                         photointerp_bus,
                                         termination_busses=(timer_bus,),
                                         delay=dt,
                                         name='photointerp')]

    ultrasonicsensor_bus = rossros.Bus(name='ultrasonicsensorbus')
    threads += [rossros.Producer(ultrasonicsensor.read,
                                 ultrasonicsensor_bus,
                                 termination_busses=(timer_bus,),
                                 delay=dt,
                                 name='sensor')]

    ultrasonicinterp_bus = rossros.Bus(name='ultrasonicinterpbus')
    threads += [rossros.ConsumerProducer(ultrasonicinterp.interpret,
                                         ultrasonicsensor_bus,
                                         ultrasonicinterp_bus,
                                         termination_busses=(timer_bus,),
                                         delay=dt,
                                         name='interp')]

    threads += [rossros.Consumer(controller.follow_line,
                                 (photointerp_bus, ultrasonicinterp_bus),
                                 termination_busses=(timer_bus,),
                                 delay=0.25,
                                 name='control')]

    timer = rossros.Timer((timer_bus,), duration=5, delay=0.1, termination_busses=(timer_bus,), name="master timer")

    rossros.runConcurrently(threads)
