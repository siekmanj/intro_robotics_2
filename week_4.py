import atexit
import sys
sys.path.insert(0, './lib')
import time
import concurrent.futures

import numpy as np

from readerwriterlock import rwlock
from utils import reset_mcu
from week_3 import PhotoSensor, PhotoInterpreter, LineController
import traceback

class Bus:
    def __init__(self, size):
        self.data = np.zeros(size)
        self.lock = rwlock.RWLockWriteD()

    def write(self, value):
        with self.lock.gen_wlock():
            self.data = value

    def read(self):
        with self.lock.gen_rlock():
            data = self.data
        return data

def sensor_producer(bus, dt=5e-1):
    sensor = PhotoSensor()
    while True:
        bus.write(sensor.read())
        time.sleep(dt)

def interpreter_consumerproducer(bus_in, bus_out, dt=1e-1):
    try:
        interpreter = PhotoInterpreter(sensitivity=10)
        while True:
            pos = interpreter.interpret(bus_in.read())
            print("line is", pos)
            bus_out.write(pos)
            time.sleep(dt)
    except:
        traceback.print_exc()

def line_controller_consumer(bus, dt=1e-1):
    try:
        controller = LineController(steering_gain=25)
        while True:
            pos = bus.read()
            print("Following line", pos)
            controller.follow_line(pos)
            time.sleep(dt)
    except:
        traceback.print_exc()

if __name__ == '__main__':
    reset_mcu()

    def clean():
        LineController().cleanup()
    atexit.register(clean)

    sensor_bus = Bus(3)
    interp_bus = Bus(1)
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        esensor = executor.submit(sensor_producer, sensor_bus, 1e-3)
        einterp = executor.submit(interpreter_consumerproducer, sensor_bus, interp_bus, 1e-3)
        econtrol = executor.submit(line_controller_consumer, interp_bus, 1e-2)
    esensor.result()
    einterp.result()
    econtrol.result()
