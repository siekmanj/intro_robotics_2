import sys
sys.path.insert(0, './lib')
import time
import concurrent.futures

import numpy as np

from readerwriterlock import rwlock
from .week_3 import PhotoSensor, PhotoInterpreter, LineController

class Bus:
    def __init__(self, size):
        self.data = np.zeros(size)
        self.lock = rwlock.RWLockWriteD()

    def write(self, value):
        with self.lock.gen_wlock():
            self.data = value

    def read(self, key):
        with self.lock.gen_rlock():
            return self.data

def sensor_producer(bus, dt=1e-1):
    sensor = Sensor()
    while True:
        bus.write('reading', sensor.read())
        time.sleep(dt)

def interpreter_consumerproducer(bus_in, bus_out, dt=1e-1):
    interpreter = Interpreter(sensitivity=10)
    while True:
        bus_out.write('position', interpreter.interpret(bus_in.read('reading')))
        time.sleep(dt)

def line_controller_consumer(bus, dt=1e-1):
    controller = LineController(steering_gain=1)
    while True:
        controller.follow_line(bus.read('position'))
        time.sleep(dt)

if __name__ == '__main__':
    reset_mcu()

    sensor_bus = Bus(1)
    interp_bus = Bus(1)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        esensor = executor.submit(sensor_producer, sensor_bus, 1e-1)
        einterp = executor.submit(interpreter_consumerproducer, sensor_bus, interp_bus, 1e-1)
        econtrol = executor.submit(line_controller_consumer, interp_bus, 1e-1)
    esensor.result()
    einterp.result()
    econtrol.result()
