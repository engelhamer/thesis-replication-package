#!/usr/bin/python3

import rclpy
import time
import os
import sys
import errno
import serial

from ina219_profiler_interfaces.srv import StartMeasurement
from ina219_profiler_interfaces.srv import StopMeasurement

from rclpy.node import Node
from rclpy.service import Service
from rclpy.timer import Rate

class INA219ProfilerServer:
    """
    Provides services for starting and stopping energy profiling.
    
    Relies on the Arduino circuit with an INA219 profiler,
    running the /ina219_profiler.ino program.
    """
    __node: Node
    __start_ina219_measurement_service: Service
    __stop_ina219_measurement_service: Service
    __ros_rate: Rate

    __profiler_running: bool
    __profiler_serial: serial.Serial

    def __init__(self, node):
        self.__node = node
        self.__start_ina219_measurement_service = self.__node.create_service(StartMeasurement, 'start_ina219_measurement', self.__start_ina219_measurement_callback)
        self.__stop_ina219_measurement_service = self.__node.create_service(StopMeasurement, 'stop_ina219_measurement', self.__stop_ina219_measurement_callback)

        # NOTE: Check if the device is set correctly.
        self.__profiler_serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        print('Initialised connection with INA219 board')
        self.__profiler_serial.flush()

        self.__time_correction = 0


    def __start_ina219_measurement_callback(self, request, response):
        """
        Starts the profiling routine on the Arduino upon service invocation.

        The procedure is:
        1. Send 'START' over the serial connection
        2. Wait until the Arduino responds with its local timestamp
        3. Compute the difference between the epoch millisecond timestamp and
           the Arduino's local timestamp, which acts as a correction value for
           translating timestamps when the measurements are obtained       
        """
        self.__profiler_running = True
        self.__profiler_serial.flush()

        self.__profiler_serial.write(b'START')
        while True:
            start_time = time.time()
            if self.__profiler_serial.in_waiting > 0:
                self.__start_time = int(start_time * 1000)
                arduino_start_time = int(self.__profiler_serial.readline().decode('utf-8').rstrip())
                self.__time_correction = self.__start_time - arduino_start_time
                break
        
        response.started = True
        return response

    def __stop_ina219_measurement_callback(self, request, response):
        """
        Stops the profiling routine on the Arduino upon service invocation and
        obtains the measurements.

        The procedure is:
        1. Send 'STOP' over the serial connection
        2. Read all measurements from the Arduino which are sent over the serial
           connection, according to the format:
           ARDUINO TIMESTAMP,POWER IN MILLIWATTS
        3. Translate the Arduino's local timestamp to the epoch millisecond timestamp
        4. Add the timestamp and measurement to their respective lists
        5. Keep reading measurements until the Arduino sends 'END' over the serial
           connection
        """
        self.__profiler_running = False
        self.__profiler_serial.flush()
        self.__profiler_serial.write(b'STOP')
        stop_time = time.time()
        stop_time = int(stop_time * 1000)

        response.timestamps = []
        response.power_mw = []

        while True:
            if self.__profiler_serial.in_waiting > 0:
                message = self.__profiler_serial.readline().decode('utf-8').rstrip()
                if message == 'END':
                    break
                timestamp, power_mw_temp = message.split(',')
                response.timestamps.append(self.__time_correction + int(timestamp))
                response.power_mw.append(float(power_mw_temp))

        return response

    def exit(self):
        self.__node.destroy_service(self.__start_ina219_measurement_service)
        self.__node.destroy_service(self.__stop_ina219_measurement_service)
        self.__profiler_serial.close()


def main():
    """Runs the profiler server on a node."""
    rclpy.init()
    server_node = rclpy.create_node("ina219_profiler_server")
    ina219_profiler = INA219ProfilerServer(server_node)

    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        ina219_profiler.exit()
        print('Server stopped cleanly')
    except BaseException:
        print('Exception in server:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        server_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()