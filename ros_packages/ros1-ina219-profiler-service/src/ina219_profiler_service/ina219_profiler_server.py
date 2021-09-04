#!/usr/bin/python3

import rospy
import time
import os
import sys
import errno
import serial

from ina219_profiler_service.srv import StartMeasurement
from ina219_profiler_service.srv import StopMeasurement

class INA219ProfilerServer:
    """
    Provides services for starting and stopping energy profiling.
    
    Relies on the Arduino circuit with an INA219 profiler,
    running the /ina219_profiler.ino program.
    """
    __start_ina219_measurement_service: rospy.Service
    __stop_ina219_measurement_service: rospy.Service
    __profiler_running: bool
    __profiler_serial: serial.Serial

    def __init__(self):
        self.__start_ina219_measurement_service = rospy.Service('start_ina219_measurement', StartMeasurement, self.__start_ina219_measurement_callback, buff_size=65536)
        self.__stop_ina219_measurement_service = rospy.Service('stop_ina219_measurement', StopMeasurement, self.__stop_ina219_measurement_callback, buff_size=65536)
        
        # NOTE: Check if the device is set correctly.
        self.__profiler_serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        print('Initialised connection with INA219 board')
        self.__profiler_serial.flush()

        self.__time_correction = 0

    def __start_ina219_measurement_callback(self, request):
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

        return {
            'started': True
        }

    def __stop_ina219_measurement_callback(self, request):
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

        timestamps = []
        power_mw = []

        while True:
            if self.__profiler_serial.in_waiting > 0:
                message = self.__profiler_serial.readline().decode('utf-8').rstrip()
                if message == 'END':
                    break
                timestamp, power_mw_temp = message.split(',')
                timestamps.append(self.__time_correction + int(timestamp))
                power_mw.append(float(power_mw_temp))

        return {
            'timestamps': timestamps,
            'power_mw': power_mw
        }

    def exit(self):
        """Stop the services and close the serial connection."""
        self.__start_ina219_measurement_service.shutdown('exit')
        self.__stop_ina219_measurement_service.shutdown('exit')
        self.__profiler_serial.close()


def main():
    """Runs the profiler server on a node."""
    rospy.init_node("ina219_profiler_server")
    ina219_profiler = INA219ProfilerServer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        ina219_profiler.exit()
        print('Server stopped cleanly')
    except BaseException:
        print('Exception in server:', file=sys.stderr)
        raise


if __name__ == '__main__':
    main()
