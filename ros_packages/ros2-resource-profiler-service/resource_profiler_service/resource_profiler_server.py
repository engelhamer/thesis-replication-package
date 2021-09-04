#!/usr/bin/python3

import rclpy
import threading
import psutil
import time
import os
import sys
import errno
import signal

from resource_profiler_interfaces.srv import StartMeasurement
from resource_profiler_interfaces.srv import StopMeasurement

from rclpy.node import Node
from rclpy.service import Service
from rclpy.timer import Rate

class ResourceProfilerServer:
    """
    Provides services for starting and stopping resource profiling.
    
    Currently supports:
    - CPU
    - MEMORY
    
    The psutil library is used to collect the measurements.
    """
    __node: Node
    __start_resource_measurement_service: Service
    __stop_resource_measurement_service: Service
    __ros_rate: Rate

    __thread_running: bool
    __profiling_thread: threading.Thread

    def __init__(self, node):
        self.__node = node
        self.__start_resource_measurement_service = self.__node.create_service(StartMeasurement, 'start_resource_measurement', self.__start_resource_measurement_callback)
        self.__stop_resource_measurement_service = self.__node.create_service(StopMeasurement, 'stop_resource_measurement', self.__stop_resource_measurement_callback)
        self.__ros_rate = self.__node.create_rate(50)  # Desired seasurement frequency

    def __start_resource_measurement_callback(self, request, response):
        """
        Starts the profiling thread upon service invocation.
        
        TODO: Add fallback for when threading fails.
        """
        self.__thread_running = True
        self.__profiling_thread = threading.Thread(target=self.__resource_profiler, args=[request.run, request.cpu, request.mem], daemon=True)
        self.__profiling_thread.start()
        
        response.started = True
        return response

    def __stop_resource_measurement_callback(self, request, response):
        """
        Stops the profiling thread upon service invocation and returns the
        collected measurements.

        The service response (as defined in srv/StopMeasurment) contains:
        - A list of epoch timestamps in milliseconds
        - A list of CPU usage measurements corresponding to the timestamps
        - A list of memory usage measurements corresponding to the timestamps
        
        TODO: Add fallback for when threading fails.
        """
        self.__thread_running = False
        time.sleep(1)

        response.timestamps = []
        response.cpu = []
        response.mem = []

        filename = f'resource_measurements/{request.run}.csv'
        with open(filename) as f:
            for line in f:
                line = line.split(',')
                response.timestamps.append(int(line[0]))
                response.cpu.append(float(line[1]))
                response.mem.append(int(line[2]))
        os.remove(filename)

        return response

    def exit(self):
        self.__thread_running = False
        time.sleep(1)
        self.__node.destroy_service(self.__start_resource_measurement_service)
        self.__node.destroy_service(self.__stop_resource_measurement_service)

    def __resource_profiler(self, run, cpu, mem):
        """Profiles resource utilisation and writes measurements to a file.

        File format:
        epoch milliseconds timestamp,cpu,memory
        
        TODO: Use the cpu and mem params to decide which to profile
        (currently both are always on)
        """        
        # Run id is used as filename.
        filename = f'resource_measurements/{run}.csv'
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        while self.__thread_running:
            with open(filename, 'a+') as f:
                virtual_memory = psutil.virtual_memory()
                # timestamp in ms,cpu utilisation percentage,mem usage bytes
                print(f'{int(time.time() * 1000)},{psutil.cpu_percent(interval=0.0)},{virtual_memory.total - virtual_memory.available}', file=f)

                self.__ros_rate.sleep()


def main():
    """Runs the profiler server on a node."""
    rclpy.init()
    server_node = rclpy.create_node("resource_profiler_server")
    resource_profiler = ResourceProfilerServer(server_node)

    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        resource_profiler.exit()
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