#!/usr/bin/python3

import rospy
import threading
import psutil
import time
import os
import sys
import errno
import signal

from resource_profiler_service.srv import StartMeasurement
from resource_profiler_service.srv import StopMeasurement

class ResourceProfilerServer:
    """
    Provides services for starting and stopping resource profiling.
    
    Currently supports:
    - CPU
    - MEMORY
    
    The psutil library is used to collect the measurements.
    """
    __start_resource_measurement_service: rospy.Service
    __stop_resource_measurement_service: rospy.Service
    __ros_rate: rospy.Rate

    __thread_running: bool
    __profiling_thread: threading.Thread

    def __init__(self):
        self.__start_resource_measurement_service = rospy.Service('start_resource_measurement', StartMeasurement, self.__start_resource_measurement_callback, buff_size=65536)
        self.__stop_resource_measurement_service = rospy.Service('stop_resource_measurement', StopMeasurement, self.__stop_resource_measurement_callback, buff_size=65536)
        self.__ros_rate = rospy.Rate(50)  # Desired seasurement frequency


    def __start_resource_measurement_callback(self, request):
        """
        Starts the profiling thread upon service invocation.
        
        TODO: Add fallback for when threading fails.
        """
        self.__thread_running = True
        self.__profiling_thread = threading.Thread(target=self.__resource_profiler, args=[request.run, request.cpu, request.mem], daemon=True)
        self.__profiling_thread.start()
        
        return {
            'started': True
        }

    def __stop_resource_measurement_callback(self, request):
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

        timestamps = []
        cpu = []
        mem = []

        with open(f'resource_measurements/{request.run}.csv') as f:
            for line in f:
                line = line.split(',')
                timestamps.append(int(line[0]))
                cpu.append(float(line[1]))
                mem.append(int(line[2]))
        os.remove(f'resource_measurements/{request.run}.csv')

        return {
            'timestamps': timestamps,
            'cpu': cpu,
            'mem': mem
        }

    def exit(self):
        self.__thread_running = False
        time.sleep(1)
        self.__start_resource_measurement_service.shutdown('exit')
        self.__stop_resource_measurement_service.shutdown('exit')

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
    rospy.init_node("resource_profiler_server")
    resource_profiler = ResourceProfilerServer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        resource_profiler.exit()
        print('Server stopped cleanly')
    except BaseException:
        print('Exception in server:', file=sys.stderr)
        raise


if __name__ == '__main__':
    main()