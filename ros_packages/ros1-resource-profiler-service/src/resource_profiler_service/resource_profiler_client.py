#!/usr/bin/python3

import rospy

from resource_profiler_service.srv import StartMeasurement
from resource_profiler_service.srv import StopMeasurement

class ResourceProfilerClient:
    """
    Service client which facilitates starting and stopping resource
    profiling.
    
    Currently supports:
    - CPU
    - MEMORY
    """
    __start_resource_measurement: rospy.ServiceProxy
    __stop_resource_measurement: rospy.ServiceProxy

    def __init__(self):
        self.__start_resource_measurement = rospy.ServiceProxy('start_resource_measurement', StartMeasurement)
        self.__stop_resource_measurement = rospy.ServiceProxy('stop_resource_measurement', StopMeasurement)

    def start_measurement(self, run, cpu=True, mem=True):
        """Starts profiling."""
        try:
            rospy.wait_for_service('start_resource_measurement')
            self.__start_resource_measurement(str(run), cpu, mem)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))   

    def stop_measurement(self, run):
        """Stops profiling and obtains measurements."""
        try:
            rospy.wait_for_service('stop_resource_measurement')
            resource_measurements = self.__stop_resource_measurement(str(run))
            return resource_measurements
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


def main():
    """Example to test the profiler."""
    import time
    rospy.init_node('resource_profiler_client')

    resource_profiler_client = ResourceProfilerClient()
    resource_profiler_client.start_measurement(10, cpu=True, mem=True)

    time.sleep(2)

    results = resource_profiler_client.stop_measurement(10)
    print(results)


if __name__ == '__main__':
    main()
