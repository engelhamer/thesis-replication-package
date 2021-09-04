#!/usr/bin/python3

import rospy

from ina219_profiler_service.srv import StartMeasurement
from ina219_profiler_service.srv import StopMeasurement

class INA219ProfilerClient:
    __start_ina219_measurement: rospy.ServiceProxy
    __stop_ina219_measurement: rospy.ServiceProxy

    def __init__(self):
        self.__start_ina219_measurement = rospy.ServiceProxy('start_ina219_measurement', StartMeasurement)
        self.__stop_ina219_measurement = rospy.ServiceProxy('stop_ina219_measurement', StopMeasurement)

    def start_measurement(self, run):
        try:
            rospy.wait_for_service('start_ina219_measurement')
            self.__start_ina219_measurement(str(run))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def stop_measurement(self, run):
        try:
            rospy.wait_for_service('stop_ina219_measurement')
            return self.__stop_ina219_measurement(str(run))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


def main():
    import time
    rospy.init_node('ina219_profiler_test_client')

    print('Initialising client for test')
    ina219_profiler_client = INA219ProfilerClient()

    print('Calling service to start measurements')
    ina219_profiler_client.start_measurement(10)

    print('Measurement started: waiting a while')
    time.sleep(20)

    print('Calling service to stop measurements')
    results = ina219_profiler_client.stop_measurement(10)
    print('Done:', results)


if __name__ == '__main__':
    main()
