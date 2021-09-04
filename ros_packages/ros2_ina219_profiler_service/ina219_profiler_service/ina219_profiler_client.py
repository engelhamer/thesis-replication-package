#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.client import Client

from ina219_profiler_interfaces.srv import StartMeasurement
from ina219_profiler_interfaces.srv import StopMeasurement

class INA219ProfilerClient:
    __node: Node
    __start_ina219_measurement: Client
    __stop_ina219_measurement: Client


    def __init__(self, node):
        self.__node = node
        self.__start_ina219_measurement = self.__node.create_client(StartMeasurement, 'start_ina219_measurement')
        self.__stop_ina219_measurement = self.__node.create_client(StopMeasurement, 'stop_ina219_measurement')

    def start_measurement(self, run):
        """Starts profiling."""
        while not self.__start_ina219_measurement.wait_for_service(timeout_sec=1.0):
            self.__node.get_logger().info('Start measurement service not available, waiting again...')
        resp = self.__start_ina219_measurement.call_async(StartMeasurement.Request(run=str(run)))
        rclpy.spin_until_future_complete(self.__node, resp)

    def stop_measurement(self, run):
        """Stops profiling and obtains measurements."""
        while not self.__stop_ina219_measurement.wait_for_service(timeout_sec=1.0):
            self.__node.get_logger().info('Stop measurement service not available, waiting again...')
        resp = self.__stop_ina219_measurement.call_async(StopMeasurement.Request(run=str(run)))
        rclpy.spin_until_future_complete(self.__node, resp)

        return resp.result()


def main(args=None):
    """Example to test the profiler."""
    import time
    rclpy.init()
    client_node = rclpy.create_node('ina219_profiler_client_node')

    ina219_profiler_client = INA219ProfilerClient(node=client_node)
    ina219_profiler_client.start_measurement(10)

    time.sleep(10)

    results = ina219_profiler_client.stop_measurement(10)
    print(results)


if __name__ == '__main__':
    main()
