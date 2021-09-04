#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.client import Client

from resource_profiler_interfaces.srv import StartMeasurement
from resource_profiler_interfaces.srv import StopMeasurement

class ResourceProfilerClient:
    __node: Node
    __start_resource_measurement: Client
    __stop_resource_measurement: Client


    def __init__(self, node):
        self.__node = node
        self.__start_resource_measurement = self.__node.create_client(StartMeasurement, 'start_resource_measurement')
        self.__stop_resource_measurement = self.__node.create_client(StopMeasurement, 'stop_resource_measurement')

    def start_measurement(self, run, cpu=True, mem=True):
        """Starts profiling."""
        while not self.__start_resource_measurement.wait_for_service(timeout_sec=1.0):
            self.__node.get_logger().info('Start measurement service not available, waiting again...')
        resp = self.__start_resource_measurement.call_async(StartMeasurement.Request(run=str(run), cpu=cpu, mem=mem))
        rclpy.spin_until_future_complete(self.__node, resp)

        return resp.result()

    def stop_measurement(self, run):
        """Stops profiling and obtains measurements."""
        while not self.__stop_resource_measurement.wait_for_service(timeout_sec=1.0):
            self.__node.get_logger().info('Stop measurement service not available, waiting again...')
        resp = self.__stop_resource_measurement.call_async(StopMeasurement.Request(run=str(run)))
        rclpy.spin_until_future_complete(self.__node, resp)

        return resp.result()


def main(args=None):
    """Example to test the profiler."""
    import time
    rclpy.init()
    client_node = rclpy.create_node('resource_profiler_client_node')

    resource_profiler_client = ResourceProfilerClient(node=client_node)
    resource_profiler_client.start_measurement(10, cpu=True, mem=True)

    time.sleep(10)

    results = resource_profiler_client.stop_measurement(10)
    print(results)


if __name__ == '__main__':
    main()
