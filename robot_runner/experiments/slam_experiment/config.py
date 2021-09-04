from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController

from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType

from ProgressManager.RunTable.Models.RunProgress import RunProgress

# Profilers
from ina219_profiler_service.ina219_profiler_client import INA219ProfilerClient
from resource_profiler_service.resource_profiler_client import ResourceProfilerClient

import pandas as pd
from typing import Dict, List
from pathlib import Path

import csv
import sys
import rosbag
import rospy
import subprocess

class RobotRunnerConfig:
    # =================================================USER SPECIFIC NECESSARY CONFIG=================================================
    # Name for this experiment
    name:                       str             = "slam_experiment"
    # Required ROS version for this experiment to be ran with
    # NOTE: (e.g. ROS2 foxy or eloquent)
    # NOTE: version: 2
    # NOTE: distro: "foxy"
    required_ros_version:       int             = 1
    required_ros_distro:        str             = "noetic"
    # Experiment operation types
    operation_type:             OperationType   = OperationType.SEMI
    # Run settings
    time_between_runs_in_ms:    int             = 60000
    # Path to store results at
    # NOTE: Path does not need to exist, will be appended with 'name' as specified in this config and created on runtime
    results_output_path:        Path             = Path("~/Documents/experiments")
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    cpu_mem_profiler: ResourceProfilerClient
    ina219_profiler: INA219ProfilerClient

    # Dynamic configurations can be one-time satisfied here before the
    # program takes the config as-is.
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        EventSubscriptionController.subscribe_to_multiple_events([
            (RobotRunnerEvents.BEFORE_RUN,          self.before_run),
            (RobotRunnerEvents.START_RUN,           self.start_run),
            (RobotRunnerEvents.START_MEASUREMENT,   self.start_measurement),
            (RobotRunnerEvents.LAUNCH_MISSION,      self.launch_mission),
            (RobotRunnerEvents.STOP_MEASUREMENT,    self.stop_measurement),
            (RobotRunnerEvents.STOP_RUN,            self.stop_run)
            (RobotRunnerEvents.POPULATE_RUN_DATA,   self.populate_run_data),
        ])

    def create_run_table(self) -> List[Dict]:
        run_table = RunTableModel(
            factors=[
                FactorModel("algorithm", ['hector', 'cartographer', 'gmapping', 'karto']),
                FactorModel("arena", ['circular', 'point_to_point']),
                FactorModel("map_resolution", ['0.05', '0.1']),
                FactorModel("linear_update", ['0.2', '1.0']),
                FactorModel("angular_update", ['0.0174', '0.9']),
            ],
            data_columns=['avg_cpu', 'avg_mem', 'energy', 'rosbag_msg_count', 'rosbag_msg_size'],
            num_of_repetitions=5,
            randomize_order=True
        )

        run_table.create_experiment_run_table()
        return run_table.get_experiment_run_table()

    def before_run(self, run_variation) -> None:
        print(f'Next run will be at arena:\t\t{run_variation["arena"]}')
        input('\n\n>> To start run, press ENTER. <<\n\n')

    def start_run(self, context: RobotRunnerContext) -> None:
        """This is the first method that runs in a multiprocessing.Process.
        It has a DEEP COPY of the config, so not the actual one."""
        rospy.init_node("robot_runner", disable_signals=True)
        self.ina219_profiler = INA219ProfilerClient()
        self.cpu_mem_profiler = ResourceProfilerClient()

    def start_measurement(self, context: RobotRunnerContext) -> None:
        print('Starting CPU/Memory profiler')
        self.cpu_mem_profiler.start_measurement(context.run_nr)

        print('Starting INA219 profiler')
        self.ina219_profiler.start_measurement(context.run_nr)

        print('Started all profilers')

    def launch_mission(self, context: RobotRunnerContext) -> RunProgress:
        print('Launching mission')

        host = 'ubuntu@192.168.1.222'
        cmd = ' '.join([
            'python3',
            'robot-runner-private/experiments/slam_experiment/turtlebot',
            self.name,
            str(context.run_nr),
            context.run_variation['algorithm'],
            context.run_variation['arena'],
            context.run_variation['linear_update'],
            context.run_variation['angular_update'],
            context.run_variation['map_resolution'],
            f'>/home/ubuntu/mission_output/{self.name}/{context.run_nr}.txt 2>&1',
        ])

        subprocess.run(f'ssh {host} \'{cmd}\'', shell=True)
        print('Mission completed')

    def stop_measurement(self, context: RobotRunnerContext) -> None:
        print('Stopping cpu/memory measurements')
        resp_cpu_mem = self.cpu_mem_profiler.stop_measurement(context.run_nr)

        print('Stopping power measurements')
        resp_power = self.ina219_profiler.stop_measurement(context.run_nr)

        print('Stopped all profilers')

        with open(f'{context.run_dir.absolute()}/cpu_mem_measurements.csv', 'w+') as f:
            for line in zip(resp_cpu_mem.timestamps, resp_cpu_mem.cpu, resp_cpu_mem.mem):
                print(','.join(map(str, line)), file=f)

        with open(f'{context.run_dir.absolute()}/energy_measurements.csv', 'w+') as f:
            for line in zip(resp_power.timestamps, resp_power.power_mw):
                print(','.join(map(str, line)), file=f)

    def stop_run(self, context: RobotRunnerContext) -> None:
        host = 'ubuntu@192.168.1.222'

        map_file_path = f'/home/ubuntu/slam_maps/{self.name}/{context.run_nr}.pgm'
        subprocess.run(f'scp {host}:{map_file_path} {context.run_dir.absolute()}', shell=True)

        rosbag_file_path = f'/home/ubuntu/recorded_rosbags/{self.name}/{context.run_nr}.bag'
        subprocess.run(f'scp {host}:{rosbag_file_path} {context.run_dir.absolute()}', shell=True)

        log_file_path = f'/home/ubuntu/mission_output/{self.name}/{context.run_nr}.txt'
        subprocess.run(f'scp {host}:{log_file_path} {context.run_dir.absolute()}', shell=True)

        last_nonzero_cmd_vel_time = None
        for _, message, ros_timestamp in rosbag.Bag(f'{context.run_dir.absolute()}/{context.run_nr}.bag').read_messages(topics=['/cmd_vel']):
            timestamp = int(ros_timestamp.to_time() * 1000)

            if abs(message.linear.x) > 0.005 or abs(message.angular.z) > 0.005:
                last_nonzero_cmd_vel_time = timestamp

        bag_start_time = None
        with open(f'{context.run_dir.absolute()}/rosbag_measurements.csv', 'w+') as outfile:
            writer = csv.DictWriter(outfile, fieldnames=['timestamp', 'topic', 'size'])

            for topic, message, ros_timestamp in rosbag.Bag(f'{context.run_dir.absolute()}/{context.run_nr}.bag').read_messages():
                timestamp = int(ros_timestamp.to_time() * 1000)
                if bag_start_time is None:
                    bag_start_time = timestamp
                if timestamp >= bag_start_time and timestamp <= last_nonzero_cmd_vel_time + 1000:
                    writer.writerow({
                        'timestamp': timestamp,
                        'topic': topic,
                        'size': sys.getsizeof(message)
                    })

        with open(f'{context.run_dir.absolute()}/energy_measurements.csv', 'r') as infile, \
                open(f'{context.run_dir.absolute()}/trimmed_energy_measurements.csv', 'w+') as outfile:
            reader = csv.DictReader(infile, fieldnames=['timestamp', 'power_mw'])
            writer = csv.DictWriter(outfile, fieldnames=['timestamp', 'power_mw'])

            for row in reader:
                if int(row['timestamp']) >= bag_start_time and int(row['timestamp']) <= last_nonzero_cmd_vel_time + 1000:
                    writer.writerow(row)

        with open(f'{context.run_dir.absolute()}/cpu_mem_measurements.csv', 'r') as infile, \
                open(f'{context.run_dir.absolute()}/trimmed_cpu_mem_measurements.csv', 'w+') as outfile:
            reader = csv.DictReader(infile, fieldnames=['timestamp', 'cpu_usage', 'mem_usage'])
            writer = csv.DictWriter(outfile, fieldnames=['timestamp', 'cpu_usage', 'mem_usage'])

            for row in reader:
                if int(row['timestamp']) >= bag_start_time and int(row['timestamp']) <= last_nonzero_cmd_vel_time + 1000:
                    writer.writerow(row)

        rospy.signal_shutdown('stop_run')

    def populate_run_data(self, context: RobotRunnerContext) -> dict:
        print('Aggregating run data')
        variation = context.run_variation

        df = pd.read_csv(f'{context.run_dir.absolute()}/trimmed_cpu_mem_measurements.csv', sep=',', header=None)
        df.columns = ['timestamp', 'cpu', 'mem']
        variation['avg_mem'] = df['mem'].mean()
        # Ignore zero mearuements from psutil inaccuracy.
        df = df[df.cpu != 0]
        variation['avg_cpu'] = df['cpu'].mean()

        df = pd.read_csv(f'{context.run_dir.absolute()}/trimmed_energy_measurements.csv', sep=',', header=None)
        df.columns = ['timestamp', 'power_mw']

        # E = p * t
        variation['energy'] = df['power_mw'].mean() * ((df.timestamp.iat[-1] - df.timestamp.iat[0])) / 1000

        df = pd.read_csv(f'{context.run_dir.absolute()}/rosbag_measurements.csv', sep=',', header=None)
        df.columns = ['timestamp', 'topic', 'size']
        variation['rosbag_msg_count'] = df.shape[0]
        variation['rosbag_msg_size'] = df['size'].sum()

        return variation

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
