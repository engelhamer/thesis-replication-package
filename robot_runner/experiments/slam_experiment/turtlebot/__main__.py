import sys
import os
import subprocess
import threading
import time


def main(experiment_name, run, algorithm, arena, linear_update,
         angular_update, map_resolution):
    # Get the location of this python file in order to access the config,
    # launch and rosbag folders.
    folder_path = os.path.dirname(os.path.realpath(__file__))

    rosbag_dir = os.path.expanduser(f'~/recorded_rosbags/{experiment_name}')
    if not os.path.exists(rosbag_dir):
        os.makedirs(rosbag_dir)

    # Start rosbag record
    rosbag_proc = subprocess.Popen([
        'rosbag',
        'record',
        '-a',
        '-O'
        f'{rosbag_dir}/{run}.bag'
    ])

    # Start SLAM algorithm
    slam_proc = subprocess.Popen([
        'roslaunch',
        f'{folder_path}/launch/turtlebot3_slam.launch',
        f'slam_methods:={algorithm}',
        f'folder_path:={folder_path}',
        f'linear_update:={linear_update}',
        f'angular_update:={angular_update}',
        f'map_resolution:={map_resolution}',
    ])

    # Wait a while for it to initialize
    time.sleep(5)

    # Play rosbag for arena
    subprocess.run([
        'rosbag',
        'play',
        f'{folder_path}/rosbags/{arena}.bag'
    ])

    map_dir = os.path.expanduser(f'~/slam_maps/{experiment_name}')

    if not os.path.exists(map_dir):
        os.makedirs(map_dir)

    # Save map
    subprocess.run([
        'rosrun',
        'map_server',
        'map_saver',
        '-f',
        f'{map_dir}/{run}'
    ])

    # Stop SLAM
    slam_proc.terminate()

    # Stop rosbag recording
    rosbag_proc.terminate()


if __name__ == '__main__':
    main(*sys.argv[1:])
