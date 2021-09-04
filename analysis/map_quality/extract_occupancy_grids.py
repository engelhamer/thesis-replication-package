import rosbag

EXPERIMENT_FOLDER = '/home/engel/Documents/experiments/slam_experiment'

for run_id in range(1, 321):
    print(f'Now doing run:\t{run_id}')
    map_width = None
    map_height = None
    map_values = None

    for _, message, ros_timestamp in rosbag.Bag(f'{EXPERIMENT_FOLDER}/run_{run_id}/{run_id}.bag').read_messages(topics=['/map']):
        timestamp = int(ros_timestamp.to_time() * 1000)

        map_width = message.info.width
        map_height = message.info.height
        map_values = list(message.data)

    with open(f'./run_{run_id}/final_map.csv', 'w+') as outfile:
        print(map_width, file=outfile)
        print(map_height, file=outfile)
        print(','.join(str(value) for value in map_values), file=outfile)
