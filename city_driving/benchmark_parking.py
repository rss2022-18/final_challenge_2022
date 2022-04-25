import rosbag
import sys
import os
import numpy as np
import matplotlib

if os.environ.get('DISPLAY', '') == '':
    print('Using non-interactive Agg backend')
    matplotlib.use('Agg')

import matplotlib.pyplot as plt

DESIRED_D = 0.75
PARKING_DISTANCE = 0.5


def parse_file(file_name):
    bag = rosbag.Bag(file_name)
    actual_distance = []
    actual_angle = []
    time = []
    for topic, msg, t in bag.read_messages(topics=['/relative_cone']):
        actual_distance.append(
            np.sqrt(msg.x_pos**2 + msg.y_pos**2))
        actual_angle.append(np.arctan(msg.x_pos/msg.y_pos) * 180/np.pi)
        time.append(t.to_sec())
    bag.close()
    return actual_distance, actual_angle, time


def benchmark_controller(actual_distances, times, controller='PD', speed=1, alpha=4):
    N = len(actual_distances)
    loss = sum([abs(actual_distances[i] - DESIRED_D)
                for i in range(1, N)])/(N-1)
    score = 1/(1+(alpha*loss)**2)
    return "Controller: {}, Safety: {}, Loss:{} Score: {}".format(controller, safety, loss, score)


def plot_controller(actual_distances, actual_angles, times, output_name='controller_output.png'):
    """
    Takes log file containing desired distance, actual 
    distance, and ROS time and plots the desired distance 
    line and the actual distance over time.
    Args:
        file_name: the name of the file to plot.
        output_name: the name of the output file.

    """
    # Extract the desired distance and actual distance
    t = [t_i - times[0] for t_i in times]
    fig, ax = plt.subplots()
    # ax2 = ax.twinx()
    plt.title('Real-World Parking Controller Performance (Test setup)')
    plt.ylabel('Distance (m)')
    plt.xlabel('ROS Time (s)')
    # Plot the desired and actual distance
    ax.hlines(PARKING_DISTANCE, t[0], t[-1], color='r',
              linewidth=2, label='Desired distance')
    ax.plot(t, actual_distances, 'b',
            linewidth=1, label='Actual Distance')
    # ax2.plot(t, actual_angles, 'blue', linewidth=2, label='Calculated angle')
    plt.legend(title='(From cone)', loc='center left', bbox_to_anchor=(1, 0))
    plt.savefig(output_name, bbox_inches='tight')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python benchmark.py <bag_file_name> <controller> <safety>')
        sys.exit(1)
    file_name = sys.argv[1]
    controller = sys.argv[2]
    safety = sys.argv[3]
    output_name = 'controller_output.png'
    if len(sys.argv) > 4:
        output_name = sys.argv[4]
    actual_distances, actual_angles, times = parse_file(file_name)
    print(benchmark_controller(actual_distances, times, controller, safety))
    plot_controller(actual_distances, actual_angles, times, output_name)
