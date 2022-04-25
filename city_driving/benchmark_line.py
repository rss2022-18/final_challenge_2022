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

FILE_VEL = [1, 2, 3]


def parse_file(file_name):
    bag = rosbag.Bag(file_name)
    actual_distance = []
    actual_angle = []
    time = []
    for topic, msg, t in bag.read_messages(topics=['/relative_cone']):
        actual_distance.append(msg.y_pos)
        actual_angle.append(np.arctan(msg.y_pos/msg.x_pos) * 180/np.pi)
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
    fig, axs = plt.subplots(3, 1)
    # ax2 = ax.twinx()
    plt.ylabel('Distance (m)')
    plt.xlabel('ROS Time (s)')
    # Plot the desired and actual distance
    min_t = 0
    max_t = 0
    colors = 'brc'
    colors2 = 'myk'
    duplicate_axes = []
    for i in range(len(times)):
        t = [t_i - times[i][0] for t_i in times[i]]
        axs[i].plot(t, actual_distances[i], color=colors[i],
                    linewidth=1, label="Y Displacement(m) for {} m/s".format(FILE_VEL[i]))
        ax2 = axs[i].twinx()
        ax2.plot(t, actual_angles[i], color=colors2[i],
                 linewidth=1, label="Angle Error(degrees) for {} m/s".format(FILE_VEL[i]))
        min_t = min(min_t, t[0])
        max_t = max(max_t, t[-1])
        duplicate_axes.append(ax2)
        lines_labels = [ax.get_legend_handles_labels()
                        for ax in (axs[i], ax2)]
        lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
        axs[i].legend(lines, labels, loc='lower right',
                      bbox_to_anchor=(1.7, 1))
    ax2.set_ylabel('Angle Error\n (degrees)')
    axs[0].set_title(
        'Real-World Line Controller Performance (Velocity Tests)')
    # plt.legend(title='(From cone)', loc='center left',
    #            bbox_to_anchor=(1, 0))
    # lines_labels = [ax.get_legend_handles_labels()
    #                 for ax in axs] + [ax.get_legend_handles_labels()
    #                                   for ax in duplicate_axes]
    # lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
    # fig.legend(lines, labels, title='From cone',
    #            loc='lower right')
    # fig.legend(handles, labels, loc='upper center')
    # ax.hlines(PARKING_DISTANCE, min_t, max_t, color='r',
    #           linewidth=2, label='Desired distance')
    # ax2.plot(t, actual_angles, 'blue', linewidth=2, label='Calculated angle')
    plt.savefig(output_name, bbox_inches='tight')


if __name__ == '__main__':
    # if len(sys.argv) < 2:
    #     print('Usage: python benchmark.py <bag_file_name> <controller> <safety>')
    #     sys.exit(1)
    # file_name = sys.argv[1]
    # controller = sys.argv[2]
    # safety = sys.argv[3]
    output_name = 'line_output.png'
    # if len(sys.argv) > 4:
    #     output_name = sys.argv[4]
    file_distances, file_angles, file_times = [], [], []
    for file_name in ['2022-03-08-18-37-55.bag', '2022-03-08-18-40-27.bag', '2022-03-08-18-41-59.bag']:
        actual_distances, actual_angles, times = parse_file(file_name)
        file_distances.append(actual_distances)
        file_angles.append(actual_angles)
        file_times.append(times)
    # print(benchmark_controller(actual_distances, times, controller, safety))
    plot_controller(file_distances, file_angles, file_times, output_name)
