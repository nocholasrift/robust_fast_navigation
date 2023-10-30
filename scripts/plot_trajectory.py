import rosbag
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory

def traj_plot():
    # Path to your ROS bag file
    bag_file = '/media/nick/Fudgcicle/ICRA_Nick_2024/sims/room_recovery2_great.bag'

    # Define the topic and message types you want to extract
    topic = '/reference_trajectory'  # Update this to your actual topic

    # Lists to store time and x position values
    time_list = []
    x_position_list = []

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        count = 0
        for topic, msg, t in bag.read_messages(topics=[topic]):
            for point in msg.points:
                time_list.append(point.time_from_start.to_sec())
                x_position_list.append(point.positions[0])

            # Create the plot
            plt.figure(figsize=(10, 6))
            plt.plot(time_list, x_position_list, marker='o')
            plt.xlabel('Time (seconds)')
            plt.ylabel('X Position')
            plt.title('X Position vs. Time')
            plt.grid(True)
            # plt.show()
            plt.savefig('/media/nick/Fudgcicle/ICRA_Nick_2024/sims/imgs/room_recovery_traj_' + str(count) + '.png')

            time_list.clear()
            x_position_list.clear()

            count += 1

def ref_plot():
    # Path to your ROS bag file
    bag_file = '/media/nick/Fudgcicle/ICRA_Nick_2024/sims/room_recovery2_great.bag'

    # Define the topic and message types you want to extract
    topic1 = '/current_reference'  # Update this to your actual topic
    topic2 = '/traj_point'

    # Lists to store time and x position values
    time_list = []
    x_position_list = []
    y_position_list = []

    time_diff_list = []  # List to store time differences between messages
    mpc_ref_points_x = []
    mpc_ref_points_y = []
    
    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        count = 0
        prev_timestamp = None
        for topic, msg, t in bag.read_messages(topics=[topic1, topic2]):
            if topic == '/current_reference':
                point = msg
                if prev_timestamp is not None:
                    time_diff = t.to_sec() - prev_timestamp
                    time_diff_list.append(time_diff)

                prev_timestamp = t.to_sec()
                time_list.append(t.to_sec())
                x_position_list.append(point.positions[0])
                y_position_list.append(point.positions[1])

            else:
                mpc_ref_points_x.append(msg.point.x)
                mpc_ref_points_y.append(msg.point.y)


        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.plot(x_position_list, y_position_list, marker='o')
        plt.plot(mpc_ref_points_x, mpc_ref_points_y, marker='o', color='orange', label='Time Differences')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('XY Position')
        plt.grid(True)

        plt.figure(figsize=(10, 6))
        plt.plot(time_list, x_position_list, marker='o')
        plt.xlabel('Time (seconds)')
        plt.ylabel('X Position')
        plt.title('X Position vs. Time')
        # plt.xlim(y_min, y_max)
        # plt.ylim(x_min, x_max)
        plt.grid(True)

        # Create the plot for time differences
        plt.figure(figsize=(10, 6))
        plt.plot(time_list[1:], time_diff_list, marker='o', color='orange', label='Time Differences')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Time Differences (seconds)')
        plt.title('Time Differences between Consecutive Messages')
        plt.grid(True)
        plt.legend()

        plt.show()



if __name__ == '__main__':
    # traj_plot()
    ref_plot()
