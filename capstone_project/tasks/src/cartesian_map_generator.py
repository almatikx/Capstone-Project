import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_msgs.msg import DisplayTrajectory
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

end_effector_points = []

# Callback function to process trajectory messages
import numpy as np

def interpolate_trajectory(start, end, num_points):
    # Linearly interpolate between two joint states
    return [np.linspace(s, e, num_points) for s, e in zip(start, end)]

def trajectory_callback(msg):
    global end_effector_points

    group_name = "moveit_inpipe"  # Replace with your planning group name
    move_group = MoveGroupCommander(group_name)

    rospy.loginfo("Received a trajectory message")

    # Extract robot trajectory
    robot_trajectory = msg.trajectory

    for trajectory in robot_trajectory:
        points = trajectory.joint_trajectory.points

        for i in range(len(points) - 1):
            start_positions = points[i].positions
            end_positions = points[i + 1].positions

            # Interpolate points between two consecutive trajectory waypoints
            interpolated_positions = interpolate_trajectory(start_positions, end_positions, num_points=10)

            for joint_values in zip(*interpolated_positions):
                try:
                    # Try to set joint values and get pose
                    move_group.set_joint_value_target(joint_values)
                    pose = move_group.get_current_pose().pose

                    # Extract Cartesian coordinates
                    x, y, z = pose.position.x, pose.position.y, pose.position.z
                    end_effector_points.append([x, y, z])

                except Exception as e:
                    # Skip invalid joint values
                    rospy.logwarn(f"Skipped invalid joint values: {joint_values}. Error: {e}")


# ROS Node Initialization
def main():
    rospy.init_node("cartesian_map_generator", anonymous=True)

    # Subscribe to the trajectory display topic
    rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, trajectory_callback)

    rospy.loginfo("Subscribed to /move_group/display_planned_path. Waiting for trajectories...")

    # Keep the node running until shutdown
    rospy.spin()

    # Once shutdown, plot the 3D Cartesian map
    plot_3d_map()

# Function to plot the 3D Cartesian map
def plot_3d_map():
    global end_effector_points

    if not end_effector_points:
        rospy.logwarn("No points received. Exiting...")
        return

    # Convert to a 3D array
    points = zip(*end_effector_points)

    # Plot in 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(*points, s=5, c="blue", alpha=0.7)
    ax.set_title("3D Cartesian Map of End-Effector Path")
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    plt.show()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
