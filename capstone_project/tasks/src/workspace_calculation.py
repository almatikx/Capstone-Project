import rospy
from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Initialize ROS node
rospy.init_node("workspace_visualizer", anonymous=True)

# Initialize MoveIt interfaces
robot = RobotCommander()
group_name = "moveit_inpipe"
move_group = MoveGroupCommander(group_name)

# Parameters for sampling
num_samples = 2000
workspace_points = []

print(f"Sampling workspace for planning group: {group_name}")

for i in range(num_samples):
    # Generate random valid joint values
    random_joint_values = move_group.get_random_joint_values()

    # Set the joint values to the robot's state
    move_group.set_joint_value_target(random_joint_values)

    # Get the FK pose (end-effector position) directly
    pose = move_group.get_random_pose().pose  # Get the FK result for the group
    
    # Extract the position (x, y, z)
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    workspace_points.append([x, y, z])

    if (i + 1) % 100 == 0:
        print(f"Sampled {i + 1}/{num_samples} poses...")

# Convert to NumPy array for easier processing
workspace_points = np.array(workspace_points)

# Plot the workspace
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.scatter(
    workspace_points[:, 0],  # X-axis values
    workspace_points[:, 1],  # Y-axis values
    workspace_points[:, 2],  # Z-axis values
    s=1, c="blue", alpha=0.5
)
ax.set_title(f"Workspace of {group_name}")
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
plt.show()

