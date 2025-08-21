import pybullet as p
import pybullet_data
import time

# Connect to physics server (GUI mode)
p.connect(p.GUI)

# Set the search path for URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a URDF robot (you can replace with your own URDF path)
robot_id = p.loadURDF("panda.urdf", useFixedBase=True)

# Get number of joints
num_joints = p.getNumJoints(robot_id)
print(f"Number of joints: {num_joints}\n")

# Loop through all joints and print their info
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    link_name = joint_info[12].decode("utf-8")
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]

    print(f"Joint {i}: {joint_name}")
    print(f"  Link Name: {link_name}")
    print(f"  Joint Type: {joint_type}")
    print(f"  Parent Index: {joint_info[16]}")
    print()

# Keep simulation running
while True:
    p.stepSimulation()
    time.sleep(0.01)
