# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from parameterized_action import ParameterizedAction
from visualization_msgs.msg import Marker, MarkerArray

from scipy.spatial.transform import Rotation

from franka_control.util import R, T

import yaml
import rospy
from pathlib import Path

import torch

def leftfinger_tf():
    # Define the transformation matrix for the left finger

    file_path = Path(__file__).resolve().parent.parent / "config" / "fingertip_tf.yaml"

    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    
        translation = torch.Tensor(config["link8_leftfinger_tip"]["translation"])
        rotation_quat = torch.Tensor(config["link8_leftfinger_tip"]["rotation"])

    leftfinger_tf = T.from_rot_xyz(
                        rotation=R.from_quat(rotation_quat),
                        translation=translation)
    
    return leftfinger_tf

class AutonomousPolicy:
    def __init__(self):
        self.vector_viz_sub = rospy.Subscriber('/visual_feedback/azure/push_vector', MarkerArray, queue_size=10, callback=self.vector_viz_callback)
        
    def add_safety_constraint(self, pose):

        safe_pose = pose
        safe_pose.position.z = pose.position.z + 0.009
        safe_pose.position.y = pose.position.y + 0.051
        safe_pose.position.x = pose.position.x + 0.025

        return safe_pose

    def rotate(self,pose):

        #rotate quaternion by -5 degrees around X axis

        rotated_pose = pose

        ori = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        quat = Rotation.from_quat(ori)

        print(pose)

        print(quat.as_quat())

        quat_rot_1 = Rotation.from_euler('x', 1.5, degrees=True)

        quat_rot = Rotation.from_euler('y', 45, degrees=True) * quat_rot_1

        final_quat = quat_rot * quat

        rotated_pose.orientation.x = final_quat.as_quat()[0]
        rotated_pose.orientation.y = final_quat.as_quat()[1]
        rotated_pose.orientation.z = final_quat.as_quat()[2]
        rotated_pose.orientation.w = final_quat.as_quat()[3]

        print("Rotated!!")
        print(final_quat.as_quat())

        return rotated_pose

    def vector_viz_callback(self, msg):

        finger_pose = msg.markers[0].pose

        finger_safe_pose_init = self.add_safety_constraint(finger_pose)
        finger_safe_pose = self.rotate(finger_safe_pose_init)

        finger_init_pos = torch.Tensor([finger_safe_pose.position.x, finger_safe_pose.position.y, finger_safe_pose.position.z])
        finger_init_quat = torch.Tensor([finger_safe_pose.orientation.x, finger_safe_pose.orientation.y, finger_safe_pose.orientation.z, finger_safe_pose.orientation.w])        
        
        T_finger = T.from_rot_xyz(
                        rotation=R.from_quat(finger_init_quat),
                        translation=finger_init_pos)
        
        print("T_finger: ", T_finger)
        print("-------------------------")
        print(leftfinger_tf().inv())
        print("-------------------------")
        print(T_finger * leftfinger_tf().inv())
        
        T_link8 = T_finger * leftfinger_tf().inv()

        link_init_pos = T_link8.translation()
        link_init_quat = T_link8.rotation().as_quat()

        pa = ParameterizedAction(name="push_cloth_0.npz", init_pos=link_init_pos, init_quat=-1*link_init_quat) #hack 

        # user_in = "r"
        # while user_in == "r":
        #     user_in = input("Ready. Press 'enter' to go to initial pose")
        
        # go to initial ee pose
        print("link8 pos: ", link_init_pos)
        print("link8 rot: ", link_init_quat)
        print("Going to initial pose")
        pa.go_home()

        p, q = pa.get_ee_pose()

        print("Current pos: ", p)
        print("Current quat: ", q)
        print("-------------------------")
        print("Initial pos: ", link_init_pos)
        print("Initial quat: ", link_init_quat)

        # assert torch.allclose(p, link_init_pos)
        # assert torch.allclose(q, link_init_quat)

        # user_in = "r"
        # while user_in == "r":
        #     user_in = input("Ready. Press 'enter' to execute action")

        print("Executing action")
        # execute the action
        pa.execute_action()

        print("Moving aside")
        pa.move_aside()

if __name__ == "__main__":
    rospy.init_node('autonomous_policy', anonymous=True)
    auto_policy = AutonomousPolicy()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught. Exiting.")
    

        




