# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
from typing import Dict
import time

import numpy as np
import torch
import math

from pathlib import Path
import glob
import numpy as np
from franka_control.franka_env import FrankaEnv

from franka_control.util import HOMES
from franka_control.util import R, T
from franka_control.pd_control import PDControl

import torchcontrol as toco

def ArgumentParser():
    parser = argparse.ArgumentParser(description="Parameterized Actions")
    parser.add_argument("-f", "--file", type=str, help="Filename of recorded trajectory", required=True)
    parser.add_argument("-m", "--manual", action='store_true', help="Enable manual setting of initial pose")
    parser.add_argument("-vp", "--auto", action='store_true', help="Enable autonomous policy for initial pose")
    args = parser.parse_args()
    return args

class ParameterizedActions():

    def __init__(self, args):
        self.args = args
        self.name = args.file[:-4]
        gain_type = (
            "stiff" if self.name.endswith("insertion") or self.name.endswith("zip") else "default"
        )

        file_path = Path(__file__).resolve().parent.parent / "data"

        data = np.load(file_path / args.file, allow_pickle=True)
        self.rel_pose_hist, self.hz = data["traj_pose"], data["hz"]
        self.env_play = FrankaEnv(home=HOMES["cloth"], hz=self.hz, gain_type=gain_type, camera=False)
        self.env_rec = FrankaEnv(home=HOMES["cloth"], hz=self.hz, gain_type="record", camera=False)

        self.init_pos, self.init_quat, self.T_home = None, None, None

    def init_pose_setup(self):

        # This function sets up the initial pose of the robot manually. Call if args.init is False.
        print("Setting initial pose manually...")

        user_in = "r"
        while user_in == "r":
            print("Going to start playing {}".format(self.name, self.hz))
            self.env_rec.reset()
            user_in = input("Move to the initial pose and press [ENTER].")

        # Get initial pose
        self.init_pos, self.init_quat = self.env_rec.robot.get_ee_pose()
        print("Home pos: ", self.init_pos)
        print("Home rot: ", self.init_quat)
    
        #home eef frame
        self.T_home = T.from_rot_xyz(
                        rotation=R.from_quat(self.init_quat),
                        translation=self.init_pos)
    
    def execute_action(self):
    
        # This function executes the action on the robot

        user_in = "r"
        while user_in == "r":
            self.env_play.reset()
            user_in = input("Ready. Loaded {} ({} hz):".format(self.name, self.hz))

        # Create policy instance
        default_kx = torch.Tensor(self.env_play.robot.metadata.default_Kx)
        default_kxd = torch.Tensor(self.env_play.robot.metadata.default_Kxd)
        policy = PDControl(
            ee_pos_current=self.init_pos,
            ee_rot_current=self.init_quat,
            kx=default_kx,
            kxd=default_kxd,
            robot_model=self.env_play.robot.robot_model,
        )


        # Send policy
        print("\nRunning PD policy...")
        self.env_play.robot.send_torch_policy(policy, blocking=False)

        # Update policy to execute a sine trajectory on joint 6 for 5 seconds
        print("Starting playback updates...")
        # ee_pos_desired = ee_pos_initial.clone()
        # ee_rot_desired = ee_rot_initial.clone()

        time_to_go = 50.0
        m = 0.07  # magnitude of sine wave (rad)
        # T = 0.5  # period of sine wave
        hz = 30  # update frequency
        for i in range(len(self.rel_pose_hist)):
            T_frame = self.T_home * self.rel_pose_hist[i] 
            print("Translation: ", T_frame.translation())
            print("Rotation: ", T_frame.rotation().as_quat())
            self.env_play.robot.update_current_policy({"ee_pos_desired": T_frame.translation(), "ee_rot_desired": T_frame.rotation().as_quat()})
            # print(f"Desired position: {ee_pos_desired}")
            # print("Current robot pos : %s", env_play.robot.get_ee_pos()[0])
            time.sleep(1 / self.hz)

        print("Terminating PD policy...")
        state_log = self.env_play.robot.terminate_current_policy()

if __name__ == "__main__":

    args = ArgumentParser()
    if args.manual == True:
        pa = ParameterizedActions(args)
        pa.init_pose_setup()
        pa.execute_action()
    elif args.auto == True:
        print(args)
        print("Feature not implemented yet!")
    else:
        print("Please set either --manual or --auto to True.")
