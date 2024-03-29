import airsim
import numpy as np
import math
import time
from argparse import ArgumentParser

import gym
import cv2

from gym import spaces
from above.envs.airsim_env import AirSimEnv

def imgs(x):
    cv2.imshow('image', np.array(x))
    key = cv2.waitKey(1) & 0xff
    if key==ord('q'):
        cv2.destroyAllWindows()


# https://github.com/microsoft/AirSim/issues/3229
# https://github.com/microsoft/AirSim/issues/1677
# https://gist.github.com/laurelkeys/cf198df240854f8ea107524ad1c5e22d
# https://github.aiurs.co/ethz-asl/unreal_airsim/blob/master/docs/coordinate_systems.md

def convert_pos_UE_to_AS(origin_UE : np.array, pos_UE : np.array):
    pos = np.zeros(3, dtype=np.float)
    pos[0] = pos_UE[0] - origin_UE[0]
    pos[1] = pos_UE[1] - origin_UE[1]
    pos[2] = -5
#    pos[2] = - pos_UE[2] + origin_UE[2]
    return pos / 100


class AirSimDroneEnv(AirSimEnv):
    def __init__(self, ip_address, step_length, image_shape):
        super().__init__(image_shape)
        self.step_length = step_length
        self.image_shape = image_shape
        


        self.state = {
            "position": np.zeros(3),
            "collision": False,
            "prev_position": np.zeros(3),
        }

        self.drone = airsim.MultirotorClient(ip=ip_address)
        
        self.player_start_UE4 = self.drone.getHomeGeoPoint()  # obs.: this can be changed by OriginGeopoint in settings.json
        self.ground_offset_NED = self.drone.simGetVehiclePose().position  # assumes the drone is at PlayerStart
        assert self.ground_offset_NED.x_val == self.ground_offset_NED.y_val == 0
        
        self.origin_UE = self.drone.getMultirotorState().kinematics_estimated.position.to_numpy_array()
        # load data coord
        self.data = self.load_coord()#[4:10]
        self.data2 = self.load_coord2()#[4:10]
        
        self.data_to_learn = [d.to_numpy_array() for d in self.data]
        
        self.action_space = spaces.Discrete(7)
        self._setup_flight()
#        self.image_request = airsim.ImageRequest(
#            3, airsim.ImageType.DepthPerspective, True, False
#        )
        
        # RGB 
        self.image_request = airsim.ImageRequest(
            "bottom_center", airsim.ImageType.Scene, False, False
        ) 
        
        # Camera pose
#        camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(-90), 0, 0)) #radians
#        self.drone.simSetCameraPose("0", camera_pose)


    def load_coord(self):
        coord_list = []
        # open file coord from spline
        with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/coord_scren.txt','r') as file:
            for line in file:
                data = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]
                data = convert_pos_UE_to_AS(self.origin_UE, np.array(data))
                coord_list.append(airsim.Vector3r(data[0], data[1], data[2]))
        return coord_list  

    def load_coord2(self):
        coord_list = []
        # open file coord from spline
        with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/test.txt','r') as file:
            for line in file:
                data = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]
                data = convert_pos_UE_to_AS(self.origin_UE, np.array(data))
                coord_list.append(airsim.Vector3r(data[0], data[1], data[2]))
        return coord_list 

    def __del__(self):
        self.drone.reset()

    def _setup_flight(self):
        self.drone.reset()
        self.drone.enableApiControl(True)
        self.drone.armDisarm(True)

        # Set home position and velocity
#        self.drone.moveToPositionAsync(-0.55265, -31.9786, -19.0225, 10).join()
#        self.drone.moveByVelocityAsync(1, -0.67, -0.8, 5).join()

        data = self.data2[0].to_numpy_array()
#        self.drone.moveToPositionAsync(int(data[0]), int(data[1]), int(data[2]), 5).join()
        self.drone.moveByVelocityAsync(1, -0.67, -0.8, 5).join()

# https://microsoft.github.io/AirSim/image_apis/
    def transform_obs(self, responses):
# v1
#        img1d = np.array(responses[0].image_data_float, dtype=np.float)
#        img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
#        img2d = np.reshape(img1d, (responses[0].height, responses[0].width))
#        from PIL import Image
#        image = Image.fromarray(img2d)
#        im_final = np.array(image.resize((84, 84)).convert("L"))
#        print (im_final.shape)
#        return im_final.reshape([84, 84, 1])

        # RGB
        # get numpy array
        img1d = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8) 
        # reshape array to 4 channel image array H X W X 4
        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
#        imgs(img_rgb[:, :, :1])
#        return img_rgb[:, :, :1].reshape([responses[0].height, responses[0].width, 1])
#        imgs(img_rgb[:, :, :])
        return img_rgb[:, :, :].reshape([responses[0].height, responses[0].width, 3])

        

    def _get_obs(self):
        responses = self.drone.simGetImages([self.image_request])
        image = self.transform_obs(responses)
        self.drone_state = self.drone.getMultirotorState()

        self.state["prev_position"] = self.state["position"]
        self.state["position"] = self.drone_state.kinematics_estimated.position
        self.state["velocity"] = self.drone_state.kinematics_estimated.linear_velocity

        collision = self.drone.simGetCollisionInfo().has_collided
        self.state["collision"] = collision

#        print (self.drone_state.kinematics_estimated.position)
        return image

    def _do_action(self, action):
        quad_offset = self.interpret_action(action)
        quad_vel = self.drone.getMultirotorState().kinematics_estimated.linear_velocity
        self.drone.moveByVelocityAsync(
            quad_vel.x_val + quad_offset[0],
            quad_vel.y_val + quad_offset[1],
            quad_vel.z_val + quad_offset[2],
            5,
        ).join()

    def _compute_reward(self):
        thresh_dist = 7
        beta = 1

        z = -10
        pts = self.data_to_learn
#        pts = [
#            np.array([-0.55265, -31.9786, -19.0225]),
#            np.array([48.59735, -63.3286, -60.07256]),
#            np.array([193.5974, -55.0786, -46.32256]),
#            np.array([369.2474, 35.32137, -62.5725]),
#            np.array([541.3474, 143.6714, -32.07256]),
#        ]

        quad_pt = np.array(
            list(
                (
                    self.state["position"].x_val,
                    self.state["position"].y_val,
                    self.state["position"].z_val,
                )
            )
        )

        if self.state["collision"]:
            reward = -100
        else:
#            dist = 10000000
            dist = 10
            for i in range(0, len(pts) - 1):
                dist = min(
                    dist,
                    np.linalg.norm(np.cross((quad_pt - pts[i]), (quad_pt - pts[i + 1])))
                    / np.linalg.norm(pts[i] - pts[i + 1]),
                )

            if dist > thresh_dist:
                reward = -10
            else:
                reward_dist = math.exp(-beta * dist) - 0.5
                reward_speed = (
                    np.linalg.norm(
                        [
                            self.state["velocity"].x_val,
                            self.state["velocity"].y_val,
                            self.state["velocity"].z_val,
                        ]
                    )
                    - 0.5
                )
                reward = reward_dist + reward_speed

        done = 0
        if reward <= -10:
            done = 1

        return reward, done

    def step(self, action):
        self._do_action(action)
        obs = self._get_obs()
        reward, done = self._compute_reward()
        return obs, reward, done, self.state

    def reset(self):
        self._setup_flight()
        return self._get_obs()

    def interpret_action(self, action):
        if action == 0:
            quad_offset = (self.step_length, 0, 0)
        elif action == 1:
            quad_offset = (0, self.step_length, 0)
        elif action == 2:
            quad_offset = (0, 0, self.step_length)
        elif action == 3:
            quad_offset = (-self.step_length, 0, 0)
        elif action == 4:
            quad_offset = (0, -self.step_length, 0)
        elif action == 5:
            quad_offset = (0, 0, -self.step_length)
        else:
            quad_offset = (0, 0, 0)

        return quad_offset
