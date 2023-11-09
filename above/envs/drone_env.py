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


def to_NED(coords_UE4, ground_offset_NED, player_start_UE4):
    ''' Converts Unreal coordinates to NED system.
        Assumes PlayerStart is at (0, 0, 0) in AirSim's local NED coordinate system. '''
    #print (coords_UE4, "\n", ground_offset_NED,  "\n",  player_start_UE4)
    
    coords_NED = coords_UE4 - player_start_UE4  # Unreal uses cm and +z aiming up
    coords_NED.z_val *= -1
    coords_NED *= 0.01
    coords_NED = coords_NED + ground_offset_NED
    return coords_NED.to_numpy_array()


#def to_NED(xyz):
#    """ Unreal by default is in centimeters and z is positive-up.
#    AirSim is in meters and z is positive-down."""
#    xyz = xyz * 0.01
#    xyz[2] = -xyz[2]
#    return xyz


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
        
        # load data coord
        self.data = self.load_coord()[4:10]
        
        
        
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
        data = []
        # open file coord from spline
        with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/coord_scren.txt','r') as file:
            for line in file:
                coord_list = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]
                coord_list = airsim.Vector3r(coord_list[0], coord_list[1], coord_list[2])
                data.append(to_NED(coord_list, self.ground_offset_NED, self.player_start_UE4))
        return data


    def __del__(self):
        self.drone.reset()

    def _setup_flight(self):
        self.drone.reset()
        self.drone.enableApiControl(True)
        self.drone.armDisarm(True)

        # Set home position and velocity
#        self.drone.moveToPositionAsync(-0.55265, -31.9786, -19.0225, 10).join()
#        self.drone.moveByVelocityAsync(1, -0.67, -0.8, 5).join()

        print (self.data[5][0])
        self.drone.moveToPositionAsync(float(self.data[5][0]), float(self.data[5][1]), float(self.data[5][2]), 10).join()
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
#        imgs(img_rgb)
        return img_rgb[:, :, :1].reshape([responses[0].height, responses[0].width, 1])


        

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
        pts = self.data
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
            dist = 10000000
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
        print ("STEP")
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
