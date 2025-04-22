import numpy as np
import time
import cv2
from dataclasses import dataclass
from collections import defaultdict

# from gello_controller import GELLOcontroller
from mobilegello.gello_controller import GELLOcontroller
from env.cameras import RealSenseCamera
import torch
from scipy.spatial.transform import Rotation as R
import requests
from common_utils import ibrl_utils as utils
import common_utils
import threading
import pyrealsense2 as rs

_ROBOT_CAMERAS = {
    "urgello3": {
        # "corner2": "944622074035",
        # "eye_in_hand": "032622072103",
    }
}

# PROP_KEYS = ["ee_pos", "joint_pos"]
PROP_KEYS_EE = ["ee_pos", "gripper_qpos"]
PROP_KEYS_JP = ["joint_pos"]

# # Euler - Quat Conversion Functions
# def eul_2_quat(rpy):
#     rotation = R.from_rotvec(rpy)
#     return rotation.as_quat()

# def quat_2_eul(quat):
#     rotation = R.from_quat(quat)
#     return rotation.as_euler('xyz', degrees=False)

# -------------------------------------------------------------------------------------- Video Stream Class
class VideoStream:
    def __init__(self, url):
        self.url = url
        self.capture = cv2.VideoCapture(url)
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.capture.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def read(self):
        with self.lock:
            return self.frame

    def stop(self):
        self.stopped = True
        self.capture.release()


# ------------------------------------------------------------------------------------- GelloEnvConfig Class

@dataclass
class GelloEnvConfig:
    task: str = "pickup"
    episode_length: int = 200
    robot: str = "Follower" # "Leader" or "Follower"
    control_hz: float = 15.0
    image_size: int = 224
    # rl_image_size: int = 96
    rl_image_size: int = 224
    use_depth: int = 0
    # rl_camera: str = "robot0_eye_in_hand"
    rl_camera: str = "corner2"
    randomize: int = 0
    show_camera: int = 0
    drop_after_terminal: int = 1
    record: int = 0
    save_dir: str = ""
    prop_stack: int = 1
    obs_stack: int = 1
    use_state: int = 0
    state_stack: int = 1

    run_mode: str = "testbc" # test, teleop(datacollect), replay 
    teleop_mode: str = "passive" # passive, active
    rate_hz: int = 5
    action_type: str = "jp_abs" # jp_abs, jp_delta, ee_abs, ee_delta

    def __post_init__(self):
        
        self.rl_cameras = self.rl_camera.split("+")
        print("ENV Rl cameras: ", self.rl_cameras)


# ------------------------------------------------------------------------------------- GelloEnv Class

class Gelloenv:

    def __init__(self, cfg: GelloEnvConfig):
        self.cfg = cfg
        self.task = self.cfg.task
        self.robot_str = self.cfg.robot

        self.run_mode = self.cfg.run_mode
        self.rate_hz = self.cfg.rate_hz
        self.step_delay = 1.0 / self.rate_hz
        self.action_type = self.cfg.action_type
        self.teleop_mode = self.cfg.teleop_mode

        print(f" wrapper {self.run_mode}")
        if self.run_mode == "testbc" or self.run_mode == "replay":
            self.controller = GELLOcontroller(self.robot_str, torque_start=True)
        elif self.run_mode == "teleop":
            if self.teleop_mode == "active":
                self.controller = GELLOcontroller(self.robot_str, torque_start=False)
            elif self.teleop_mode == "passive":
                self.controller = GELLOcontroller(self.robot_str, torque_start=True)


        if self.run_mode == "teleop" and self.teleop_mode == "passive":
            self.passive_controller = GELLOcontroller("Leader", torque_start=False)            
            self.actpas_offset = np.array(self.passive_controller.robot.read_position()) - np.array(self.controller.robot.read_position())
            print(f"Offset: {self.actpas_offset}")


        self.episode_length = self.cfg.episode_length
        self.device = "cuda"

        self.image_size = self.cfg.image_size
        self.rl_image_size = self.cfg.rl_image_size
        self.camera_str = "camera1"
        self.show_camera = self.cfg.show_camera
        self.rl_camera = ""
        self.control_hz = 100
        self.resize_transform = None
        self.rl_cameras = self.cfg.rl_cameras

        self.robot_str = "Follower"

        self.prop_shape = (6,)
        
        if self.action_type[:2] == "ee":
            self.action_dim = (4,)
        elif self.action_type[:2] == "jp":
            self.action_dim = 6
    
        self.time_step = 0

        # RealSense Camera
        # Configure depth and color streams
        # self.pipeline = rs.pipeline()
        # self.real_config = rs.config()
        # self.real_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # # Start streaming
        # self.pipeline.start(self.real_config)
                

        # ESP32
        # self.url ="http://192.168.230.187:81/stream"
        self.url_index = 4
        # self.url_video ="http://192.168.212.187/video"
        # self.video_stream = VideoStream(self.url_video).start()

        self.cap = cv2.VideoCapture(self.url_index)
        if not self.cap.isOpened():
            print("Error: Unable to open video stream")
            exit()

        # stack stuff
        self.past_obses = defaultdict(list)
        self.prop_stack = self.cfg.prop_stack
        self.obs_stack = self.cfg.obs_stack

        self.observation_shape: tuple[int, ...] = (3*self.obs_stack, cfg.rl_image_size, cfg.rl_image_size)

        if self.cfg.use_state:
            self.observation_shape: tuple[int] = (6*self.cfg.state_stack,)



    def get_image_from_usb(self):
        ret, frame = self.cap.read()
        image = cv2.resize(frame, (self.rl_image_size, self.rl_image_size))
        return image
    

    def get_frame(self):
        try:
            # Send a GET request to the ESP32-CAM stream URL
            response = requests.get(self.url, stream=True)
            if response.status_code == 200:
                bytes_data = bytes()
                for chunk in response.iter_content(chunk_size=1024):
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        return frame
        except Exception as e:
            print(f"Error capturing frame: {str(e)}")
        return None



    def get_realsense_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        # crop sides of image to make it a square
        color_image = color_image[:, 80:560]
        return color_image


    def observe(self):
        print("in obs")

        # # image from esp32 cameras
        cp0 = time.time()
        # frame = self.get_frame()  
        frame = self.get_image_from_usb()
        cp1 = time.time()
        # frame = cv2.resize(frame, (self.rl_image_size, self.rl_image_size))

        if self.cfg.show_camera:
            image = frame
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow("ESP32 Image", image)
            cv2.waitKey(1)
        cp2 = time.time()

        # # image from realsense camera
        # real_frame = self.get_realsense_frame()
        # cp4= time.time()
        # real_frame = cv2.resize(real_frame, (self.rl_image_size, self.rl_image_size))
        
        # if self.cfg.show_camera:
        #     real_image = real_frame
        #     real_image = cv2.cvtColor(real_image, cv2.COLOR_RGB2BGR)
        #     cv2.imshow("RealSense Image", real_image)
        #     cv2.waitKey(1)

        current_jp, current_ee = self.controller.read_ee_and_joint_positions()
        
        props ={
            "ee_pos": current_ee,
            "joint_pos": current_jp,
            "gripper_qpos": [0.0]
        }

        cp3 = time.time()
        rl_obs = {}
        # print mode
        # print(f"Run Mode: {self.run_mode}")
        if self.run_mode == "testbc":
            # print(f"Observing in testbc")

            PROP_KEYS = PROP_KEYS_EE if self.action_type[:2] == "ee" else PROP_KEYS_JP

            prop = torch.from_numpy(
            np.concatenate([props[prop_key] for prop_key in PROP_KEYS]).astype(np.float32)
            )
            assert prop.size(0) == self.prop_shape[0], f"{prop.size(0)=}, {self.prop_shape[0]=}"

            # first append, then concat
            self.past_obses["prop"].append(prop)
            rl_obs["state"] = utils.concat_obs(
                len(self.past_obses["prop"]) - 1, self.past_obses["prop"], self.prop_stack
            ).to(self.device)
            
            cp4 = time.time()
            key = "eye_in_hand"
            rl_image_obs = torch.from_numpy(frame).permute([2, 0, 1])

            # first append, then concat
            self.past_obses[key].append(rl_image_obs)
            rl_obs[key] = utils.concat_obs(
                len(self.past_obses[key]) - 1, self.past_obses[key], self.obs_stack,
            ).to(self.device)

            cp5 = time.time()
            # print(f"Time taken to observe: {cp1-cp0}, {cp2-cp1}, {cp3-cp2}, {cp4-cp3}, {cp5-cp4}")
        
        
        if self.run_mode == "teleop" or self.run_mode == "replay":
            rl_obs["ee_pos"] = props["ee_pos"]
            rl_obs["joint_pos"] = props["joint_pos"]
            rl_obs["frame"] = frame
            cp4 = time.time()
            # rl_obs["real_frame"] = real_frame
            # print(f"Time taken to observe: {cp1-cp0}, {cp2-cp1},{cp3-cp2},{cp4-cp3}")


        return rl_obs
    


    def get_reward_terminal(self):

        self.terminal = False
        reward = 0
        if self.time_step >= self.episode_length:
            self.terminal = True
            reward = 0
        return self.terminal, reward


    # def apply_action(self, action: torch.Tensor):

    #     if isinstance(action, np.ndarray):
    #         targetangles = self.controller.update_ee_position(action)
    #     else:
    #         targetangles = self.controller.update_ee_position(action.numpy())
    #     self.time_step += 1
    #     return targetangles


    def reset(self) -> tuple[dict[str, torch.Tensor]]:
        # self.controller.goto_home()
        self.past_obses.clear()

        self.time_step = 0
        self.episode_reward = 0
        self.terminal = False
        self.video_frames = []

        self.time_step = 0
        self.Terminal = False

        rl_obs = self.observe()
        return rl_obs
    

    def step(
        self, action: torch.Tensor, joint_angles: bool = False, cp_1: float = 0.0
    ) -> tuple[dict[str, torch.Tensor], float, bool, bool, dict[str, torch.Tensor]]:
        self.time_step += 1
        if isinstance(action, torch.Tensor):
            action = action.numpy() # for greater good
        cp1 = time.time()

        # aoo;y action only in testbc and replay mode
        if self.run_mode == "replay" or self.run_mode == "testbc":
            if self.action_type == "ee_delta":
                assert action.size == 3, f"Expected action size of 3, but got {action.size}"
                self.controller.update_ee_position(action)

            elif self.action_type == "ee_abs":
                assert action.size == 3, f"Expected action size of 3, but got {action.size}"
                self.controller.set_ee_position(action)

            elif self.action_type == "jp_delta":
                assert action.size == 6, f"Expected action size of 6, but got {action.size}"
                self.controller.update_joint_position(action)
            
            elif self.action_type == "jp_abs":
                assert action.size == 6, f"Expected action size of 6, but got {action.size}"
                self.controller.set_joint_position(action)

        if self.run_mode == "teleop" and self.teleop_mode == "passive":
                active_pos = np.array(self.passive_controller.robot.read_position()) - self.actpas_offset
                self.controller.set_joint_position(active_pos, encoder=True)
                print(f"Active Pos: {active_pos}")
        cp2 = time.time()


        if self.run_mode == "testbc":
            time.sleep(max(0, self.step_delay - (time.time() - cp_1)))
        
        cp3 = time.time()
        rl_obs = self.observe()
        cp4 = time.time()
 
        if self.run_mode == "teleop" or self.run_mode == "replay":
            time.sleep(max(0, self.step_delay - (time.time() - cp_1)))
            cp5 = time.time()

        terminal, reward = self.get_reward_terminal()

        action_apply_time = cp2-cp1
        obs_time = cp4-cp3
        cp3 = cp5 if self.run_mode == "teleop" else cp3
        return rl_obs, reward, terminal, False, {}, action_apply_time, obs_time, cp3



# ------------------------------------------------------------------------------------- Main Function

if __name__ == "__main__":

    url = "http://192.168.31.187:81/stream"
    # Set your desired FPS
    desired_fps = 1  # Change this value to set how many frames per second to save
    frame_interval = 1 / desired_fps  # Interval between frames in seconds

    # Open the video stream
    cap = cv2.VideoCapture(url)
    
    # Check if the video stream is opened successfully
    if not cap.isOpened():
        print("Error: Unable to open video stream")
        exit()
    
    frame_count = 0  # Frame counter for unique filenames
    last_saved_time = time.time()  # Initialize the time of the last saved frame












# ---------------------------------------------- NOTES ----------------------------------------------

    # def step(
    #     self, action: torch.Tensor, joint_angles: bool = False, cp_1: float = 0.0
    # ) -> tuple[dict[str, torch.Tensor], float, bool, bool, dict[str, torch.Tensor]]:
    #     self.time_step += 1

    #     cp1 = time.time()

    #     # aoo;y action only in testbc and replay mode
    #     if self.run_mode == "testbc":
    #         goal_ee_pos = self.apply_action(action)
    #         cp2 = time.time()
    #         time.sleep(max(0, self.step_delay - (time.time() - cp_1 + 0.08)))
    #         cp3 = time.time()
    #         rl_obs = self.observe()
    #         cp4 = time.time()

    #     terminal, reward = self.get_reward_terminal()
        

    #     action_apply_time = cp2-cp1
    #     obs_time = cp4-cp3
    #     return rl_obs, reward, terminal, False, {}, action_apply_time, obs_time