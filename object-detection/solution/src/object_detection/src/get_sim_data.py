#!/usr/bin/env python3

import io
import time
import cv2
import geometry

import numpy as np
from PIL import Image
import rospy
from aido_schemas import (
    Context,
    DB20Commands,
    DB20Observations,
    EpisodeStart,
    GetCommands,
    LEDSCommands,
    protocol_agent_DB20,
    PWMCommands,
    RGB,
    wrap_direct,
    DTSimRobotState,
    InteractionProtocol,
    RobotState,
    protocol_agent_DB20_fullstate,
    DB20ObservationsPlusState,
    JPGImage
)

from typing import Any
from zuper_commons.logs import setup_logging, zlogger
from zuper_commons.logs import ZLogger
from rosagent import ROSAgent
from zuper_nodes import particularize_no_check, particularize
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu

# XXX this is not implemented yet
protocol_scorer = InteractionProtocol(
    description="""Protocol for scorer""",
    language="""\
            in:robot_state
        """,
    inputs={
        "robot_state": Any,
    },
    outputs={},
)

protocol_agent_b = InteractionProtocol(
    description="""

Generic protocol for an agent that receives "observations" and responds 
with "commands".

"episode_start" marks the beginning of an episode.  

    """.strip(),
    inputs={"observations": Any, "seed": int, "get_commands": GetCommands, "episode_start": EpisodeStart, "robot_state": Any},
    outputs={"commands": Any},
    language="""
            in:seed? ;
            (   in:episode_start ; 
                (in:observations | 
                 in:robot_state |
                    (in:get_commands ; out:commands)
                 )* 
            )*
        """,
)

protocol_agent_DB20__b = particularize_no_check(
    protocol_scorer,
    description="""Particularization for DB20 b observations and commands.""",
    inputs={"robot_state": DTSimRobotState}
)

protocol_agent_DB20_c = particularize(
    protocol_agent_b,
    description="""Particularization for DB20 observations and commands.""",
    inputs={"observations": DB20Observations,
    "robot_state": DTSimRobotState},
    outputs={"commands": DB20Commands},
)

class ROSTemplateAgent:
    def __init__(self):
        from cv_bridge import CvBridge
        # Start the ROSAgent, which handles publishing images and subscribing to action
        self.agent = ROSAgent()
        self.lastTime = None
        self.lastRoboInfo = None
        self.theta_prev = 0
        self.x_prev = 0
        self.y_prev = 0
        self.last_axis_left_rad = 0
        self.last_axis_right_rad = 0
        self.last_v = 0

        self.cam0Data = []

        self.bridge = CvBridge()

        '''
        import sys
        print('ash: loading data')
        with open('/code/solution/src/object_detection/src/mav0/cam0/data.csv') as file:
            
            import os,cv2
            
            print(file.readline())

            lines = file.readlines()

            for line in lines:
                tokens = line.split(',')
                entry = {}
                entry["timestamp"] = float(tokens[0])
                entry["image"] = tokens[1].strip()
                imageFile = os.path.join('/code/solution/src/object_detection/src/mav0/cam0/data',entry["image"])
                #print(imageFile)
                img = cv2.imread(imageFile, cv2.IMREAD_COLOR)
                #print(img)
                entry["compressed"] = self.bridge.cv2_to_compressed_imgmsg(img)
                self.cam0Data.append(entry)

        self.imu0Data = []
        #timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
        with open('/code/solution/src/object_detection/src/mav0/imu0/data.csv') as file:
            print(file.readline())

            lines = file.readlines()

            for line in lines:
                tokens = line.split(',')
                entry = {}
                entry["timestamp"] = float(tokens[0])
                entry["w_RS_S_x"] = float(tokens[1])
                entry["w_RS_S_y"] = float(tokens[2])
                entry["w_RS_S_z"] = float(tokens[3])
                entry["a_RS_S_x"] = float(tokens[4])
                entry["a_RS_S_y"] = float(tokens[5])
                entry["a_RS_S_z"] = float(tokens[6])
                self.imu0Data.append(entry)

        self.cam0DataIndex = 0
        self.imu0DataIndex = 0

        with open('/code/solution/src/object_detection/src/sim0/cam0/data.csv','w') as f:
            f.write('#timestamp [ns],filename\n')

        with open('/code/solution/src/object_detection/src/sim0/imu0/data.csv','w') as f:
            f.write('#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n')
        print('ash: Done loading data')

        '''

    def init(self, context: Context):
        context.info("init()")

    #def on_received_robot_state(self,  context: Context, data: DTSimRobotState):
       #context.info("state %s." % data)
    #   print("ash: state")

    def on_received_seed(self, context: Context, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info("Starting episode %s." % data.episode_name)
        yaml_payload = getattr(data, 'yaml_payload', '{}')
        self.agent._publish_episode_start(data.episode_name, yaml_payload)

    def jpg2rgb(self, image_data: bytes) -> np.ndarray:
        """ Reads JPG bytes as RGB"""

        im = Image.open(io.BytesIO(image_data))
        im = im.convert("RGB")
        data = np.array(im)
        assert data.ndim == 3
        assert data.dtype == np.uint8
        return data

    def write_cam_line(self, file, timestamp):
        with open('/code/solution/src/object_detection/src/sim0/cam0/data.csv','a') as f:
            f.write(f'{timestamp},{file}\n')

    def write_imu_line(self, imu, pos, v):
        #timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
        with open('/code/solution/src/object_detection/src/sim0/imu0/data.csv','a') as f:
            f.write(f'{imu.header.stamp},{imu.angular_velocity.x},{imu.angular_velocity.y},{imu.angular_velocity.z},{imu.linear_acceleration.x},'
                f'{imu.linear_acceleration.y},{imu.linear_acceleration.z},{pos[0]},{pos[1]},{v[0]},{v[1]}\n')

    def on_received_observations(self, context: Context, data: DB20ObservationsPlusState):
        roboInfo = None
        camera: JPGImage = data.camera
        rosTime = rospy.get_rostime()
        currentTime = data.state.t_effective
        print(f'ash: effective time {currentTime}, cam time {data.camera.timestamp} odo time {data.odometry.timestamp}')
        for duckie, simState in data.state.duckiebots.items():
            print(f'ash: {duckie} with state {simState}')
            roboInfo = simState
        jpg_data = data.camera.jpg_data
        #jpg_data = self.cam0Data[self.cam0DataIndex]["compressed"].data

        imageFile = f'/code/solution/src/object_detection/src/sim5/cam0/data/{int(currentTime*1000000000)}.png'
        cv2.imwrite(imageFile, cv2.cvtColor(self.jpg2rgb(camera.jpg_data), cv2.COLOR_BGR2RGB))
        self.write_cam_line(imageFile, currentTime*1000000000)
        #currentTimeOrg = self.cam0Data[self.cam0DataIndex]["timestamp"]
        #currentTime = currentTimeOrg/1000000000
        #self.cam0DataIndex += 1
        self.agent._publish_img(jpg_data, currentTime)
        self.agent._publish_info()
        odometry = data.odometry
        self.agent._publish_odometry(
            odometry.resolution_rad,
            odometry.axis_left_rad,
            odometry.axis_right_rad
        )

        cp, angle = geometry.translation_angle_from_SE2(roboInfo.pose)
        print(f'ash: pose trans {cp} angle {angle}')

        v, omega = geometry.linear_angular_from_se2(roboInfo.velocity)

        currentTime = data.state.t_effective
        if self.lastTime is None:
            delta_t = 0.05
        else:
            delta_t = (currentTime-self.lastTime)
        if self.lastRoboInfo:
            accel = (v - self.last_v)/delta_t
            #print(f'ash: accel {accel}')

            R = 0.0318 # for the sake of this unit test, keep these values, regardless of what you have measured.
            baseline_wheel2wheel = 0.1
            R_left = R # * (1-r)
            R_right = R # * (1+r)
            
            # Define distance travelled by each wheel [m]
            print(f"ash: {data.odometry}")
            d_left = R_left * (self.last_axis_left_rad - data.odometry.axis_left_rad)
            d_right = R_right * (self.last_axis_right_rad - data.odometry.axis_right_rad )
            
            # Define distance travelled by the robot, in body frame [m]
            
            d_A = (d_left + d_right)/2
            
            # Define rotation of the robot [rad]
            
            Dtheta = (d_right - d_left)/baseline_wheel2wheel
            
            # Define distance travelled by the robot, in world frame [m]
            
            Dx = d_A * np.cos(self.theta_prev)
            Dy = d_A * np.sin(self.theta_prev)
            
            # Update pose estimate
            
            x_curr = self.x_prev + Dx
            y_curr = self.y_prev + Dy
            theta_curr = self.theta_prev + Dtheta

            self.theta_prev = theta_curr
            self.x_prev = x_curr
            self.y_prev = y_curr
        else:
            accel = np.array([0,0])
            Dtheta = 0

        angular_velocity = omega

        print(f"ash: t={currentTime} using angular velocity {angular_velocity} calculated {Dtheta}, linear acc {accel}, v {v}")

        imu = Imu()
        #imu.header.frame_id = f"{self.frame_id}"
        imu.header.stamp = currentTime*1000000000
        imu.linear_acceleration.x = accel[0]
        imu.linear_acceleration.y = 0
        imu.linear_acceleration.z = accel[1]
        imu.angular_velocity.x = angular_velocity
        imu.angular_velocity.y = 0
        imu.angular_velocity.z = 0

        #self.write_imu_line(imu, cp, v)

        '''
        old code
        
        imu.linear_acceleration.x = accel[0]
        imu.linear_acceleration.y = 0
        imu.linear_acceleration.z = accel[1]
        imu.angular_velocity.x = angular_velocity
        imu.angular_velocity.y = 0
        imu.angular_velocity.z = 0
        '''
        '''
        while self.imu0Data[self.imu0DataIndex]["timestamp"]<=currentTimeOrg:
            imuData = self.imu0Data[self.imu0DataIndex]
            self.agent._publish_imu(imuData["timestamp"]/1000000000, [imuData["w_RS_S_x"], imuData["w_RS_S_y"], imuData["w_RS_S_z"]], 
                    [imuData["a_RS_S_x"], imuData["a_RS_S_y"], imuData["a_RS_S_z"]])
            self.imu0DataIndex += 1
        '''

        self.lastRoboInfo = roboInfo
        self.lastTime = currentTime
        self.last_axis_left_rad = data.odometry.axis_left_rad
        self.last_axis_right_rad = data.odometry.axis_right_rad
        self.last_v = v


    def on_received_get_commands(self, context: Context, data: GetCommands):
        if not self.agent.initialized:
            pwm_left, pwm_right = [0, 0]
        else:
            # TODO: let's use a queue here. Performance suffers otherwise.
            # What you should do is: *get the last command*, if available
            # otherwise, wait for one command.
            while not self.agent.updated:
                time.sleep(0.01)

            pwm_left, pwm_right = self.agent.action
            self.agent.updated = False

        grey = RGB(0.5, 0.5, 0.5)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = DB20Commands(pwm_commands, led_commands)

        context.write("commands", commands)

    def finish(self, context):
        context.info("finish()")
        rospy.signal_shutdown("My job here is done.")


if __name__ == "__main__":
    setup_logging()
    #ZLogger.setLevel(ZLogger.DEBUG)
    from logging import Logger as logger
    import logging
    logging.getLogger('aido_schemas').setLevel(logging.DEBUG)
    logging.getLogger('nodes').setLevel(logging.DEBUG)
    logging.getLogger('nodes_wrapper').setLevel(logging.DEBUG)
    logging.getLogger('ipce').setLevel(logging.DEBUG)
    #logger.
    print("starting my node")
    node = ROSTemplateAgent()
    #node = None
    protocol = protocol_agent_DB20_fullstate
    wrap_direct(node=node, protocol=protocol, args=[])
    print("ash:  exiting main")
