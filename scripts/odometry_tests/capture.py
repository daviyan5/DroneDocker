import numpy as np
import rospy as rp
import cv2
import psutil
import os
import sys
import time
import keyboard
import shlex 
from subprocess import Popen, PIPE, STDOUT, DEVNULL
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


ROSBAG_DURATION          = 35 # s
LIMIT = 4 # m / s
VINS_POSE_TOPIC          = '/vins_estimator/camera_pose'
ROSBAG_CAMERA            = '/camera/image_mono'
odometry_command         = "gnome-terminal --tab -e 'bash -c \"sh scripts/odometry.sh vins\"'"
vins_fusion_process_name = "vins_node"
loop_fusion_process_name = "loop_fusion_node"
rosbag_path              = "./datasets/TesteDef.bag"

class OdometryCapture:
    def __init__(self, loop = False):
        self.odometry_command = odometry_command + " --tab -e 'bash -c \"sleep 1 ; sh scripts/odometry.sh loop\"'" if loop else odometry_command
        self.timestamps = np.array([])
        self.poses = np.empty((0, 3))
        self.cpu_usage = np.array([])
        self.memory_usage = np.array([])
        self.sub = None
        self.fail_flag = 0

    def run(self, command_bar, roscore_process):
        # Start the odometry process
        odometry_process = None
        rosbag_process   = None
        odometry_log     = open("./scripts/odometry_tests/logs/odometry.log", "w+")
        rosbag_log       = open("./scripts/odometry_tests/logs/rosbag.log", "w+")
        self.odometry_psutil = None
        self.odometry_pid = None
        self.loop_pid = None
        try:
            if roscore_process.poll() is None:
                # Start ROS Node
                rp.init_node('odometry_capture', anonymous=True)
                
            
                command_bar.set_description("Starting Odometry Process...")
                odometry_process = Popen(self.odometry_command, shell=True, stdout=odometry_log, stderr=STDOUT)
                command_bar.update(5)
                time.sleep(2)
                # Subscribe to topics
                
                self.sub = rp.Subscriber(VINS_POSE_TOPIC, Odometry, self.vins_pose_callback, queue_size=1)
                for proc in psutil.process_iter():
                    if proc.name() == vins_fusion_process_name:
                        self.odometry_pid = proc.pid
                    if proc.name() == loop_fusion_process_name:
                        self.loop_pid = proc.pid
                self.odometry_psutil = psutil.Process(self.odometry_pid) if odometry_process is not None else None
                self.loop_psutil = psutil.Process(self.loop_pid) if self.loop_pid is not None else None
                
                command_bar.set_description("Playing Rosbag...")
                rosbag_process = Popen(shlex.split("rosbag play " + rosbag_path), shell=False, stdout=rosbag_log, stderr=STDOUT)
                command_bar.update(5)
                
                
                denom = 90 / ROSBAG_DURATION
                i = 0
                while rosbag_process.poll() is None:
                    time.sleep(1)
                    command_bar.set_description("Saving measurements..." + "len(timestamps) = " + str(len(self.timestamps)))
                    command_bar.update(denom) if i < ROSBAG_DURATION else None
                    i += 1
                if not self.check_odometry_speed():
                    command_bar.set_description("Speed Test Fail!")
                    time.sleep(1)
                    self.fail_flag = 1
                else:
                    command_bar.set_description("Speed Test Passed!")
                    time.sleep(1)
                command_bar.set_description("Ending...")
                odometry_process.terminate() if odometry_process != None else None
                rosbag_process.terminate() if rosbag_process != None else None
                odometry_log.close()
                rosbag_log.close()
                self.odometry_psutil.terminate() if self.odometry_psutil != None else None
                self.sub.unregister()
                del self.sub
                time.sleep(1)
                
                return (self.timestamps, self.cpu_usage, self.memory_usage, self.poses) if self.fail_flag == 0 else None
            else:
                raise Exception("Roscore process terminated")
        except Exception as e:
            command_bar.set_description("Ending Processes..." + str(e))
            odometry_process.terminate() if odometry_process != None else None
            rosbag_process.terminate() if rosbag_process != None else None
            self.odometry_psutil.terminate() if self.odometry_psutil != None else None
            if self.sub != None:
                self.sub.unregister()
                del self.sub
            odometry_log.close()
            rosbag_log.close()
            raise e
    
    def vins_pose_callback(self, data):
        position = data.pose.pose.position
        stamp = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        self.timestamps = np.append(self.timestamps, stamp)
        self.poses      = np.append(self.poses, [[position.x, position.y, position.z]], axis=0)
        
        # Use psutil to monitor CPU usage
        self.cpu_usage      = np.append(self.cpu_usage, self.odometry_psutil.cpu_percent() + (self.loop_psutil.cpu_percent() if self.loop_pid != None else 0))
        self.memory_usage   = np.append(self.memory_usage, self.odometry_psutil.memory_percent() + (self.loop_psutil.memory_percent() if self.loop_pid != None else 0))
        
    
    def check_odometry_speed(self):
        if len(self.timestamps) < 10:
            return True
        h = np.diff(self.timestamps)
        # Get indices of h = 0
        non_zero_indices = h <= 1e-10
        h[non_zero_indices] = 1e-10
        fx = self.poses[:, 0]
        fy = self.poses[:, 1]
        fz = self.poses[:, 2]
        df_dx = (fx[1:] - fx[:-1]) / h
        df_dy = (fy[1:] - fy[:-1]) / h
        df_dz = (fz[1:] - fz[:-1]) / h
        # Speed is the norm of the derivative
        speed = np.sqrt(df_dx**2 + df_dy**2 + df_dz**2)
        if np.any(speed > LIMIT):
            return False
        return True
        
