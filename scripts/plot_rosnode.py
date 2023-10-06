
# VINS PROCESS: vins_node
# LOOP PROCESS: loop_fusion_node


import rospy
import datetime
import cv2
import io

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

VINS_POSE_TOPIC  = '/vins_estimator/camera_pose'
CAMERA_TOPIC     = '/camera/image_mono'

class OdometryPlot():
    def __init__(self, ground_truth):
        # Ground truth should be a np.array like [x, y, z, timestamp]
        self.last_pose = 0
        self.counter = 0
        self.COUNTER_SKIP = 15

        self.ground_truth = ground_truth
        self.plot_publisher = rospy.Publisher('odometry_plot', Image, queue_size=10)
        self.stamped_poses = np.empty((0, 4))
        self.diff = np.array([])
        self.rmse = np.array([])
        self.stamps = np.array([])

        if self.ground_truth != None:
            self.plotx_min, self.plotx_max = np.min(self.ground_truth[0, :]), np.max(self.ground_truth[0, :])
            self.ploty_min, self.ploty_max = np.min(self.ground_truth[1, :]), np.max(self.ground_truth[1, :])
            self.plotz_min, self.plotz_max = np.min(self.ground_truth[2, :]), np.max(self.ground_truth[2, :])
        else:
            self.plotx_min, self.plotx_max = -5, 5
            self.ploty_min, self.ploty_max = -5, 5
            self.plotz_min, self.plotz_max = -5, 5

        self.fig = plt.figure(figsize=(10, 5))
        self.ax = self.fig.add_subplot(121, projection='3d')
        self.rmse_ax = self.fig.add_subplot(122)
        self.make_plot()
        
        self.bridge = CvBridge()
        self.last_ros_image = self.bridge.cv2_to_imgmsg(np.zeros((100, 100, 3), np.uint8), encoding='rgb8')

    def make_plot(self):
        
       
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.rmse_ax.set_xlabel('Time')
        self.rmse_ax.set_ylabel('RMSE')
        
        self.ax.set_xlim(self.plotx_min, self.plotx_max)
        self.ax.set_ylim(self.ploty_min, self.ploty_max)
        self.ax.set_zlim(self.plotz_min, self.plotz_max)

        self.ax.set_title('Odometry Plot')
        self.ax.legend()

        self.rmse_ax.set_title('RMSE')
        self.rmse_ax.legend()



    def estimate_ground_truth(self, stamp):
        if self.ground_truth is None:
            return None
        # Get the closest ground truth pose
        # TODO: Better estimation
        closest = np.min(np.abs(self.ground_truth[3, :] - stamp))
        return closest
    
    def plot_odometry(self):
        # Plots a Image type message in a topic
        if len(self.stamped_poses) == 0:
            rospy.logwarn("No Poses!")
            self.plot_publisher.publish(self.last_ros_image)
            return 
        if self.last_pose >= len(self.stamped_poses):
            self.plot_publisher.publish(self.last_ros_image)
            return
        # Get the current pose
        pose = self.stamped_poses[self.last_pose]
        self.last_pose += 1
        rospy.loginfo("Processing frame {}/{}...".format(self.last_pose, len(self.stamped_poses)))
        # Get the ground truth pose
        ground_truth = self.estimate_ground_truth(pose[3])
        if ground_truth is None:
            ground_truth = pose
        self.diff = np.append(self.diff, np.linalg.norm(pose[0:3] - ground_truth[0:3]))
        self.rmse = np.append(self.rmse, np.sqrt(np.mean(self.diff**2)))
        self.stamps = np.append(self.stamps, pose[3])

        if self.counter % self.COUNTER_SKIP != 0:
            self.counter += 1
            return 
        else:
            rospy.loginfo("Plotting...")
        self.counter += 1

        # Plot the current pose
        self.ax.clear()
        self.rmse_ax.clear()
        self.make_plot()
        self.ax.scatter(self.stamped_poses[:self.last_pose, 0], 
                        self.stamped_poses[:self.last_pose, 1], 
                        self.stamped_poses[:self.last_pose, 2], c='r', marker='o', label = "Estimated", s = 1)
        self.rmse_ax.plot(self.stamps, self.rmse, c='r', label = "RMSE")

        # Plot the ground truth pose
        if self.ground_truth is not None:
            self.ax.scatter(ground_truth[0], ground_truth[1], ground_truth[2], c='b', marker='o', label = "Ground Truth")
        self.fig.legend()
        plt.tight_layout()

        #Convert fig to image
        self.fig.canvas.draw()
        with io.BytesIO() as buff:
            self.fig.savefig(buff, format='raw')
            buff.seek(0)
            data = np.frombuffer(buff.getvalue(), dtype=np.uint8)
        w, h = self.fig.canvas.get_width_height()
        image = data.reshape((int(h), int(w), -1))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')

        self.last_ros_image = ros_image
        self.plot_publisher.publish(ros_image)

    def vins_pose_callback(self, msg):
        position = msg.pose.pose.position
        stamp = msg.header.stamp.nsecs
        self.stamped_poses = np.vstack((self.stamped_poses, np.array([position.x, position.y, position.z, stamp])))
# ------------------------------ PLOT ------------------------------
def run_node():
    rospy.init_node('odometry_plot')
    rospy.loginfo('Starting odometry plot node!')
    # Initialize the class
    odometry_plot = OdometryPlot(None)

    # Starts to read from the vins fusion topic
    rospy.Subscriber(VINS_POSE_TOPIC, Odometry, odometry_plot.vins_pose_callback)
    while not rospy.is_shutdown():
        odometry_plot.plot_odometry()
    



if __name__ == '__main__':
    run_node()
