import numpy as np
import math
import rospy
import time
import cv2
from threading import Lock

# ros imports
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import tf2_geometry_msgs

class PixelSelector:
    def __init__(self):
        pass

    def load_image(self, img):
        self.img = img

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.clicks.append([x, y])
            cv2.circle(self.img, (x, y), 3, (200, 0, 0), -1)
            cv2.imshow("pixel_selector", self.img)

    def run(self, img, num_clicks=2):
        self.load_image(img)
        self.clicks = []
        cv2.namedWindow('pixel_selector')
        cv2.setMouseCallback('pixel_selector', self.mouse_callback)
        while True:
            cv2.imshow("pixel_selector", self.img)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:  # Escape key
                break
            if len(self.clicks) >= num_clicks:
                break
        return self.clicks

class VisualFeedback_Azure:
    def __init__(self):
        self.bridge = CvBridge()

        self.camera_lock = Lock()
        self.camera_header = None
        self.camera_color_data = None
        self.camera_info_data = None
        self.camera_depth_data = None

        queue_size = 1000

        self.color_image_sub = message_filters.Subscriber("/rgb/image_raw", Image, queue_size= queue_size, buff_size = 65536*queue_size)
        self.camera_info_sub = message_filters.Subscriber("/rgb/camera_info", CameraInfo, queue_size= queue_size, buff_size = 65536*queue_size)
        self.depth_image_sub = message_filters.Subscriber("/depth_to_rgb/image_raw", Image, queue_size= queue_size, buff_size = 65536*queue_size)
        ts_top = message_filters.ApproximateTimeSynchronizer([self.color_image_sub, self.camera_info_sub, self.depth_image_sub], queue_size= queue_size, slop=0.1)
        ts_top.registerCallback(self.rgbdCallback)
        ts_top.enable_reset = True

        self.point_visualization_pub = rospy.Publisher('/visual_feedback/azure/push_vector', MarkerArray, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        time.sleep(1.0)

    def rgbdCallback(self, rgb_image_msg, camera_info_msg, depth_image_msg):
        # print("Entered rgbdCallback!")
        try:
            # Convert your ROS Image message to OpenCV2
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, "16UC1")
        except CvBridgeError as e:
            print(e)

        with self.camera_lock:
            self.camera_header = rgb_image_msg.header
            self.camera_color_data = rgb_image
            self.camera_info_data = camera_info_msg
            self.camera_depth_data = depth_image

    def get_camera_data(self):
        with self.camera_lock:
            return self.camera_header, self.camera_color_data, self.camera_info_data, self.camera_depth_data
    
    def pixel2World(self, camera_info, image_x, image_y, depth_image, box_width = 2):

        if image_y >= depth_image.shape[0] or image_x >= depth_image.shape[1]:
            return False, None

        depth = depth_image[image_y, image_x]

        if math.isnan(depth) or depth < 50 or depth > 2000:

            depth = []
            for i in range(-box_width,box_width):
                for j in range(-box_width,box_width):
                    if image_y+i >= depth_image.shape[0] or image_x+j >= depth_image.shape[1]:
                        return False, None
                    pixel_depth = depth_image[image_y+i, image_x+j]
                    if not (math.isnan(pixel_depth) or pixel_depth < 50 or pixel_depth > 2000):
                        depth += [pixel_depth]

            if len(depth) == 0:
                return False, None

            depth = np.mean(np.array(depth))

        depth = depth/1000.0 # Convert from mm to m

        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]  

        # Convert to world space
        world_x = (depth / fx) * (image_x - cx)
        world_y = (depth / fy) * (image_y - cy)
        world_z = depth

        return True, np.array([world_x, world_y, world_z])

    def world2Pixel(camera_info, world_x, world_y, world_z):

        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]  

        image_x = world_x * (fx / world_z) + cx
        image_y = world_y * (fy / world_z) + cy

        return image_x, image_y
    
    def get_pose_msg_from_transform(self, transform):

        pose = Pose()
        pose.position.x = transform[0,3]
        pose.position.y = transform[1,3]
        pose.position.z = transform[2,3]

        quat = Rotation.from_matrix(transform[:3,:3]).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose
    
    def visualize_point(self, pose, id = 0):

        # publish a cube marker
        marker_array = MarkerArray()
        marker = Marker()
        marker.id = id
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "panda_link0"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0

        # marker color is red
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5

        # pose = self.get_pose_msg_from_transform(transform)
        marker.pose = pose

        marker_array.markers.append(marker)

        self.point_visualization_pub.publish(marker_array)

    def fixed_policy(self, pose_array):

        # Fixed policy : x, y are unchanged. z changes
        # pose.position.x += 0.1
        # pose.position.y += 0.1

        pose_init = pose_array[0].pose
        pose_fin = pose_array[1].pose

        roll = 0
        pitch = 0
        yaw = math.atan2(pose_fin.position.y - pose_init.position.y, pose_fin.position.x - pose_init.position.x)

        print("Yaw (deg): ", yaw * 180 / math.pi)

        quat = Rotation.from_euler('zyx', [yaw, pitch, roll]).as_quat()

        pose_policy = Pose()
        pose_policy.position.x = pose_init.position.x
        pose_policy.position.y = pose_init.position.y
        pose_policy.position.z = (pose_init.position.z + pose_fin.position.z) / 2

        pose_policy.orientation.x = quat[0]
        pose_policy.orientation.y = quat[1]
        pose_policy.orientation.z = quat[2]
        pose_policy.orientation.w = quat[3]       #scipy follows scalar-last convention for quaternions (source : https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html)

        return pose_policy

if __name__ == "__main__":
    rospy.init_node('VisualFeedback_Azure')
    azure_ros = VisualFeedback_Azure()
    pixel_selector = PixelSelector()
    
    while True:        
        while True:
            header, color_data, info_data, depth_data = azure_ros.get_camera_data() 
            if not color_data is None:
                break
            else:
                print("Waiting for camera data ...")
                time.sleep(0.1)
        pixel = pixel_selector.run(color_data)
        print("Pixels: ",pixel)

        valid = [False, False]
        world_coordinate = [None, None]

        for i in range(0,2):
            valid[i], world_coordinate[i] = azure_ros.pixel2World(info_data, pixel[i][0], pixel[i][1], depth_data)
        
        print(valid)
        pose_transformed = [PoseStamped(), PoseStamped()]
        if all(valid):
            for i in range(0,2):
                point_transform = np.eye(4)
                point_transform[:3,3] = world_coordinate[i].reshape(1,3)

                # azure_ros.visualize_point(point_transform)
                print("Camera Coordinate: ", world_coordinate[i])

                pose = PoseStamped()
                pose.pose = azure_ros.get_pose_msg_from_transform(point_transform)
                pose.header.frame_id = "rgb_camera_link"
                pose.header.stamp = rospy.Time.now()

                try:
                    transform = azure_ros.tf_buffer.lookup_transform("panda_link0",
                                               "rgb_camera_link",
                                               rospy.Time.now(),
                                               rospy.Duration(1.0))
                    print("Transform: ", transform)
                    pose_transformed[i] = tf2_geometry_msgs.do_transform_pose(pose, transform)
                    print("Successfully transformed point %s", i)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as ex:
                    rospy.logerr('Transformation error occurred: %s', ex)

            
            pose_policy = azure_ros.fixed_policy(pose_transformed)
            azure_ros.visualize_point(pose_policy)
            print("Pose for policy is: ", pose_policy)

        else:
            print("Some pixels selected have invalid depth :(")
            print(valid)

        input("Press [ENTER] to move to try again ...")
        cv2.destroyAllWindows()