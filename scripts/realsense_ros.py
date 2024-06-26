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

    def run(self, img, num_clicks=1):
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

class RealSenseROS:
    def __init__(self):
        self.bridge = CvBridge()

        self.camera_lock = Lock()
        self.camera_header = None
        self.camera_color_data = None
        self.camera_info_data = None
        self.camera_depth_data = None

        queue_size = 1000

        self.color_image_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size= queue_size, buff_size = 65536*queue_size)
        self.camera_info_sub = message_filters.Subscriber("/camera/color/camera_info", CameraInfo, queue_size= queue_size, buff_size = 65536*queue_size)
        self.depth_image_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size= queue_size, buff_size = 65536*queue_size)
        ts_top = message_filters.TimeSynchronizer([self.color_image_sub, self.camera_info_sub, self.depth_image_sub], queue_size= queue_size)
        ts_top.registerCallback(self.rgbdCallback)
        ts_top.enable_reset = True

        self.point_visualization_pub = rospy.Publisher('point_visualization_marker_array', MarkerArray, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        time.sleep(1.0)

    def rgbdCallback(self, rgb_image_msg, camera_info_msg, depth_image_msg):

        try:
            # Convert your ROS Image message to OpenCV2
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, "32FC1")
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
    
    def visualize_point(self, transform, id = 0):

        # publish a cube marker
        marker_array = MarkerArray()
        marker = Marker()
        marker.id = id
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "camera_color_optical_frame"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0

        # marker color is red
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        pose = self.get_pose_msg_from_transform(transform)
        marker.pose = pose

        marker_array.markers.append(marker)

        self.point_visualization_pub.publish(marker_array)

if __name__ == "__main__":
    rospy.init_node('RealSenseROS')
    rs_ros = RealSenseROS()
    pixel_selector = PixelSelector()
    
    while True:    

        header, color_data, info_data, depth_data = rs_ros.get_camera_data()    
        pixel = pixel_selector.run(color_data)
        print("Pixel: ",pixel)

        valid, world_coordinate = rs_ros.pixel2World(info_data, pixel[0][0], pixel[0][1], depth_data)
        
        if valid:
            point_transform = np.eye(4)
            point_transform[:3,3] = world_coordinate.reshape(1,3)
        
            rs_ros.visualize_point(point_transform)
            print("World Coordinate: ", world_coordinate)

            pose = PoseStamped()

            pose.pose = rs_ros.get_pose_msg_from_transform(point_transform)
            pose.header.frame_id = "camera_color_optical_frame"
            pose.header.stamp = rospy.Time.now()

            try:
                transform = rs_ros.tf_buffer.lookup_transform("panda_link0",
                                           "camera_color_optical_frame",
                                           rospy.Time.now(),
                                           rospy.Duration(1.0))
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                print("Successfully transformed. The coordinates in ", pose_transformed.header.frame_id, " is: ", [pose_transformed.pose.position.x, pose_transformed.pose.position.y, pose_transformed.pose.position.z])
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as ex:
                rospy.logerr('Transformation error occurred: %s', ex)
        else:
            print("Pixel selected has invalid depth :(")

        input("Press [ENTER] to move to try again ...")
        cv2.destroyAllWindows()