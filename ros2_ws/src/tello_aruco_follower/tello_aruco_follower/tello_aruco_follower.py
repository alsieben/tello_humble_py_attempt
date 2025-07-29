import rclpy
import cv2
import numpy as np
import tf2_ros
import ament_index_python
import yaml

from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster


# set up aruco detection stuff
# need to do it this way bc commands changed in version 4.7.0 of opencv (pyimagesearch is outdated)
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters()

# note: ensure calibration plate and (especially) aruco marker are flat so it works properly
# otherwise the rmat and tves will be off and change with bending.



def rotm2quat(rotm):
    r11, r12, r13 = rotm[0, 0], rotm[0, 1], rotm[0, 2]
    r21, r22, r23 = rotm[1, 0], rotm[1, 1], rotm[1, 2]
    r31, r32, r33 = rotm[2, 0], rotm[2, 1], rotm[2, 2]
        
    q = np.empty((4, ))
    q[0] = 0.5*np.sqrt(1+r11+r22+r33)
    q[1] = 0.5*np.sqrt(1+r11-r22-r33)*np.sign(r32-r23)
    q[2] = 0.5*np.sqrt(1-r11+r22-r33)*np.sign(r13-r31)
    q[3] = 0.5*np.sqrt(1-r11-r22+r33)*np.sign(r21-r12)

    return q

def clamp(value, maxval, minval):
    return max(minval, min(value, maxval))

# create aruco follower control node
class ArucoFollowControl(Node):
    def __init__(self):
        super().__init__('aruco_follow_control')
        
        # declare parameter for aruco marker size
        self.declare_parameter('aruco_edge_size_mm', 88.2) # Aruco marker total edge length in mm
        self.aruco_edge_size_mm = float(self.get_parameter('aruco_edge_size_mm').value)

        # declare parameter for aruco marker follow distance
        self.declare_parameter('aruco_follow_distance_mm', 600)
        self.aruco_follow_distance_mm = float(self.get_parameter('aruco_follow_distance_mm').value)

        # declare parameters for limits on linear and angular speed to prevent the drone from going too fast
        self.declare_parameter('drone_max_percent_spd_linear_x', 50)
        self.declare_parameter('drone_max_percent_spd_linear_y', 35)
        self.declare_parameter('drone_max_percent_spd_linear_z', 70)
        self.declare_parameter('drone_max_percent_spd_angular', 50)

        self.max_linear_spd_x = int(self.get_parameter('drone_max_percent_spd_linear_x').value)
        self.max_linear_spd_y = int(self.get_parameter('drone_max_percent_spd_linear_y').value)
        self.max_linear_spd_z = int(self.get_parameter('drone_max_percent_spd_linear_z').value)
        self.max_angular_spd = int(self.get_parameter('drone_max_percent_spd_angular').value)

        # define aruco marker object points: topleft, topright, bottomright, bottomleft (clockwise around marker)
        # marker points are defined based on their relative position to the center, in a right handed coordinate system with the z-axis pointing out of the aruco marker.
        self.edge = self.aruco_edge_size_mm / 2.0
        self.objectPoints = np.array([[-self.edge, self.edge, 0], [self.edge, self.edge, 0], [self.edge, -self.edge, 0], [-self.edge, -self.edge, 0]])

        self.bridge = CvBridge()
        self.detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

        # set up subscriber to get camera feed from drone
        self.aruco_img_sub = self.create_subscription(Image, 'image_raw', self.aruco_detect_callback, 1)

        # set up publishers to:
            # publish images with the tracked aruco markers on them
        self.aruco_img_pub = self.create_publisher(Image, 'image_aruco_det', 1)
            # publish the transform between the aruco marker and the drone camera
        #self.aruco_tf_pub = self.TransformBroadcaster(self)
            # publish the control inputs to the tello drone to make it follow the aruco marker
        self.tello_control_pub = self.create_publisher(Twist, 'control', 1)

        self.filepath = ament_index_python.get_package_share_directory('tello_aruco_follower')
        self.file = self.filepath + '/camera_params.yaml'

        with open(self.file, 'r') as yamlpath:
            self.loaded_dict = yaml.safe_load(yamlpath)

        self.img_width = self.loaded_dict["img_width"]
        self.img_height = self.loaded_dict["img_height"]
        self.camera_matrix = np.asarray(self.loaded_dict["camera_matrix"])
        self.dist_coeffs = np.asarray(self.loaded_dict["dist_coeffs"])

        self.tf_broadcaster = TransformBroadcaster(self)

        self.drone_frame_from_cam_rmat = np.array([[1, 0, 0], [0, 0, 1],[0, -1, 0]])

        # initialize pid controller params
        self.previous_error_pos = 0
        self.previous_target_pos = np.array([0,0,0])
        self.integral_pos = np.array([0.0,0.0,0.0])
        self.previous_error_angle = 0
        self.dt_timesteps = 1

        self.previous_control_msg = Twist()
        self.previous_control_msg.linear.x = 0.0
        self.previous_control_msg.linear.y = 0.0
        self.previous_control_msg.linear.z = 0.0
        self.previous_control_msg.angular.x = 0.0
        self.previous_control_msg.angular.y = 0.0
        self.previous_control_msg.angular.z = 0.0


        # declare pd controller gains as parameters
        self.declare_parameter('linear_kp', 0.21)
        self.linear_kp = float(self.get_parameter('linear_kp').value)
        # self.declare_parameter('linear_ki', 0.15)
        # self.linear_ki = float(self.get_parameter('linear_ki').value)
        self.declare_parameter('linear_kd', 0.285)
        self.linear_kd = float(self.get_parameter('linear_kd').value)
        self.declare_parameter('angular_kp', 350)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.declare_parameter('angular_kd', 150)
        self.angular_kd = float(self.get_parameter('angular_kd').value)
        self.declare_parameter('stop_timeout_steps', 2)
        self.stop_timeout = int(self.get_parameter('stop_timeout_steps').value)
        

    def aruco_detect_callback(self, aruco_img_msg):
        """Detects aruco marker within the image, finds its relative pose to the drone, and publishes control inputs and relevant information."""

        # try to convert it to cv2 image
        try:
            cv2_img =  self.bridge.imgmsg_to_cv2(aruco_img_msg, 'rgb8')
        except CvBridgeError as e:
            self.get_logger().warn(e)

        marker_det, marker_rmat_cam, marker_tvec_cam, transform_cameraframe, aruco_cv2_image_draw = self.aruco_img_detect(cv2_img)

        # publish aruco detection image
        msg = self.bridge.cv2_to_imgmsg(np.array(aruco_cv2_image_draw), 'rgb8')
        msg.header.frame_id = 'drone'
        self.aruco_img_pub.publish(msg)

        # get position and rotation targets (if marker is detected)
        if marker_det:
            drone_pos_targ_mm, drone_zang_targ_rad = self.drone_target_output(marker_rmat_cam, marker_tvec_cam)
            # apply controller
            self.drone_pd_controller(drone_pos_targ_mm, drone_zang_targ_rad)
            # reset dt timesteps to 1 since the drone just got a read
            self.dt_timesteps = 1
        else:
            # increment dt timesteps
            self.dt_timesteps+=1
            # if we don't detect a marker for too long, stop the drone, otherwise, publish the last control message again (ok if going slow, may change this)
            if self.dt_timesteps > self.stop_timeout:
                self.previous_control_msg.linear.x = 0.0
                self.previous_control_msg.linear.y = 0.0
                self.previous_control_msg.linear.z = 0.0
                self.previous_control_msg.angular.x = 0.0
                self.previous_control_msg.angular.y = 0.0
                self.previous_control_msg.angular.z = 0.0
            self.tello_control_pub.publish(self.previous_control_msg)
                


        


    def aruco_img_detect(self, aruco_cv2_img_draw):
        """Detects the aruco marker within a cv2 image, outputs the transform, and draws its location on the image."""
        # heavily based on the pyimagesearch detecting aruco markers with opencv and python tutorial
        # this line is different (uses a detector object) which was a change made in opencv 4.7.0 which was apparently done after the pyimagesearch thing was written. Don't take pyimagesearch as the last word or you will get errors without knowing why.
        (corners, ids, rejected) = self.detector.detectMarkers(aruco_cv2_img_draw)
        
        transform = None
        aruco_rmat = np.zeros((3,3))
        aruco_tvecs_first = np.array([0,0,0])
        transform_camframe = np.zeros((4,4))

        marker_det = False
        if len(corners) > 0:
            ids = ids.flatten()
            marker_det = True
            # iterate through detected markers
            for (markerCorner, markerID) in zip(corners, ids):
                # extract marker corners (always returned in the order as below)
                corners = markerCorner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # convert to integers for drawing on the image
                topLeft_int = (int(topLeft[0]), int(topLeft[1]))
                topRight_int = (int(topRight[0]), int(topRight[1]))
                bottomRight_int = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft_int = (int(bottomLeft[0]), int(bottomLeft[1]))

                # draw the aruco detection bounding box
                cv2.line(aruco_cv2_img_draw, topLeft_int, topRight_int, (0, 255, 0), 2)
                cv2.line(aruco_cv2_img_draw, topRight_int, bottomRight_int, (0, 255, 0), 2)
                cv2.line(aruco_cv2_img_draw, bottomRight_int, bottomLeft_int, (0, 255, 0), 2)
                cv2.line(aruco_cv2_img_draw, topLeft_int, bottomLeft_int, (0, 255, 0), 2)

                # find and draw the center by finding the midpoint btwn top left and bottom right corners
                cent_x = int((topLeft_int[0] + bottomRight_int[0]) / 2.0)
                cent_y = int((topLeft_int[1] + bottomRight_int[1]) / 2.0)
                cv2.circle(aruco_cv2_img_draw, (cent_x, cent_y), 4, (0, 255, 255), -1)

            # no need to put marker ID text since there shouldn't be more than one ever

            # now do solvePnP

            retval, aruco_rvecs, aruco_tvecs, aruco_reprj_error = cv2.solvePnPGeneric(self.objectPoints, corners, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

            #print(aruco_rvecs[0])

            aruco_rmat, jacobian = cv2.Rodrigues(aruco_rvecs[0])
            aruco_tvecs_first_vert = aruco_tvecs[0]
            aruco_tvecs_first = aruco_tvecs_first_vert.reshape(-1)

            

            transform_camframe = np.vstack((np.hstack((aruco_rmat, aruco_tvecs_first_vert)),np.array([0, 0, 0, 1])))


        else:
            marker_det = False
            # self.get_logger().info('No ArUco marker detected.')
        

        return marker_det, aruco_rmat, aruco_tvecs_first, transform_camframe, aruco_cv2_img_draw
    
    def drone_target_output(self, aruco_rmat_camera, aruco_tvec_camera):
        """Creates drone position and angle target outputs on the input transform between the detected aruco marker and the drone."""

        # ideal pose is (in relation to drone frame: x is right, y is forward, z is up):
            # 1000mm from the marker (y)
            # 0mm from the marker in x (horizontal alignment)
            # zdist*sin(vertical angle)*scaling factor from marker in z: move up and down with marker tilt, but keep it in camera frame since the drone can't tilt without moving

            # ignore x-axis rotation (can't maintain sideways tilt)
            # ignore y-axis rotation (can't maintain front-back tilt)
            # minimize difference in z-axis (look at the marker straight on)

        # remap to drone frame (x-right, y-forwards, z-up) from camera frame (x-right, y-down, z-forwards)

        aruco_rmat_droneframe = np.matmul(self.drone_frame_from_cam_rmat, aruco_rmat_camera)
        aruco_zvec_droneframe = aruco_rmat_droneframe[:,2].reshape(-1)

        aruco_tvec_droneframe = np.array([aruco_tvec_camera[0], aruco_tvec_camera[2], -aruco_tvec_camera[1]])

        # specify target position for drone relative to aruco marker (in drone frame)
        # drone should keep a specified distance from the marker (2-norm)
        # drone should change altitude to match the vertical angle of the marker (without letting the marker leave the FOV(10 deg up, 30 down))
        # drone should always face the marker to retain it in FOV: attitude control should be based on tvec only

        # position target:
        # get target vector from aruco marker (in drone frame)
        target_from_aruco_vec_droneframe = aruco_zvec_droneframe*self.aruco_follow_distance_mm
        
        # ensure the marker is within vertical FOV
        if target_from_aruco_vec_droneframe[2] > 0.5*self.aruco_follow_distance_mm-self.aruco_edge_size_mm/2.0:
            # too high case: drone target pos is over 30 deg above aruco marker bottom edge. Lower target position to compensate
            target_from_aruco_vec_droneframe[2] = 0.5*self.aruco_follow_distance_mm-self.aruco_edge_size_mm/2
            self.get_logger().info('Target position indicated by ArUco marker vertical angle is too high. Limiting target altitude to retain marker in FOV')
        elif target_from_aruco_vec_droneframe[2] < -0.174*self.aruco_follow_distance_mm+self.aruco_edge_size_mm/2.0:
            # too low case: drone target pos is over 10 degrees below aruco marker top edge. Raise target position to compensate
            self.get_logger().info('Target position indicated by ArUco marker vertical angle is too low. Increasing target altitude to retain marker in FOV')
            target_from_aruco_vec_droneframe[2] = -0.174*self.aruco_follow_distance_mm+self.aruco_edge_size_mm/2.0

        # drone movement target: current vector to marker + current target from marker (both in drone frame)
        drone_position_target = aruco_tvec_droneframe+target_from_aruco_vec_droneframe

        # yaw is positive clockwise (according to google), so no negative here (at least not yet, we'll see if it works or not)
        drone_zangle_target_rad = np.arctan2(aruco_tvec_droneframe[0],aruco_tvec_droneframe[1])

        # publish the transform of the aruco marker
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'drone'
        t.child_frame_id = 'aruco_marker'

        t.transform.translation.x = aruco_tvec_droneframe[0]
        t.transform.translation.y = aruco_tvec_droneframe[1]
        t.transform.translation.z = aruco_tvec_droneframe[2]

        q = rotm2quat(aruco_rmat_droneframe)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        return drone_position_target, drone_zangle_target_rad
        

    def drone_pd_controller(self, drone_pos_target, drone_ang_target):

        
        # do pd control (don't do d if you don't have a previous position or if it's been too long since you got one)
        prop_linear = self.linear_kp*drone_pos_target
        # self.integral_pos += drone_pos_target
        # int_linear = self.linear_ki*self.integral_pos
        if self.previous_error_pos != 0 and self.dt_timesteps <=10:
            deriv_linear = self.linear_kd*(drone_pos_target-self.previous_target_pos)/self.dt_timesteps
            linear_move_array = prop_linear+deriv_linear
        else:
            linear_move_array = prop_linear
        
        prop_angular = self.angular_kp*drone_ang_target
        if self.previous_error_angle !=0 and self.dt_timesteps <=10:
            deriv_angular = self.angular_kd*(drone_ang_target-self.previous_error_angle)/self.dt_timesteps
            angular_move = prop_angular+deriv_angular
        else:
            angular_move = prop_angular

        control_msg = Twist()

        control_msg.linear.x = float(int(clamp(linear_move_array[0],100,-100)*self.max_linear_spd_x/100))
        control_msg.linear.y = float(int(clamp(linear_move_array[1],100,-100)*self.max_linear_spd_y/100))
        control_msg.linear.z = float(int(clamp(linear_move_array[2],100,-100)*self.max_linear_spd_z/100))

        control_msg.angular.x = 0.0
        control_msg.angular.y = 0.0
        control_msg.angular.z = float(int(clamp(angular_move,100,-100)*self.max_angular_spd/100))
        self.tello_control_pub.publish(control_msg)

        


        self.previous_control_msg = control_msg
        self.previous_error_pos = np.linalg.norm(drone_pos_target)
        self.previous_target_pos = drone_pos_target
        self.previous_error_angle = drone_ang_target
        

        

    



def main(args=None):
    rclpy.init(args=args)
    tello_aruco_follower = ArucoFollowControl()

    rclpy.spin(tello_aruco_follower)

    tello_aruco_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()