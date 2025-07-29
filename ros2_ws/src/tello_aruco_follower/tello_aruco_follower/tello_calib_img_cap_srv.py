import rclpy
import cv2
import numpy as np
import yaml
import ament_index_python

from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger # empty request, boolean response to indicate success
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

# will need to use a multithreadedexecutor and reentrantcallbackgroup
# this allows this code to run a subscriber to get the image data and a service server to run calibration on it when called

# subscriber gets images, callback converts them to opencv and saves them to a self variable within the object
# service callback gets called with a Trigger service type, but makes use of the self image variable to do the calibration


class TelloCalibratorService(Node):
    """Calibration service server node for the tello drone"""
    def __init__(self):
        super().__init__('tello_calibrator_srv')

        self.bridge = CvBridge()
        self.image_read_successful = False

        # declare a parameter for the number of images to use for calibration
        self.declare_parameter('calib_img_num', 10)
        # declare chessboard parameters
        # perhaps we should use images on a laptop monitor for both calibration and guiding with the marker
        self.declare_parameter('chessboard_square_edge_mm', 106.2/6)
        self.declare_parameter('chessboard_corners_vert', 6)
        self.declare_parameter('chessboard_corners_horiz', 8)

        self.calib_img_num = int(self.get_parameter('calib_img_num').value)
        self.chessboard_square_edge_mm = float(self.get_parameter('chessboard_square_edge_mm').value)
        self.chessboard_corners_vert = int(self.get_parameter('chessboard_corners_vert').value)
        self.chessboard_corners_horiz = int(self.get_parameter('chessboard_corners_horiz').value)

        self.objp = np.zeros((self.chessboard_corners_vert*self.chessboard_corners_horiz, 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.chessboard_corners_vert,0:self.chessboard_corners_horiz].T.reshape(-1,2)*self.chessboard_square_edge_mm

        self.object_points=[]
        self.image_points=[]
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # define the callback group as a ReentrantCallbackGroup so both can run
        self.group = ReentrantCallbackGroup()
        # create the calibration service
        self.calib_srv = self.create_service(
            Trigger, 'tello_calib_srv', self.tello_calib_callback
        )

        self.image_sub = self.create_subscription(Image, 'image_raw', self.read_image_callback, 1)
        self.calib_image_list = []
        self.calib_img_counter = 0

        self.filepath = ament_index_python.get_package_share_directory('tello_aruco_follower')
        self.file = self.filepath + '/camera_params.yaml'

       


        

    # subscriber callback reads the image and converts it to cv2 for use by the calibrator    
    def read_image_callback(self, image_msg):
        
        # try to convert it to cv2 image
        try:
            self.image_read_successful = True
            self.cv2_img_grey =  cv2.cvtColor(self.bridge.imgmsg_to_cv2(image_msg, 'rgb8'), cv2.COLOR_RGB2GRAY)
        except CvBridgeError as e:
            self.get_logger().warn(e)
            self.image_read_successful = False

    def tello_calib_callback(self, request, response):
        
        
        
        if self.image_read_successful:
            if self.calib_img_counter < self.calib_img_num:
                # if image read is successful and you don't have enough calib images, add the calib image to the list and increment the counter
                self.calib_image_list.append(self.cv2_img_grey)
                self.calib_img_counter += 1
                self.get_logger().info('Calibration image capture successful.\nCaptured %d out of %d calibration images' % (self.calib_img_counter, self.calib_img_num))
                if self.calib_img_counter == self.calib_img_num:
                    self.get_logger().info('All calibration images gathered. Call service again to run calibration')
                response.success = True
                response.message = 'Calibration image collected'
            else:
                # if image read is successful and you do have enough calib images, then do the calibration
                self.get_logger().info('Beginning camera calibration')
                # do the calibration
                for calib_img_gray in self.calib_image_list:
                    # find chessboard corners
                    self.ret, self.corners = cv2.findChessboardCorners(calib_img_gray, (self.chessboard_corners_vert, self.chessboard_corners_horiz))
                    # if found then add the objpoints and img points to the calibration lists
                    if self.ret == True:
                        self.object_points.append(self.objp)

                        self.corners2 = cv2.cornerSubPix(calib_img_gray, self.corners, (11,11), (-1,-1), self.criteria)
                        self.image_points.append(self.corners2)

                        cv2.drawChessboardCorners(calib_img_gray, (self.chessboard_corners_vert, self.chessboard_corners_horiz), self.corners2, self.ret)
                        cv2.imshow('calib_img', calib_img_gray)
                        cv2.waitKey(500)
                    
                cv2.destroyAllWindows()    
                self.calib_flags = cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL + cv2.CALIB_TILTED_MODEL 
                self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(self.object_points, self.image_points, self.cv2_img_grey.shape[::-1], None, None, flags=self.calib_flags)

                # if successful

                # now dump all the stuff to a yaml file in the right directory
                self.get_logger().info('Dumping camera params to os path ' + self.file)
                
                self.cam_param_dict = {
                    "img_width": calib_img_gray.shape[0],
                    "img_height": calib_img_gray.shape[1],
                    "camera_matrix": self.mtx.tolist(),
                    "dist_coeffs": self.dist.tolist(),
                }

                with open(self.file, 'w') as yamlpath:
                    yaml.safe_dump(self.cam_param_dict, yamlpath)
                    self.get_logger().info('YAML file successfully written.')

                response.success = True
                response.message = 'Calibration completed successfully'
        else:
            response.success = False
            response.message = 'Image capture failed'
            self.get_logger().warn('Image capture failed, calibration image not gathered')
        
        return response

def main(args=None):
    rclpy.init(args=args)

    # create the multithreaded executor, add the node to it, and spin it
    try:
        tello_calib_service = TelloCalibratorService()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(tello_calib_service)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            tello_calib_service.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

            

    

