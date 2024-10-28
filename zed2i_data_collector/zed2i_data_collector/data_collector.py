import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import os
from datetime import datetime
from cv_bridge import CvBridge
import cv2

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # Initialize dataset path (input parameter)
        self.declare_parameter('camera_direction', 'front')
        self.direction = self.get_parameter('camera_direction').value

        # Create subscriptions for RGB and depth images
        self.sub_rgb= self.create_subscription(
            Image,
            f'/zed_{self.direction}/zed_node/rgb_raw/image_raw_color',
            self.rgb_callback,
            10)
        self.sub_depth = self.create_subscription(
            Image,
            f'/zed_{self.direction}/zed_node/depth/depth_registered',
            self.depth_callback,
            10)
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        self.start_time = datetime.now().strftime('_DMY_%d_%m_%Y_HM_%H_%M')
        
        # Initialize dataset path (input parameter)
        self.declare_parameter('dataset_location', '~/tau_dataset')
        self.dataset_location = self.get_parameter('dataset_location').value
        
        # Resolve the dataset location
        self.base_path = os.path.expanduser(self.dataset_location)
        os.makedirs(self.base_path, exist_ok=True)
        
        # Create subdirectories for RGB and depth images
        self.rgb_path = os.path.join(self.base_path, f'{self.direction}_camera_rgb_raw')
        self.rgb_path = os.path.join(self.rgb_path, f'{self.direction}_camera_rgb_raw_{self.start_time}')
        self.depth_path = os.path.join(self.base_path, f'{self.direction}_camera_depth')
        self.depth_path = os.path.join(self.depth_path, f'{self.direction}_camera_depth_{self.start_time}')
        os.makedirs(self.rgb_path, exist_ok=True)
        os.makedirs(self.depth_path, exist_ok=True)
        
        # Create 'data' folders inside RGB and depth paths
        timestamp = datetime.now().strftime('%d/%m/%Y %H:%M')
        self.rgb_data_path = os.path.join(self.rgb_path, 'data')
        self.depth_data_path = os.path.join(self.depth_path, 'data')
        os.makedirs(self.rgb_data_path, exist_ok=True)
        os.makedirs(self.depth_data_path, exist_ok=True)
        
        # Initialize counters for image numbering
        self.rgb_count = 0
        self.depth_count = 0
        
        # Log initialization message
        self.get_logger().info(f'Data collector node has been started. Saving data to {self.base_path}')

 

    def rgb_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.rgb_count += 1
        timestamp = datetime.now().strftime('%d/%m/%Y %H:%M:%S:%f')
        
        # Save RGB image
        file_name = os.path.join(self.rgb_data_path, f'image_{self.rgb_count}.png')
        cv2.imwrite(file_name, cv_image)
        
        # Append timestamp to timestamps.txt
        with open(os.path.join(self.rgb_path, 'timestamps.txt'), 'a') as f:
            f.write(f'{timestamp}\n')
        
        self.get_logger().info(f'Saved RGB image to {file_name}')

    def depth_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_count += 1
        timestamp = datetime.now().strftime('%d/%m/%Y %H:%M:%S:%f')
        
        # Save Depth image
        file_name = os.path.join(self.depth_data_path, f'image_{self.depth_count}.png')
        cv2.imwrite(file_name, cv_image)
        
        # Append timestamp to timestamps.txt
        with open(os.path.join(self.depth_path, 'timestamps.txt'), 'a') as f:
            f.write(f'{timestamp}\n')
        
        self.get_logger().info(f'Saved Depth image to {file_name}')

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()