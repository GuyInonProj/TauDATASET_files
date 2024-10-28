import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import os
from datetime import datetime
import struct

class OusterDataCollector(Node):
    def __init__(self):
        super().__init__('ouster_data_collector')
        
        # Create subscription for PointCloud2 data
        self.sub_pc = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.pc_callback,
            10)
        
        # Initialize dataset path (input parameter)
        self.declare_parameter('dataset_location', '~/tau_dataset')
        self.dataset_location = self.get_parameter('dataset_location').value
        
        # Resolve the dataset location
        self.base_path = os.path.expanduser(self.dataset_location)
        os.makedirs(self.base_path, exist_ok=True)
        
        # Create subdirectory for PointCloud data with timestamp
        self.pc_path = os.path.join(self.base_path, 'ouster_data')
        self.timestamped_path = os.path.join(self.pc_path, datetime.now().strftime('ouster_data_DMY_%d_%m_%Y_HM_%H_%M'))
        os.makedirs(self.timestamped_path, exist_ok=True)
        
        # Create 'data' subdirectory inside the timestamped path
        self.data_path = os.path.join(self.timestamped_path, 'data')
        os.makedirs(self.data_path, exist_ok=True)
        
        # Initialize file paths and counters
        self.file_index = 0
        
        # Initialize timestamp file paths
        self.timestamp_file_paths = {
            'start': os.path.join(self.timestamped_path, 'timestamp_start.txt'),
            'end': os.path.join(self.timestamped_path, 'timestamp_end.txt'),
            'middle': os.path.join(self.timestamped_path, 'timestamp.txt')
        }
        
        # Open timestamp files for writing
        self.timestamp_files = {key: open(path, 'w') for key, path in self.timestamp_file_paths.items()}
        
        # Log initialization message
        self.get_logger().info(f'Ouster data collector node has been started. Saving data to {self.data_path}')

    def pc_callback(self, msg):
        # Use the current system time for the timestamp
        current_time = datetime.now().strftime('%d/%m/%Y %H:%M:%S:%f')
        
        # Save the raw point cloud data as a binary file
        self.save_raw_data(msg, current_time)

    def save_raw_data(self, msg, timestamp):
        self.file_index += 1
        bin_file_path = os.path.join(self.data_path, f'data_{self.file_index}.bin')
        
        # Convert the point cloud data to a binary format
        bin_data = bytearray(struct.pack('f' * (len(msg.data) // 4), *struct.unpack('f' * (len(msg.data) // 4), msg.data)))
        
        with open(bin_file_path, 'wb') as bin_file:
            bin_file.write(bin_data)
        
        # Logging message for each file saved
        self.get_logger().info(f'Saved raw point cloud data to {bin_file_path}')

        # Save timestamps
        with open(self.timestamp_file_paths['start'], 'a') as f:
            f.write(f'{self.file_index} {timestamp}\n')
        
        with open(self.timestamp_file_paths['end'], 'a') as f:
            f.write(f'{self.file_index} {timestamp}\n')
        
        with open(self.timestamp_file_paths['middle'], 'a') as f:
            f.write(f'{self.file_index} {timestamp}\n')

    def destroy_node(self):
        # Close files on node destruction
        for f in self.timestamp_files.values():
            f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OusterDataCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
