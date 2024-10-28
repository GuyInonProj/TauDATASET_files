import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
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
        self.point_count = 0
        self.file_index = 0
        self.current_points = []
        self.start_time = None
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
        if self.start_time is None:
            self.start_time = current_time
        
        point_data, data = self.process_point_cloud(msg)
        
        for i in range(0, len(data), 4):
            x, y, z, intensity = data[i:i+4]
            self.current_points.append((x, y, z, intensity, current_time))
            self.point_count += 1
            
            if self.point_count >= 131072:
                self.save_current_points()
                self.current_points = []
                self.point_count = 0
                self.start_time = current_time
        
    def process_point_cloud(self, msg):
        print("Lidar data received processing......")
        point_data = Float32MultiArray()
        data = []
        for i in range(0, len(msg.data), msg.point_step):  
            x, = struct.unpack_from('f', msg.data, i + 0)  
            y, = struct.unpack_from('f', msg.data, i + 4)  
            z, = struct.unpack_from('f', msg.data, i + 8)  
            intensity, = struct.unpack_from('H', msg.data, i + 24)
            data.append(x)
            data.append(y)
            data.append(z)
            data.append(intensity)
            point_data.data.extend([x, y, z, intensity])
        
        return point_data, data
    
    def save_current_points(self):
        self.file_index += 1
        txt_file_path = os.path.join(self.data_path, f'data_{self.file_index}.txt')
        
        with open(txt_file_path, 'w') as txt_file:
            for point in self.current_points:
                x, y, z, intensity, timestamp = point
                txt_file.write(f'{x} {y} {z} {intensity}\n')
        
        end_time = self.current_points[-1][-1]
        start_time_obj = datetime.strptime(self.start_time, '%d/%m/%Y %H:%M:%S:%f')
        end_time_obj = datetime.strptime(end_time, '%d/%m/%Y %H:%M:%S:%f')
        middle_time = start_time_obj + (end_time_obj - start_time_obj) / 2
        
        with open(self.timestamp_file_paths['start'], 'a') as f:
            f.write(f'{self.file_index} {self.start_time}\n')
        
        with open(self.timestamp_file_paths['end'], 'a') as f:
            f.write(f'{self.file_index} {end_time}\n')
        
        with open(self.timestamp_file_paths['middle'], 'a') as f:
            f.write(f'{self.file_index} {middle_time.strftime("%d/%m/%Y %H:%M:%S:%f")}\n')
        
        # Logging message for each file saved
        self.get_logger().info(f'Saved point cloud data to {txt_file_path}')
        self.get_logger().info(f'Updated timestamps for file {self.file_index}: start - {self.start_time}, end - {end_time}, middle - {middle_time.strftime("%d/%m/%Y %H:%M:%S:%f")}')

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