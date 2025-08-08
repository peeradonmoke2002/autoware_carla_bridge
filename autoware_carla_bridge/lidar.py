#!/usr/bin/python3
# lidar.py
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class LidarExtended(object):

    def __init__(self, node: Node):
        self.node = node
        self.input_pointcloud = PointCloud2()
        self._lidar_subscriber = self.node.create_subscription(
            PointCloud2, '~/input/lidar',
            self.lidar_callback, 1)
        self._lidar_ex_publisher = self.node.create_publisher(
            PointCloud2, '~/output/lidar_ex', 1)

    def lidar_callback(self, msg: PointCloud2):
        self.input_pointcloud = msg
        
    def update(self):
        # Skip processing if no pointcloud data received yet
        if len(self.input_pointcloud.data) == 0:
            return
            
        pointcloud_ex = PointCloud2()
        pointcloud_ex.header.stamp = self.node.get_clock().now().to_msg()
        pointcloud_ex.header.stamp.sec = self.input_pointcloud.header.stamp.sec
        pointcloud_ex.header.stamp.nanosec = self.input_pointcloud.header.stamp.nanosec
        pointcloud_ex.header.frame_id = self.input_pointcloud.header.frame_id
        pointcloud_ex.height = self.input_pointcloud.height
        pointcloud_ex.width = self.input_pointcloud.width
        pointcloud_ex.is_bigendian = self.input_pointcloud.is_bigendian
        pointcloud_ex.point_step = self.input_pointcloud.point_step + 8  # two of float32 numbers
        pointcloud_ex.row_step = pointcloud_ex.width * pointcloud_ex.point_step  # Update row_step with new point_step
        pointcloud_ex.is_dense = self.input_pointcloud.is_dense
        
        # Use frombuffer instead of fromstring (fromstring is deprecated)
        data = np.frombuffer(
            bytes(self.input_pointcloud.data),
            dtype=self._convert_fields_to_dtype(self.input_pointcloud.fields))

        pointcloud_ex.fields = self.input_pointcloud.fields
        self._add_field(pointcloud_ex.fields,
                        PointField(name='azimuth', datatype=PointField.FLOAT32, count=1))
        self._add_field(pointcloud_ex.fields,
                        PointField(name='distance', datatype=PointField.FLOAT32, count=1))

        data = self._calculate_and_add_azimuth_and_distance(data)

        pointcloud_ex.data = self._create_pointcloud_data(
            data, pointcloud_ex.fields, pointcloud_ex.is_bigendian)

        self._lidar_ex_publisher.publish(pointcloud_ex)
        
  

    def _add_field(self, fields, new_field):
        last_field = fields[-1]
        last_bytes_count = self._get_bytes_count_from_datatype(last_field.datatype)

        new_field.offset = last_field.offset + last_bytes_count * last_field.count

        fields.append(new_field)

    def _get_bytes_count_from_datatype(self, datatype):
        if datatype >= PointField.INT8 and datatype <= PointField.UINT8:
            bytes_count = 1
        elif datatype >= PointField.INT16 and datatype <= PointField.UINT16:
            bytes_count = 2
        elif datatype >= PointField.INT32 and datatype <= PointField.FLOAT32:
            bytes_count = 4
        elif datatype == PointField.FLOAT64:
            bytes_count = 8
        else:
            raise RuntimeError(f'{datatype} datatype unimplemented')
        return bytes_count

    def _convert_fields_to_dtype(self, fields):
        types = []
        for field in fields:
            types.append((
                field.name,
                self._convert_datatype_to_dtype(field.datatype)
            ))
        return np.dtype(types)

    def _convert_fields_to_struct_format(self, fields, is_bigendian):
        struct_format = ''
        for field in fields:
            struct_format += self._convert_datatype_to_struct_format(field.datatype)

        return struct_format

    def _convert_datatype_to_dtype(self, datatype):
        if datatype == PointField.INT8:
            return np.int8
        elif datatype == PointField.UINT8:
            return np.uint8
        elif datatype == PointField.INT16:
            return np.int16
        elif datatype == PointField.UINT16:
            return np.uint16
        elif datatype == PointField.INT32:
            return np.int32
        elif datatype == PointField.UINT32:
            return np.uint32
        elif datatype == PointField.FLOAT32:
            return np.float32
        elif datatype == PointField.FLOAT64:
            return np.float64

    def _convert_datatype_to_struct_format(self, datatype):
        if datatype == PointField.INT8:
            return 'b'
        elif datatype == PointField.UINT8:
            return 'B'
        elif datatype == PointField.INT16:
            return 'h'
        elif datatype == PointField.UINT16:
            return 'H'
        elif datatype == PointField.INT32:
            return 'i'
        elif datatype == PointField.UINT32:
            return 'I'
        elif datatype == PointField.FLOAT32:
            return 'f'
        elif datatype == PointField.FLOAT64:
            return 'd'

    def _calculate_and_add_azimuth_and_distance(self, data):
        # Check if the data contains the required fields
        required_fields = ['x', 'y', 'z']
        if not all(field in data.dtype.names for field in required_fields):
            self.node.get_logger().warn(f"Required fields missing in PointCloud2. Available fields: {data.dtype.names}")
            return data  # Return unchanged data if fields are missing
            
        new_dtype = np.dtype(data.dtype.descr + [('azimuth', '<f4'), ('distance', '<f4')])
        new_data = np.empty(data.shape, dtype=new_dtype)
        for description in data.dtype.descr:
            field_name = description[0]
            new_data[field_name] = data[field_name]

        for row in new_data:
            # Correct azimuth calculation: atan2(y, x) not atan2(x, y)
            row['azimuth'] = np.arctan2(row['y'], row['x'])  
            row['distance'] = np.sqrt(
                np.square(row['x']) + np.square(row['y']) + np.square(row['z']))

        return new_data

    def _create_pointcloud_data(self, data, fields, is_bigendian):
        return data.tobytes('C')
    
    def destroy(self):
        self.node.get_logger().info("Destroying LidarExtended")
        self._lidar_subscriber.destroy()
        self._lidar_ex_publisher.destroy()