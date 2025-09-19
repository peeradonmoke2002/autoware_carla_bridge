#!/usr/bin/python3
# lidar.py
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class LidarExtended(object):

    def __init__(self, node: Node):
        self.node = node
        self.input_pointcloud = PointCloud2()
        self._has_msg = False  
        self._lidar_subscriber = self.node.create_subscription(
            PointCloud2, '~/input/lidar_ex', self.lidar_callback, 1)
        self._lidar_ex_publisher = self.node.create_publisher(
            PointCloud2, '~/output/lidar_ex', 1)

    def lidar_callback(self, msg: PointCloud2):
        self.input_pointcloud = msg
        self._has_msg = True

    def update(self):
        if (
            not self._has_msg
            or not self.input_pointcloud.fields
            or not self.input_pointcloud.data
        ):
            return

        pc_in = self.input_pointcloud

        pc_out = PointCloud2()
        pc_out.header.frame_id = pc_in.header.frame_id
        pc_out.header.stamp = pc_in.header.stamp
        pc_out.height = pc_in.height
        pc_out.width = pc_in.width
        pc_out.is_bigendian = pc_in.is_bigendian
        pc_out.is_dense = pc_in.is_dense

        pc_out.fields = [
            PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity',   offset=12, datatype=PointField.UINT8,   count=1),
            PointField(name='return_type', offset=13, datatype=PointField.UINT8,   count=1),
            PointField(name='channel',     offset=14, datatype=PointField.UINT16,  count=1),
        ]
        pc_out.point_step = 16
        pc_out.row_step = pc_out.point_step * pc_out.width

        data_in = self._to_numpy(pc_in)

        out_dtype = np.dtype([
            ('x','<f4'), ('y','<f4'), ('z','<f4'),
            ('intensity','u1'), ('return_type','u1'), ('channel','<u2'),
        ])
        out = np.empty(data_in.shape, dtype=out_dtype)

        out['x'] = data_in['x'].astype(np.float32, copy=False)
        out['y'] = data_in['y'].astype(np.float32, copy=False)
        out['z'] = data_in['z'].astype(np.float32, copy=False)

        if 'intensity' in data_in.dtype.names and data_in.size > 0:
            i = data_in['intensity'].astype(np.float32, copy=False)
            if np.nanmax(i) <= 1.0:
                i = i * 255.0
            out['intensity'] = np.clip(i, 0, 255).astype(np.uint8)
        else:
            out['intensity'] = 0

        out['return_type'] = 0

        if 'channel' in data_in.dtype.names:
            out['channel'] = data_in['channel'].astype(np.uint16, copy=False)
        elif 'ring' in data_in.dtype.names:
            out['channel'] = data_in['ring'].astype(np.uint16, copy=False)
        else:
            out['channel'] = 0

        pc_out.data = out.tobytes('C')
        self._lidar_ex_publisher.publish(pc_out)


    def _add_field(self, fields, new_field):
        last_field = fields[-1]
        last_bytes_count = self._get_bytes_count_from_datatype(last_field.datatype)
        new_field.offset = last_field.offset + last_bytes_count * last_field.count
        fields.append(new_field)

    def _get_bytes_count_from_datatype(self, datatype):
        if PointField.INT8 <= datatype <= PointField.UINT8:
            return 1
        elif PointField.INT16 <= datatype <= PointField.UINT16:
            return 2
        elif PointField.INT32 <= datatype <= PointField.FLOAT32:
            return 4
        elif datatype == PointField.FLOAT64:
            return 8
        else:
            raise RuntimeError(f'{datatype} datatype unimplemented')

    def _convert_fields_to_dtype(self, fields):
        types = []
        for field in fields:
            types.append((field.name, self._convert_datatype_to_dtype(field.datatype)))
        return np.dtype(types)

    def _convert_datatype_to_dtype(self, datatype):
        if datatype == PointField.INT8:    return np.int8
        if datatype == PointField.UINT8:   return np.uint8
        if datatype == PointField.INT16:   return np.int16
        if datatype == PointField.UINT16:  return np.uint16
        if datatype == PointField.INT32:   return np.int32
        if datatype == PointField.UINT32:  return np.uint32
        if datatype == PointField.FLOAT32: return np.float32
        if datatype == PointField.FLOAT64: return np.float64

    def _to_numpy(self, pc2: PointCloud2):
        dtype = self._convert_fields_to_dtype(pc2.fields)
        count = pc2.width * pc2.height
        arr = np.frombuffer(pc2.data, dtype=dtype, count=count)
        return arr

    def _calculate_and_add_azimuth_and_distance(self, data):
        new_dtype = np.dtype(data.dtype.descr + [('azimuth', '<f4'), ('distance', '<f4')])
        new_data = np.empty(data.shape, dtype=new_dtype)
        for name, _fmt in data.dtype.descr:
            new_data[name] = data[name]
        x = new_data['x'].astype(np.float32, copy=False)
        y = new_data['y'].astype(np.float32, copy=False)
        z = new_data['z'].astype(np.float32, copy=False)
        new_data['azimuth'] = np.arctan2(y, x)
        new_data['distance'] = np.sqrt(x*x + y*y + z*z, dtype=np.float32)
        return new_data

    def _create_pointcloud_data(self, data, fields, is_bigendian):
        return data.tobytes('C')

    def destroy(self):
        self.node.get_logger().info("Destroying LidarExtended")
        self._lidar_subscriber.destroy()
        self._lidar_ex_publisher.destroy()
