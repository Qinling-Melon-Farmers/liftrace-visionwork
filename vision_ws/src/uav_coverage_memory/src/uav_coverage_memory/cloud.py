"""Bounded PointCloud2 XYZ decoding preserving row padding and byte order."""
import numpy as np


def decode_xyz(message, limit, max_bytes=16*1024*1024):
    if len(message.data) > max_bytes:
        raise ValueError('cloud_byte_limit')
    width, height = int(message.width), int(message.height)
    if width < 0 or height < 0 or width*height > limit:
        raise ValueError('cloud_point_limit')
    if not width*height:
        return np.empty((0,3), dtype=np.float64)
    point_step, row_step = int(message.point_step), int(message.row_step)
    if point_step <= 0 or row_step < width*point_step or len(message.data) < row_step*height:
        raise ValueError('invalid_cloud_layout')
    source = {field.name:field for field in message.fields}
    formats, offsets = [], []
    for name in ['x','y','z']:
        field = source.get(name)
        if field is None or field.count != 1 or field.datatype not in (7,8):
            raise ValueError('unsupported_xyz_field')
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset+size > point_step:
            raise ValueError('invalid_xyz_offset')
        formats.append(('>' if message.is_bigendian else '<')+('f4' if size==4 else 'f8'))
        offsets.append(field.offset)
    dtype = np.dtype({'names':['x','y','z'],'formats':formats,'offsets':offsets,'itemsize':point_step})
    cloud = np.ndarray((height,width),dtype=dtype,buffer=message.data,strides=(row_step,point_step))
    points = np.column_stack([cloud[name].ravel() for name in ['x','y','z']]).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]
