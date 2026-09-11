import unittest
import struct
from types import SimpleNamespace
import numpy as np
from uav_coverage_memory.cloud import decode_xyz


class CloudTests(unittest.TestCase):
    def cloud(self,big=False,double=False):
        size=8 if double else 4;step=4*size
        endian='>' if big else '<';code='d' if double else 'f'
        data=b''
        for row in range(2):
            for column in range(2):data+=struct.pack(endian+code*4,row,column,row+column,999)
            data+=b'pad!'
        fields=[SimpleNamespace(name=name,offset=i*size,count=1,datatype=8 if double else 7)
                for i,name in enumerate(['x','y','z'])]
        return SimpleNamespace(width=2,height=2,point_step=step,row_step=step*2+4,
                               data=data,fields=fields,is_bigendian=big)

    def test_padding_float32_float64_and_both_byte_orders(self):
        for big in [False,True]:
            for double in [False,True]:
                np.testing.assert_allclose(decode_xyz(self.cloud(big,double),4),[[0,0,0],[0,1,1],[1,0,1],[1,1,2]])

    def test_limit_not_silent_truncation(self):
        with self.assertRaises(ValueError):decode_xyz(self.cloud(),3)

    def test_missing_fields_and_short_buffers_rejected(self):
        c=self.cloud();c.fields=c.fields[:2]
        with self.assertRaises(ValueError):decode_xyz(c,4)
        c=self.cloud();c.data=c.data[:-8]
        with self.assertRaises(ValueError):decode_xyz(c,4)

    def test_nan_filter(self):
        c=self.cloud();c.data=struct.pack('<f',float('nan'))+c.data[4:]
        self.assertEqual(decode_xyz(c,4).shape,(3,3))

    def test_padding_bytes_count_toward_limit(self):
        c=self.cloud()
        with self.assertRaisesRegex(ValueError,'cloud_byte_limit'):
            decode_xyz(c,4,max_bytes=len(c.data)-1)
