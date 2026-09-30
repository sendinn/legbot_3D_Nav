#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
import struct

import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from navigation_core import read_pcd,make_tomogram,nearest_surface,register_cloud,transform_points,prepare_map


def write_pcd(path,points,mode='binary'):
    points=np.asarray(points,dtype='<f4')
    header=('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA %s\n'%(len(points),len(points),mode)).encode()
    if mode=='binary':raw=points.tobytes()
    elif mode=='ascii':raw=('\n'.join(' '.join(map(str,p)) for p in points)+'\n').encode()
    else:
        source=points.T.copy().tobytes()
        packed=b''.join(bytes([len(source[i:i+32])-1])+source[i:i+32] for i in range(0,len(source),32))
        raw=struct.pack('<II',len(packed),len(source))+packed
    path.write_bytes(header+raw)


def plane():
    x,y=np.meshgrid(np.arange(-3,3.01,.05),np.arange(-3,3.01,.05))
    return np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))


class NavigationTests(unittest.TestCase):
    def test_pcd_encodings_and_truncation(self):
        xyz=np.arange(90,dtype=float).reshape(30,3)/10
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'test.pcd'
            for mode in ('binary','ascii','binary_compressed'):
                write_pcd(p,xyz,mode)
                np.testing.assert_allclose(read_pcd(p),xyz,rtol=1e-6)
            write_pcd(p,xyz)
            p.write_bytes(p.read_bytes()[:-1])
            with self.assertRaises(ValueError):read_pcd(p)

    def test_unknown_and_low_clearance_stay_blocked(self):
        points=plane()
        roof=points[(abs(points[:,0])<.8)&(abs(points[:,1])<.8)].copy()
        roof[:,2]=.25
        data=make_tomogram(np.vstack((points,roof)),.15,.5)
        # At ground height there is no valid surface beneath the low ceiling.
        with self.assertRaises(ValueError):nearest_surface(data,[0,0,0],radius=.2,height_tolerance=.1)
        surface,_=nearest_surface(data,[2,2,0])
        self.assertLess(abs(surface[2]),.01)
        with self.assertRaises(ValueError):nearest_surface(data,[8,8,0])

    def test_registration_and_rejection(self):
        rng=np.random.default_rng(13)
        target=rng.uniform(-2,2,(2500,3))
        truth=np.eye(4)
        truth[:3,:3]=Rotation.from_euler('z',.2).as_matrix()
        truth[:3,3]=[.5,-.3,.2]
        source=transform_points(np.linalg.inv(truth),target)
        initial=truth.copy();initial[:3,3]+=[.08,-.04,.02]
        found,fitness,rmse=register_cloud(source,target,initial)
        np.testing.assert_allclose(found,truth,atol=.04)
        self.assertGreater(fitness,.9)
        self.assertLess(rmse,.08)
        bad=np.eye(4);bad[:3,3]=[100,0,0]
        with self.assertRaises(ValueError):register_cloud(source,target,bad)

    def test_cache_invalidates_changed_map(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'cloud.pcd'
            write_pcd(p,plane())
            a=prepare_map(p,Path(d)/'cache')
            self.assertEqual(a,prepare_map(p,Path(d)/'cache'))
            write_pcd(p,plane()+[1,0,0])
            self.assertNotEqual(a,prepare_map(p,Path(d)/'cache'))


if __name__=='__main__':unittest.main()
