"""PCD parsing, CPU PCT tomography, surface selection and rigid registration."""
import hashlib
import json
import math
from pathlib import Path
import pickle
import struct

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def read_pcd(path):
    path = Path(path)
    with path.open('rb') as stream:
        header = {}
        for _ in range(100):
            line = stream.readline(8192)
            if not line:
                raise ValueError('Missing PCD DATA header (Git LFS pointer is not a PCD)')
            text = line.decode('ascii').strip()
            if text and not text.startswith('#'):
                key, *value = text.split()
                header[key.upper()] = value
                if key.upper() == 'DATA':
                    break
        else:
            raise ValueError('PCD header too long')
        names = header.get('FIELDS', header.get('FIELD'))
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        points = int(header.get('POINTS', [str(int(header['WIDTH'][0])*int(header.get('HEIGHT',['1'])[0]))])[0])
        if not 0 < points <= 30_000_000 or not (len(names) == len(sizes) == len(types) == len(counts)):
            raise ValueError('Invalid PCD dimensions/fields')
        if not all(axis in names for axis in ('x','y','z')):
            raise ValueError('PCD needs x/y/z fields')
        fields = []
        for name, size, typ, count in zip(names, sizes, types, counts):
            if count < 1 or size not in (1,2,4,8) or typ not in ('F','I','U'):
                raise ValueError('Unsupported PCD field')
            code = '<' + {'F':'f','I':'i','U':'u'}[typ] + str(size)
            fields.append((name, code) if count == 1 else (name, code, (count,)))
        dtype = np.dtype(fields)
        mode = header['DATA'][0]
        if mode == 'binary':
            raw = stream.read()
            if len(raw) != points*dtype.itemsize:
                raise ValueError('Incomplete binary PCD')
            records = np.frombuffer(raw, dtype=dtype, count=points)
            xyz = np.column_stack([records[k] for k in ('x','y','z')])
        elif mode == 'ascii':
            records = np.loadtxt(stream, ndmin=2)
            offsets = np.cumsum([0]+counts[:-1])
            if records.shape != (points, sum(counts)):
                raise ValueError('Invalid ASCII PCD dimensions')
            xyz = records[:, [offsets[names.index(k)] for k in ('x','y','z')]]
        elif mode == 'binary_compressed':
            length = stream.read(8)
            if len(length) != 8:
                raise ValueError('Incomplete compressed PCD')
            compressed_size, full_size = struct.unpack('<II', length)
            if full_size != points*dtype.itemsize:
                raise ValueError('Invalid compressed PCD size')
            packed = stream.read()
            if len(packed) != compressed_size:
                raise ValueError('Incomplete compressed PCD')
            raw = lzf_decode(packed, full_size)
            offset, values = 0, {}
            for name, size, typ, count in zip(names, sizes, types, counts):
                n = points*size*count
                if name in ('x','y','z'):
                    if count != 1:
                        raise ValueError('XYZ fields must be scalar')
                    values[name] = np.frombuffer(raw, dtype=dtype[name], count=points, offset=offset)
                offset += n
            xyz = np.column_stack([values[k] for k in ('x','y','z')])
        else:
            raise ValueError('Unsupported PCD DATA encoding: ' + mode)
    xyz = np.asarray(xyz, dtype=np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < 30:
        raise ValueError('PCD has too few finite XYZ points')
    return xyz


def lzf_decode(data, expected):
    out = bytearray()
    i = 0
    while i < len(data):
        ctrl = data[i]; i += 1
        if ctrl < 32:
            length = ctrl + 1
            if i+length > len(data): raise ValueError('Bad LZF literal')
            out.extend(data[i:i+length]); i += length
        else:
            length = ctrl >> 5
            offset = (ctrl & 31) << 8
            if length == 7:
                length += data[i]; i += 1
            offset += data[i]; i += 1
            ref = len(out)-offset-1
            if ref < 0: raise ValueError('Bad LZF reference')
            for _ in range(length+2):
                out.append(out[ref]); ref += 1
        if len(out) > expected: raise ValueError('LZF output exceeds header')
    if len(out) != expected: raise ValueError('Truncated LZF data')
    return bytes(out)


def voxel_downsample(points, resolution):
    index = np.floor(points/resolution).astype(np.int64)
    _, keep = np.unique(index, axis=0, return_index=True)
    return points[np.sort(keep)]


def make_tomogram(points, resolution=0.15, slice_dh=0.5, inflation=0.20):
    """CPU implementation of PCT's floor/ceiling, gradient and clearance model.
    Unobserved XY cells stay blocked. No synthetic floor or hole filling.
    """
    lo, hi = points.min(axis=0), points.max(axis=0)
    dims = np.ceil((hi[:2]-lo[:2])/resolution).astype(int)+6
    center = (hi[:2]+lo[:2])/2
    ground = math.floor(lo[2]/slice_dh)*slice_dh
    slices = max(2, int(math.ceil((hi[2]-ground)/slice_dh))+1)
    cells = slices*int(np.prod(dims))
    if cells > 25_000_000:
        raise ValueError('Map too large; increase --resolution or crop the PCD')
    index = np.rint((points[:,:2]-center)/resolution).astype(int)+dims//2
    linear = index[:,0]*dims[1]+index[:,1]
    floors, ceilings, costs = [], [], []
    kernel = 7
    stand_step = 1.2*resolution*math.tan(0.4)
    for layer in range(slices):
        cut = ground+(layer+1)*slice_dh
        floor = np.full(int(np.prod(dims)), -np.inf)
        ceiling = np.full(int(np.prod(dims)), np.inf)
        below = points[:,2] <= cut
        np.maximum.at(floor, linear[below], points[below,2])
        np.minimum.at(ceiling, linear[~below], points[~below,2])
        floor = floor.reshape(tuple(dims))
        ceiling = ceiling.reshape(tuple(dims))
        observed = np.isfinite(floor)
        # Sentinel also makes known/unknown boundaries impassable.
        f = np.where(observed, floor, -1e6)
        dx = np.maximum((f-np.roll(f,1,0))**2, (f-np.roll(f,-1,0))**2)
        dy = np.maximum((f-np.roll(f,1,1))**2, (f-np.roll(f,-1,1))**2)
        grad, biggest = dx+dy, np.maximum(dx,dy)
        flat = grad <= stand_step**2
        stands = ndimage.uniform_filter(flat.astype(float), size=kernel, mode='constant')*kernel**2
        steps = (biggest <= 0.16**2) & (stands >= int(0.5*kernel**2)-1)
        clearance = ceiling-floor
        cost = np.where(flat, 15*grad/max(stand_step**2,1e-9), 20*biggest/(0.16**2))
        cost += np.maximum(0,20*(0.55-clearance))
        cost = np.where(observed & (clearance >= 0.50) & (flat | steps), np.maximum(cost,0.001),50.0)
        cost[[0,-1],:] = 50; cost[:,[0,-1]] = 50
        distance = ndimage.distance_transform_edt(cost < 50)*resolution
        cost[distance <= inflation] = 50
        band = (distance > inflation) & (distance < inflation+0.4)
        cost[band] = np.maximum(cost[band], 20*(1-(distance[band]-inflation)/0.4))
        floors.append(np.where(observed, floor, np.nan).astype(np.float32))
        ceilings.append(np.where(np.isfinite(ceiling),ceiling,np.nan).astype(np.float32))
        costs.append(cost.astype(np.float32))
    floor = np.stack(floors); ceiling = np.stack(ceilings); cost = np.stack(costs)
    gx = np.zeros_like(cost); gy = np.zeros_like(cost)
    gx[:,1:-1,:] = cost[:,2:,:]-cost[:,:-2,:]
    gy[:,:,1:-1] = cost[:,:,2:]-cost[:,:,:-2]
    return {'data':np.stack((cost,gx,gy,floor,ceiling)).astype(np.float32),
            'resolution':resolution,'center':center,'slice_h0':ground+slice_dh,'slice_dh':slice_dh}


def nearest_surface(data, point, radius=0.6, height_tolerance=0.45):
    layers = data['data']
    nx, ny = layers.shape[2:]
    res = data['resolution']
    row, col = np.rint((np.asarray(point[:2])-data['center'])/res).astype(int)+np.array([nx,ny])//2
    n = int(math.ceil(radius/res))
    r0,r1,c0,c1 = max(0,row-n),min(nx,row+n+1),max(0,col-n),min(ny,col+n+1)
    if r0 >= r1 or c0 >= c1: raise ValueError('Target is outside the scanned map')
    rs,cs = np.meshgrid(np.arange(r0,r1),np.arange(c0,c1), indexing='ij')
    x=(rs-nx//2)*res+data['center'][0]; y=(cs-ny//2)*res+data['center'][1]
    z=layers[3,:,r0:r1,c0:c1]; cost=layers[0,:,r0:r1,c0:c1]
    dxy=np.hypot(x-point[0],y-point[1])
    dz=np.abs(z-point[2])
    valid=np.isfinite(z)&(cost>0)&(cost<=20)&(dxy<=radius)&(dz<=height_tolerance)
    score=np.where(valid,dxy+dz*0.5+cost*0.0001,np.inf)
    if not np.isfinite(score).any():
        raise ValueError('No walkable surface near target on this floor; map may be sparse or blocked')
    l,rr,cc=np.unravel_index(np.argmin(score),score.shape)
    surface=np.array([x[rr,cc],y[rr,cc],z[l,rr,cc]])
    return surface, np.array([l,c0+cc,r0+rr],dtype=np.int32)


def transform_points(matrix, points):
    return np.asarray(points) @ matrix[:3,:3].T + matrix[:3,3]


def register_cloud(source, target, initial, max_distance=0.8, min_fitness=0.35, max_rmse=0.25):
    source=voxel_downsample(source,0.20)
    target=voxel_downsample(target,0.15)
    if len(source)>8000: source=source[::int(math.ceil(len(source)/8000))]
    if len(source)<100 or len(target)<100: raise ValueError('Not enough scan points for registration')
    tree=cKDTree(target); matrix=initial.copy()
    for _ in range(35):
        moved=transform_points(matrix,source)
        dist,index=tree.query(moved)
        mask=dist<max_distance
        if mask.sum()<60: raise ValueError('Initial pose too far from map; select robot position and heading again')
        cutoff=np.quantile(dist[mask],0.85)
        mask &= dist<=cutoff
        a,b=moved[mask],target[index[mask]]
        ac,bc=a.mean(axis=0),b.mean(axis=0)
        u,_,vt=np.linalg.svd((a-ac).T@(b-bc))
        rotation=vt.T@u.T
        if np.linalg.det(rotation)<0:
            vt[-1,:]*=-1; rotation=vt.T@u.T
        delta=np.eye(4);delta[:3,:3]=rotation;delta[:3,3]=bc-rotation@ac
        matrix=delta@matrix
        if np.linalg.norm(delta-np.eye(4))<1e-5:break
    distances,_=tree.query(transform_points(matrix,source))
    mask=distances<max_distance
    fitness=float(mask.mean())
    rmse=float(np.sqrt(np.mean(distances[mask]**2))) if mask.any() else math.inf
    shift=matrix@np.linalg.inv(initial)
    angle=math.acos(float(np.clip((np.trace(shift[:3,:3])-1)/2,-1,1)))
    if fitness<min_fitness or rmse>max_rmse or np.linalg.norm(shift[:3,3])>2.0 or angle>math.radians(25):
        raise ValueError('Registration rejected: fitness=%.3f RMSE=%.3f m; refine initial pose' % (fitness,rmse))
    return matrix,fitness,rmse


def prepare_map(pcd, cache_root, resolution=0.15, slice_dh=0.5):
    source=Path(pcd).resolve()
    digest=hashlib.sha256()
    with source.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):digest.update(chunk)
    options={'version':1,'resolution':resolution,'slice_dh':slice_dh,'inflation':0.20}
    digest.update(json.dumps(options,sort_keys=True).encode())
    directory=Path(cache_root)/digest.hexdigest()[:20]
    metadata=directory/'map.json'
    if metadata.exists() and (directory/'map.pickle').exists() and (directory/'points.npy').exists():
        return directory
    points=read_pcd(source)
    data=make_tomogram(points,resolution,slice_dh)
    free=int(np.count_nonzero((data['data'][0]>0)&(data['data'][0]<=20)))
    if free==0: raise ValueError('PCD produced no traversable cells; scan floor more densely or adjust resolution')
    directory.mkdir(parents=True,exist_ok=True)
    np.save(directory/'points.npy',voxel_downsample(points,0.10).astype(np.float32))
    with (directory/'map.pickle.tmp').open('wb') as f:pickle.dump(data,f,protocol=4)
    (directory/'map.pickle.tmp').replace(directory/'map.pickle')
    info={'source':str(source),'sha256_and_config':digest.hexdigest(),'options':options,
          'points':len(points),'bounds':[points.min(0).tolist(),points.max(0).tolist()],
          'shape':list(data['data'].shape),'walkable_cells':free,'frame':'navigation_map'}
    metadata.write_text(json.dumps(info,indent=2)+'\n')
    return directory
