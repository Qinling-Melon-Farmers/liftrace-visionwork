"""Bounded ground-cell observation history, independent of ROS and target truth.

Fresh occupied point clouds can reject known occlusions but cannot prove free
space. SEEN_ESTIMATE is repeated quality-filtered visibility estimation, never
navigation clearance, target recall, or a reason to declare search complete.
"""
from dataclasses import dataclass
from enum import IntEnum
from functools import lru_cache
import math
import cv2
import numpy as np


class State(IntEnum):
    UNSEEN = 0
    VISIBILITY_UNVERIFIED = 20
    LOW_QUALITY = 40
    OCCLUDED = 60
    VISIBLE_PENDING = 80
    SEEN_ESTIMATE = 100


@dataclass(frozen=True)
class Config:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    resolution: float
    ground_z: float
    frame_id: str
    max_cells: int = 20000
    max_image_age: float = .5
    future_tolerance: float = .03
    max_tf_delta: float = .05
    max_map_age: float = 2.0
    max_camera_age: float = 2.0
    memory_ttl: float = 60.0
    min_observations: int = 3
    min_dwell: float = .5
    min_observation_interval: float = .1
    max_observation_gap: float = .8
    image_margin_px: int = 16
    quality_window_px: int = 31
    min_laplacian_variance: float = 10.0
    max_dark_fraction: float = .6
    max_bright_fraction: float = .6
    dark_level: int = 8
    bright_level: int = 247
    occlusion_tile_px: int = 8
    occlusion_margin_m: float = .08
    max_cloud_points: int = 500000
    flat_ground_reference_enabled: bool = False
    min_reference_fraction: float = .02
    min_reference_tiles: int = 3
    projection_chunk_points: int = 16384
    max_image_pixels: int = 2073600

    def __post_init__(self):
        for name, value in vars(self).items():
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError('nonfinite config: '+name)
        if not self.frame_id or self.min_x >= self.max_x or self.min_y >= self.max_y:
            raise ValueError('invalid frame/bounds')
        positive = ['resolution', 'max_image_age', 'max_map_age', 'max_camera_age',
                    'memory_ttl', 'min_observation_interval', 'max_observation_gap']
        if any(getattr(self, key) <= 0 for key in positive):
            raise ValueError('positive durations/resolution required')
        for key in ['max_cells', 'min_observations', 'quality_window_px', 'occlusion_tile_px',
                    'max_cloud_points', 'projection_chunk_points', 'max_image_pixels']:
            value = getattr(self, key)
            if int(value) != value or value <= 0:
                raise ValueError('positive integer required: '+key)
        if self.quality_window_px % 2 != 1 or self.min_observations < 2:
            raise ValueError('odd quality window and multiple observations required')
        if self.min_dwell < 0 or self.min_observation_interval > self.max_observation_gap:
            raise ValueError('invalid observation interval')
        if min(self.future_tolerance, self.max_tf_delta, self.image_margin_px,
               self.occlusion_margin_m, self.min_laplacian_variance) < 0:
            raise ValueError('negative tolerance')
        if not (0 <= self.max_dark_fraction <= 1 and 0 <= self.max_bright_fraction <= 1):
            raise ValueError('invalid exposure fractions')
        if not 0 <= self.dark_level < self.bright_level <= 255:
            raise ValueError('invalid exposure thresholds')
        if not 0 < self.min_reference_fraction <= 1 or not 1 <= self.min_reference_tiles <= 16:
            raise ValueError('invalid sharp-reference limits')


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    k: tuple
    d: tuple
    frame_id: str
    distortion_model: str = 'plumb_bob'

    def __post_init__(self):
        k = np.asarray(self.k, dtype=float)
        if (self.width <= 0 or self.height <= 0 or len(k) != 9 or
                not np.isfinite(k).all() or not np.isfinite(self.d).all() or
                k[0] <= 0 or k[4] <= 0 or not self.frame_id):
            raise ValueError('invalid camera calibration')
        if not np.allclose(k[[1, 3, 6, 7, 8]], [0, 0, 0, 0, 1]):
            raise ValueError('unsupported intrinsic matrix')
        if self.distortion_model not in ('plumb_bob', 'rational_polynomial') or len(self.d) not in (0, 4, 5, 8):
            raise ValueError('unsupported distortion model')


@lru_cache(maxsize=1)
def _ray_domain(width, height, intrinsics, distortion):
    """One calibration entry only; cached data never include images or maps."""
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    d = np.asarray(distortion, dtype=float) if distortion else None
    xs, ys = np.linspace(0, width-1, 33), np.linspace(0, height-1, 33)
    border = np.concatenate([np.column_stack([xs, np.zeros(33)]),
        np.column_stack([xs, np.full(33, height-1)]),
        np.column_stack([np.zeros(33), ys]), np.column_stack([np.full(33, width-1), ys])])
    rays = cv2.undistortPoints(border.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    if not np.isfinite(rays).all():
        raise ValueError('invalid calibrated ray domain')
    return k, d, rays.min(axis=0), rays.max(axis=0)


class Memory:
    def __init__(self, config):
        self.config = config
        c = config
        self.nx = int(math.ceil((c.max_x-c.min_x)/c.resolution-1e-9))
        self.ny = int(math.ceil((c.max_y-c.min_y)/c.resolution-1e-9))
        if self.nx*self.ny > c.max_cells:
            raise ValueError('grid exceeds configured cell bound')
        xe = np.minimum(c.min_x+np.arange(self.nx+1)*c.resolution, c.max_x)
        ye = np.minimum(c.min_y+np.arange(self.ny+1)*c.resolution, c.max_y)
        x, y = np.meshgrid((xe[:-1]+xe[1:])/2, (ye[:-1]+ye[1:])/2)
        dx, dy = np.meshgrid(np.diff(xe), np.diff(ye))
        self.centers = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, c.ground_z)])
        self.area = (dx*dy).ravel()
        self.samples = np.repeat(self.centers[:, None, :], 5, axis=1)
        for i, (sx, sy) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)], 1):
            self.samples[:, i, 0] += sx*.49*dx.ravel()
            self.samples[:, i, 1] += sy*.49*dy.ravel()
        self.generation = 0
        self.epoch = ''
        self.reset('initial')

    def reset(self, reason, epoch=None):
        self.generation += 1
        if epoch is not None:
            self.epoch = str(epoch)
        n = len(self.centers)
        self.state = np.zeros(n, dtype=np.int8)
        self.last_seen = np.full(n, -np.inf)
        self.last_good = np.full(n, -np.inf)
        self.first_good = np.full(n, -np.inf)
        self.count = np.zeros(n, dtype=np.int32)
        self.last_stamp = -np.inf
        self.camera = None
        self.reset_reason = reason
        self.last_now = -np.inf

    def expire(self, now):
        expired = now-self.last_seen > self.config.memory_ttl
        self.last_seen[expired] = -np.inf
        self.state[expired & (self.state == State.SEEN_ESTIMATE)] = State.UNSEEN

    def reject(self, reason, now):
        self.expire(now)
        self.count.fill(0)
        return self.summary(now, accepted=False, reason=reason)

    def summary(self, now, **extra):
        self.expire(now)
        result = {'epoch': self.epoch, 'generation': self.generation,
                  'reset_reason': self.reset_reason, 'frame_id': self.config.frame_id,
                  'estimated_seen_area_m2': float(self.area[np.isfinite(self.last_seen)].sum()),
                  'grid_area_m2': float(self.area.sum()),
                  'state_area_m2': {s.name: float(self.area[self.state == s].sum()) for s in State},
                  'scope': 'SHADOW_ONLY_VISIBILITY_ESTIMATE_NOT_FREE_SPACE_OR_TARGET_RECALL'}
        result.update(extra)
        return result

    @staticmethod
    def _project(points, camera, camera_to_map):
        local = (points-camera_to_map[:3, 3]) @ camera_to_map[:3, :3]
        uv = np.full((len(local), 2), -1e9)
        front = local[:, 2] > 1e-5
        # Brown distortion is calibrated over the sensor's field of view, not
        # arbitrary rays. Polynomial extrapolation can fold distant ground
        # points back INSIDE image bounds and falsely grow the footprint.
        k, d, lower, upper = _ray_domain(camera.width, camera.height, tuple(camera.k), tuple(camera.d))
        normalized = local[:,:2]/np.maximum(local[:,2,None],1e-5)
        front &= ((normalized >= lower) & (normalized <= upper)).all(axis=1)
        if front.any():
            uv[front] = cv2.projectPoints(local[front], np.zeros(3), np.zeros(3),
                k, d)[0].reshape(-1,2)
        return uv, local[:, 2]

    def observe(self, image, camera, camera_to_map, stamp, now, tf_stamp,
                camera_stamp, image_frame, map_points=None, map_stamp=None, map_frame=None):
        c = self.config
        if not np.isfinite([stamp, now, tf_stamp, camera_stamp]).all() or stamp <= 0 or now <= 0:
            return self.reject('invalid_time', max(0, now) if math.isfinite(now) else 0)
        if now < self.last_now-c.future_tolerance:
            self.reset('clock_rollback')
        self.last_now = now
        if stamp <= self.last_stamp:
            return self.reject('duplicate_or_out_of_order_image', now)
        if not -c.future_tolerance <= now-stamp <= c.max_image_age:
            return self.reject('image_stale_or_future', now)
        if not -c.future_tolerance <= stamp-camera_stamp <= c.max_camera_age:
            return self.reject('calibration_stale_or_future', now)
        if tf_stamp != 0 and abs(stamp-tf_stamp) > c.max_tf_delta:
            return self.reject('tf_time_mismatch', now)
        if image_frame != camera.frame_id:
            return self.reject('camera_frame_mismatch', now)
        if camera.width*camera.height > c.max_image_pixels:
            return self.reject('image_pixel_limit', now)
        if image.dtype != np.uint8 or image.shape not in [(camera.height,camera.width), (camera.height,camera.width,3)]:
            return self.reject('image_shape_or_encoding', now)
        transform = np.asarray(camera_to_map, dtype=float)
        if (transform.shape != (4,4) or not np.isfinite(transform).all() or
                not np.allclose(transform[3], [0,0,0,1]) or
                not np.allclose(transform[:3,:3].T @ transform[:3,:3], np.eye(3), atol=1e-5) or
                abs(np.linalg.det(transform[:3,:3])-1) > 1e-5):
            return self.reject('invalid_transform', now)
        if transform[2,3] <= c.ground_z:
            return self.reject('camera_not_above_ground', now)
        if self.camera is not None and self.camera != camera:
            self.reset('calibration_changed')
        self.camera = camera
        self.last_now = now
        self.last_stamp = stamp
        self.expire(now)
        before = np.isfinite(self.last_seen)
        uv, depth = self._project(self.samples.reshape(-1,3), camera, transform)
        uv = uv.reshape(-1,5,2); depth = depth.reshape(-1,5)
        m = c.image_margin_px
        inside = ((uv[:,:,0] >= m) & (uv[:,:,0] < camera.width-m) &
                  (uv[:,:,1] >= m) & (uv[:,:,1] < camera.height-m) & (depth > 0)).all(axis=1)
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        box = (c.quality_window_px,)*2
        lap = cv2.Laplacian(gray, cv2.CV_32F)
        mean = cv2.boxFilter(lap, -1, box)
        np.square(mean, out=mean)
        np.square(lap, out=lap)
        variance = cv2.boxFilter(lap, -1, box)
        variance -= mean
        del mean, lap
        exposure = ((cv2.boxFilter((gray <= c.dark_level).astype(np.float32), -1, box) <= c.max_dark_fraction) &
                    (cv2.boxFilter((gray >= c.bright_level).astype(np.float32), -1, box) <= c.max_bright_fraction))
        local_sharp = variance >= c.min_laplacian_variance
        reference = local_sharp & exposure
        tile_support = sum(tile.size > 0 and float(tile.mean()) >= .05 for band in np.array_split(reference,4,axis=0)
                           for tile in np.array_split(band,4,axis=1))
        reference_available = bool(reference.mean() >= c.min_reference_fraction and tile_support >= c.min_reference_tiles)
        # Low texture alone does not demonstrate blur. Opt-in research mode
        # permits exposure-valid flat cells when spatially distributed sharp
        # references exist in this SAME image. Still only a visibility estimate.
        use_reference = c.flat_ground_reference_enabled and reference_available
        quality = exposure & (local_sharp | use_reference)
        good_quality = np.zeros(len(self.centers), dtype=bool)
        ids = np.where(inside)[0]
        pixels = uv[ids].astype(int)
        good_quality[ids] = quality[pixels[:,:,1], pixels[:,:,0]].all(axis=1)
        local_quality = np.zeros(len(self.centers),dtype=bool)
        local_quality[ids] = reference[pixels[:,:,1],pixels[:,:,0]].all(axis=1)
        fresh_map = (map_stamp is not None and math.isfinite(map_stamp) and map_stamp > 0 and
                     map_frame == c.frame_id and -c.future_tolerance <= stamp-map_stamp <= c.max_map_age)
        points = np.asarray(map_points, dtype=float) if map_points is not None else np.empty((0,3))
        fresh_map = bool(fresh_map and points.ndim == 2 and points.shape[1:] == (3,) and
                         0 < len(points) <= c.max_cloud_points and np.isfinite(points).all())
        blocked = np.zeros(len(self.centers), dtype=bool)
        if fresh_map:
            tile = c.occlusion_tile_px
            tw, th = math.ceil(camera.width/tile), math.ceil(camera.height/tile)
            zbuffer = np.full(tw*th, 1e6, dtype=np.float32)
            # Visit EVERY point, in bounded chunks. No downsampling or missing
            # obstacle evidence; take the same global minimum before splatting.
            for start in range(0, len(points), c.projection_chunk_points):
                puv, pz = self._project(points[start:start+c.projection_chunk_points], camera, transform)
                valid = ((pz > 0) & (puv[:,0] >= 0) & (puv[:,0] < camera.width) &
                         (puv[:,1] >= 0) & (puv[:,1] < camera.height))
                loc = (puv[valid]/tile).astype(int)
                np.minimum.at(zbuffer, loc[:,1]*tw+loc[:,0], pz[valid])
            # Conservative neighboring tile splat rejects rays near known surfaces.
            zbuffer = cv2.erode(zbuffer.reshape(th,tw), np.ones((3,3),np.uint8))
            loc = (uv[ids]/tile).astype(int)
            blocked[ids] = (zbuffer[loc[:,:,1],loc[:,:,0]] < depth[ids]-c.occlusion_margin_m).any(axis=1)
        else:
            # No fresh observation of obstacles: retain no active visibility credit.
            self.last_seen.fill(-np.inf)
        self.last_seen[blocked] = -np.inf
        candidate = inside & good_quality & ~blocked & fresh_map
        previous = self.last_good.copy()
        gap = stamp-previous
        restart = candidate & ((gap > c.max_observation_gap) | (self.count == 0))
        self.count[~candidate] = 0
        self.count[restart] = 0
        self.first_good[restart] = stamp
        increment = candidate & (gap >= c.min_observation_interval)
        self.count[increment] = np.minimum(self.count[increment]+1, c.min_observations)
        self.last_good[increment] = stamp
        qualified = candidate & (self.count >= c.min_observations) & (stamp-self.first_good >= c.min_dwell)
        self.last_seen[qualified] = stamp
        seen = np.isfinite(self.last_seen)
        self.state.fill(State.UNSEEN)
        self.state[inside] = State.VISIBILITY_UNVERIFIED
        self.state[inside & ~good_quality] = State.LOW_QUALITY
        self.state[blocked] = State.OCCLUDED
        self.state[candidate] = State.VISIBLE_PENDING
        self.state[seen] = State.SEEN_ESTIMATE
        return self.summary(now, accepted=True, reason='observed', map_fresh=fresh_map,
            quality_reference_available=reference_available,
            quality_mode='same_image_sharp_reference' if c.flat_ground_reference_enabled else 'local_variance_only',
            reference_supported_area_m2=float(self.area[inside & good_quality & ~local_quality].sum()),
            newly_estimated_area_m2=float(self.area[qualified & ~before].sum()),
            revisited_estimated_area_m2=float(self.area[qualified & before].sum()),
            footprint_area_m2=float(self.area[inside].sum()), image_stamp=stamp)
