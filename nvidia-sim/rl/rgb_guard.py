"""Near-contact stop decision from RGB tracks and calibrated camera poses only.

RGB parallax during a short, initially static approach estimates feature
positions. These are estimates, NOT renderer depth / object truth. A frozen
estimate predicts ego-motion; persistent residuals flag motion or model
inconsistency. It cannot identify a contacted stem or certify a hook.
"""
import cv2
import numpy as np

from rgb_camera import project


class RGBMotionGuard:
    def __init__(self, intrinsics, residual_px=4., persistence=3, min_tracks=8):
        self.k = np.asarray(intrinsics, dtype=float)
        self.threshold = residual_px
        self.persistence = persistence
        self.min_tracks = min_tracks
        self.hits = 0
        self.frames = 0
        self.model = None
        self.last = dict(stop=False, reason='not_initialized')

    def initialize(self, rgb, roi, world_from_camera):
        x, y, w, h = map(int, roi)
        if min(w,h) <= 0 or x < 0 or y < 0 or x+w > rgb.shape[1] or y+h > rgb.shape[0]:
            raise ValueError('Target ROI is outside the RGB image')
        self.previous = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        mask = np.zeros(self.previous.shape, dtype=np.uint8)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        # Operator selects the fruit + its calyx. Saturation suppresses grey
        # greenhouse/tool background; this is NOT automatic target detection.
        # Smooth fruit alone often has too few trackable RGB corners.
        colored = (hsv[:,:,1] > 65) & (hsv[:,:,2] > 35)
        mask[y:y+h,x:x+w] = colored[y:y+h,x:x+w]*255
        mask = cv2.erode(mask, np.ones((3,3), np.uint8))
        points = cv2.goodFeaturesToTrack(self.previous, 100, .006, 3, mask=mask, blockSize=5)
        self.points = np.empty((0,1,2), np.float32) if points is None else points
        self.initial_pixels = self.points[:,0].copy()
        self.initial_pose = np.asarray(world_from_camera).copy()
        self.previous_pose = self.initial_pose.copy()
        self.flow_window = []
        self.pixel_history = [self.initial_pixels.copy()]
        self.pose_history = [self.initial_pose.copy()]
        self.roi = (x,y,w,h)
        self.frames = 0
        self.model = None
        self.hits = 0
        self.last = dict(stop=False, reason='calibrating', tracks=len(self.points), residual_px=None)
        self.visual = dict(observed_pixels=self.points[:,0].tolist())

    def _stop(self, reason, **kwargs):
        self.last = dict(stop=True, reason=reason, tracks=len(self.points), residual_px=None, **kwargs)
        return self.last.copy()

    def observe(self, rgb, world_from_camera):
        if self.last['stop']:
            return self.last.copy()
        self.visual = {}
        self.frames += 1
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        if len(self.points) < self.min_tracks:
            return self._stop('insufficient_visible_features')
        lk = dict(winSize=(21,21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        tracked, status, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, self.points, None, **lk)
        if tracked is None:
            return self._stop('tracking_lost')
        back, backward_status, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous, tracked, None, **lk)
        if back is None:
            return self._stop('tracking_lost')
        pixels = tracked[:,0]
        previous_pixels = self.points[:,0].copy()
        previous_pose = self.previous_pose.copy()
        valid = (status[:,0] != 0) & (backward_status[:,0] != 0)
        valid &= np.linalg.norm(back[:,0]-self.points[:,0], axis=1) < 1.
        valid &= np.isfinite(pixels).all(axis=1)
        valid &= (pixels[:,0] > 4) & (pixels[:,0] < gray.shape[1]-4) & (pixels[:,1] > 4) & (pixels[:,1] < gray.shape[0]-4)
        self.points = tracked[valid]
        previous_pixels = previous_pixels[valid]
        self.initial_pixels = self.initial_pixels[valid]
        if self.model is None:
            self.pixel_history = [pixels[valid] for pixels in self.pixel_history]
        if self.model is not None:
            self.model = self.model[valid]
        self.previous = gray
        self.visual = dict(previous_pixels=previous_pixels.tolist(), observed_pixels=self.points[:,0].tolist())
        if len(self.points) < self.min_tracks:
            return self._stop('tracking_lost')
        pose = np.asarray(world_from_camera)
        self.previous_pose = pose.copy()
        if self.model is None:
            self.pixel_history.append(self.points[:,0].copy())
            self.pose_history.append(pose.copy())
            baseline = float(np.linalg.norm(pose[:3,3]-self.initial_pose[:3,3]))
            if baseline >= .030:
                equations = []
                for pixels, camera_pose in zip(self.pixel_history,self.pose_history):
                    projection = self.k@np.linalg.inv(camera_pose)[:3]
                    equations.extend([pixels[:,0,None]*projection[2]-projection[0],
                                      pixels[:,1,None]*projection[2]-projection[1]])
                _,_,vt = np.linalg.svd(np.stack(equations,axis=1),full_matrices=False)
                homogeneous = vt[:,-1,:]
                world = homogeneous[:,:3]/homogeneous[:,3:]
                p0, z0 = project(world, self.initial_pose, self.k)
                p1, z1 = project(world, pose, self.k)
                rays0 = world-self.initial_pose[:3,3]
                rays1 = world-pose[:3,3]
                cosine = np.sum(rays0*rays1,axis=1)/(np.linalg.norm(rays0,axis=1)*np.linalg.norm(rays1,axis=1))
                parallax = np.arccos(np.clip(cosine,-1,1))
                valid = np.isfinite(world).all(axis=1) & (z0 > .05) & (z0 < .8) & (z1 > .05) & (z1 < .8)
                valid &= (np.linalg.norm(p0-self.initial_pixels,axis=1) < 1.) & (np.linalg.norm(p1-self.points[:,0],axis=1) < 1.) & (parallax > .008)
                if valid.sum() >= self.min_tracks:
                    self.model = world[valid]
                    self.points = self.points[valid]
                    previous_pixels = previous_pixels[valid]
                    self.initial_pixels = self.initial_pixels[valid]
            if self.model is None:
                if self.frames >= 60:
                    return self._stop('unobservable_motion_model', baseline_m=baseline)
                self.last = dict(stop=False, reason='calibrating', tracks=len(self.points), residual_px=None, baseline_m=baseline)
                return self.last.copy()
        expected, depths = project(self.model, pose, self.k)
        if (depths <= .01).any():
            return self._stop('invalid_motion_prediction')
        expected_previous, _ = project(self.model, previous_pose, self.k)
        # Use short-horizon flow innovations rather than absolute projection
        # errors: small monocular depth errors grow over long approaches even
        # for a static fruit. Keep the estimate frozen so motion is not fitted away.
        errors = (self.points[:,0]-previous_pixels)-(expected-expected_previous)
        # Diagnostic output only. These are the exact tracks used for this
        # decision, not a second pass through compressed/annotated video.
        self.visual = dict(previous_pixels=previous_pixels.tolist(),
                           observed_pixels=self.points[:,0].tolist(),
                           predicted_pixels=(previous_pixels+expected-expected_previous).tolist(),
                           innovation_vectors_px=errors.tolist())
        innovation = np.median(errors, axis=0)
        self.flow_window = (self.flow_window+[innovation])[-6:]
        shift = np.sum(self.flow_window,axis=0)
        residual = float(np.linalg.norm(shift))
        coherence = float(np.mean(np.linalg.norm(errors-innovation,axis=1) < 1.5))
        self.hits = self.hits+1 if residual > self.threshold else 0
        reason = 'motion_or_model_mismatch' if self.hits >= self.persistence else 'tracking'
        stop = self.hits >= self.persistence
        if coherence < .45:
            stop, reason = True, 'inconsistent_visual_tracks'
        self.last = dict(stop=stop, reason=reason, tracks=len(self.points), residual_px=residual,
                         residual_xy_px=shift.tolist(), coherence=coherence, persistence=self.hits,
                         flow_innovation_px=innovation.tolist(),
                         static_reprojection_px=float(np.linalg.norm(np.median(self.points[:,0]-expected,axis=0))))
        return self.last.copy()

    def annotate(self, rgb, draw_tracks=True):
        image = rgb.copy()
        if draw_tracks:
            for point in self.points[:,0]:
                cv2.circle(image, tuple(np.round(point).astype(int)), 2, (0,255,255), -1)
        text = f"{self.last['reason']} | tracks={len(self.points)}"
        if not draw_tracks:
            text = 'Latched decision: '+self.last['reason']
        residual = self.last.get('residual_px')
        if residual is not None:
            text += f' | residual={residual:.1f}px'
        cv2.putText(image, text, (10,45), cv2.FONT_HERSHEY_SIMPLEX, .45,
                    (255,100,100) if self.last['stop'] else (100,255,100), 1)
        return image
