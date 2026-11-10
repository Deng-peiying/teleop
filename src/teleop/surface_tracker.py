"""Experimental continuous surface correspondence, separate from the frozen baseline."""
import numpy as np
import mujoco as mj

from .thumb_patch import AutomaticPatch
from .hand2_geometry import nearest_arc


class TrackedSurfacePatch(AutomaticPatch):
    """A2: local correspondence; A3: tangent increments; A4: surface normals.

    Geometry-only confidence remains a model hypothesis, not measured contact.
    No historical minimum gap or contact latch is used. Release is immediate.
    """
    normal_loss_band_m = np.inf

    def __init__(self, method, stage=4, calibration=None):
        super().__init__(method)
        self.stage = stage
        self.selected_finger = None
        self.local_history = {}
        self.previous_targets = {}
        self.previous_outputs = {}
        self.frame_targets = {}
        self.increment_targets = {}
        self.cached = None
        self.identity_switches = 0
        self.local_reacquisitions = 0
        self.increment_error_m = None
        self.tangent_speed_m_s = None
        self.calibrated = calibration is not None
        self.effective_offset_m = 0.
        if calibration is not None:
            if calibration.get('status') != 'measured' or not calibration.get('evidence'):
                raise ValueError('A5 requires measured calibration and evidence')
            self.skin_offset_m = float(calibration['human_skin_offset_m'])
            self.effective_offset_m = float(calibration['robot_effective_offset_m'])
            if not 0 < self.skin_offset_m < .03 or abs(self.effective_offset_m) > .01:
                raise ValueError('Calibration values outside supported range')
        # Mesh samples are prepared once. No full mesh-pair nearest query in
        # this variant's objective or correspondence update.
        self.thumb_grid = []
        for t in np.linspace(.23, .9, 12):
            for a in np.linspace(-1.25, 1.25, 19):
                try:
                    p, n = self.surface._ray(self.geoms[0], t*self.ends[0],
                                             [np.sin(a), -np.cos(a), 0.])
                    self.thumb_grid.append((p, n))
                except ValueError:
                    pass
        if not self.thumb_grid:
            raise ValueError('No usable thumb surface samples')

    def update_anchors(self):
        # Parent computes continuously observed local (segment,s,theta), then
        # calls this hook instead of reinitializing global nearest witnesses.
        self.anchors = []
        self.frame_targets = {}
        self.increment_targets = {}
        self.cached = None
        scores = self.weights.reshape(4, 3).sum(axis=1)
        best = int(np.argmax(scores))
        if self.strength == 0 or scores.max() == 0:
            self.local_history.clear()
            self.previous_targets.clear()
            self.previous_outputs.clear()
            self.selected_finger = None
            return
        old = self.selected_finger
        if old is not None and scores[old] > 0 and scores[best] < scores[old] + .2:
            best = old
        if old is not None and old != best:
            self.identity_switches += 1
            self.local_history.clear()
            self.previous_targets.clear()
            self.previous_outputs.clear()
        self.selected_finger = best
        keep = [(k, w, sample) for k, (w, sample) in zip(self.sample_segments, self.samples) if k//3 == best]
        support = sum(w for _,w,_ in keep)
        self.sample_segments = [k for k,_,_ in keep]
        self.samples = [(w/max(support,1e-12),sample) for _,w,sample in keep]
        body = self.bodies[0]
        rotation = self.surface.data.xmat[body].reshape(3,3)
        origin = self.surface.data.xpos[body]
        for k, (_, sample) in zip(self.sample_segments, self.samples):
            (t, nt), (i, ni) = self.world(sample)
            previous = self.local_history.get(k)
            if previous is not None and np.linalg.norm(previous[0]-sample[0][1]) > .012:
                previous = None
                self.previous_targets.pop(k,None)
                self.previous_outputs.pop(k,None)
                self.local_reacquisitions += 1
            candidates = self.thumb_grid + [(sample[0][1], sample[0][2])]
            if previous is not None:
                local = [(p,n) for p,n in candidates if np.linalg.norm(p-previous[0]) <= .005]
                candidates = [previous] + local
            def score(candidate):
                p,n = candidate
                point = origin+rotation@p
                return np.sum((point-i)**2) + .15*np.sum((point-t)**2)
            point, normal = min(candidates, key=score)
            self.local_history[k] = (point.copy(), normal.copy())
            self.anchors.append((point.copy(), normal.copy()))
            # Store in the target body's coordinates; rigid finger motion does
            # not masquerade as surface slip. Never compare across body IDs.
            target_local = sample[1][1].copy()
            self.frame_targets[k] = target_local
            if k in self.previous_targets and k in self.previous_outputs:
                self.increment_targets[k] = self.previous_outputs[k] + target_local-self.previous_targets[k]
        if self.samples:
            world = [self.world(sample) for _,sample in self.samples]
            desired_increment = []
            for k,(_,sample),target in zip(self.sample_segments,self.samples,world):
                target_body = sample[1][0]
                desired_increment.append(self.surface.data.xpos[target_body] +
                    self.surface.data.xmat[target_body].reshape(3,3) @ self.increment_targets[k]
                    if k in self.increment_targets else target[1][0])
            self.cached = dict(local=np.array([p for p,n in self.anchors]),
                normals=np.array([n for p,n in self.anchors]),
                target=np.array([w[1][0] for w in world]),
                target_normal=np.array([w[1][1] for w in world]),
                region=np.array([sample[0][1] for _,sample in self.samples]),
                desired=np.maximum(self.gaps[self.sample_segments]+self.effective_offset_m,.0002),
                weights=np.array([w for w,_ in self.samples]),
                increments=np.array(desired_increment),
                increment_mask=np.array([k in self.increment_targets for k in self.sample_segments]))

    def cost(self):
        c = self.cached
        if c is None or self.strength == 0:
            return 0.
        body = self.bodies[0]
        rotation = self.surface.data.xmat[body].reshape(3,3)
        points = self.surface.data.xpos[body]+c['local']@rotation.T
        delta = points-c['target'];ni=c['target_normal']
        gap = np.sum(delta*ni,axis=1)
        tangent = delta-gap[:,None]*ni
        region = (c['local']-c['region'])@rotation.T
        normal_residual = gap-c['desired']
        clipped = np.clip(normal_residual,-self.normal_loss_band_m,self.normal_loss_band_m)
        errors = clipped*(2*normal_residual-clipped)/.001**2 + np.sum(tangent**2,axis=1)/.006**2
        errors += (np.maximum(0.,np.linalg.norm(region,axis=1)-.006)/.008)**2
        if self.stage >= 3:
            slip=points-c['increments']
            slip-=np.sum(slip*ni,axis=1)[:,None]*ni
            errors+=np.sum(slip**2,axis=1)/.004**2*c['increment_mask']
        if self.stage >= 4:
            errors+=.15*np.sum((c['normals']@rotation.T+ni)**2,axis=1)/.7**2
        return float(self.strength*(c['weights']@errors))

    def finalize(self):
        outputs = {}
        errors=[];speeds=[]
        for k, (_,sample), (local,_) in zip(self.sample_segments,self.samples,self.anchors):
            body = self.bodies[0]
            point = self.surface.data.xpos[body]+self.surface.data.xmat[body].reshape(3,3)@local
            target_body = sample[1][0]
            outputs[k] = self.surface.data.xmat[target_body].reshape(3,3).T @ (point-self.surface.data.xpos[target_body])
            if k in self.previous_targets and k in self.previous_outputs:
                desired=self.frame_targets[k]-self.previous_targets[k]
                actual=outputs[k]-self.previous_outputs[k]
                normal=sample[1][2]
                error=actual-desired
                error-=(error@normal)*normal
                errors.append(np.linalg.norm(error))
                speeds.append(np.linalg.norm(desired-(desired@normal)*normal)/self.method.dt)
        self.increment_error_m=float(np.mean(errors)) if errors else None
        self.tangent_speed_m_s=float(np.mean(speeds)) if speeds else None
        self.previous_targets = {k:p.copy() for k,p in self.frame_targets.items()}
        self.previous_outputs = outputs

    def gradient(self):
        c=self.cached
        if c is None or self.strength==0:return np.zeros(4)
        s=self.surface;body=self.bodies[0];rot=s.data.xmat[body].reshape(3,3)
        jp=np.empty((3,s.model.nv));jr=np.empty_like(jp)
        mj.mj_jac(s.model,s.data,jp,jr,s.data.xpos[body],body)
        jp=jp[:,s.thumb_dof];jr=jr[:,s.thumb_dof]
        radii=c['local']@rot.T
        points=s.data.xpos[body]+radii
        jac=jp[None,:,:]+np.cross(jr.T[None,:,:],radii[:,None,:]).transpose(0,2,1)
        ni=c['target_normal'];delta=points-c['target']
        gaps=np.sum(delta*ni,axis=1);tangent=delta-gaps[:,None]*ni
        force=2*np.clip(gaps-c['desired'],-self.normal_loss_band_m,self.normal_loss_band_m)[:,None]*ni/.001**2+2*tangent/.006**2
        if self.stage>=3:
            slip=points-c['increments'];slip-=np.sum(slip*ni,axis=1)[:,None]*ni
            force+=2*slip/.004**2*c['increment_mask'][:,None]
        grads=np.einsum('ni,nij->nj',force,jac)
        if self.stage>=4:
            nt=c['normals']@rot.T
            jac_n=np.cross(jr.T[None,:,:],nt[:,None,:]).transpose(0,2,1)
            grads+=.3*np.einsum('ni,nij->nj',nt+ni,jac_n)/.7**2
        return self.strength*np.sum(c['weights'][:,None]*grads,axis=0)

    def diagnostic(self):
        result = super().diagnostic()
        result.update(tracker_stage=self.stage, calibrated=self.calibrated,
                      identity_switches=self.identity_switches,
                      local_reacquisitions=self.local_reacquisitions,
                      correspondence_source='local_mesh_neighborhood',
                      local_thumb_points=[p.tolist() for p,n in self.anchors],
                      tangent_increment_error_m=self.increment_error_m,
                      mapped_tangent_speed_m_s=self.tangent_speed_m_s,
                      metric_kind='model-correspondence proxy; not video-verified material slip',
                      tangent_increment_support=len(self.increment_targets))
        return result


class ContinuousSurfacePatch(TrackedSurfacePatch):
    """B4: continuous on-mesh UV updates and bounded tangent-history correction.

    The sample grid proposes a local correction; it no longer directly
    becomes the material correspondence used by the optimizer.
    """
    normal_loss_band_m = .002
    preserve_sdk_root = True

    def contact_progress(self, distances):
        # Zero throughout predicted separation, then ramp over 2 mm inside
        # the estimated skin envelope. This is a geometry proxy.
        return np.clip((self.skin_offset_m-distances)/.002,0.,1.)

    def __init__(self, method):
        super().__init__(method,stage=4)
        self.uv_history={}
        self.source_history={}
        self.correspondence_step_m=0.
        self.index_arc = None
        self.index_return_release = False
        self.limit_return_rebound = False

    def prepare(self, kp, rotations):
        was_on_index = self.selected_finger == 0 and self.strength > 0
        super().prepare(kp, rotations)
        self.limit_return_rebound = False
        if self.selected_finger != 0 and not (was_on_index or self.index_return_release):
            self.index_arc = None
            return
        arc, _, _ = nearest_arc(.4*kp[3]+.6*kp[4], kp[5:9])
        delta = 0. if self.index_arc is None else arc-self.index_arc
        self.index_arc = arc
        if self.strength > 0 or delta > .003:
            self.index_return_release = False
        elif was_on_index and delta < -.003:
            self.index_return_release = True
        self.limit_return_rebound = self.index_return_release and delta < -.001

    def update_anchors(self):
        old_history=self.local_history.copy()
        old_finger=self.selected_finger
        super().update_anchors()
        if not self.strength or self.cached is None:
            self.strength = 0.
            self.uv_history.clear();self.source_history.clear()
            self.correspondence_step_m=0.
            return
        if old_finger!=self.selected_finger:
            self.uv_history.clear();self.source_history.clear()
            old_history={}
        updated=[];steps=[]
        for k,(local,normal),(_,sample) in zip(self.sample_segments,self.anchors,self.samples):
            fraction=local[2]/self.ends[0][2]
            center=fraction*self.ends[0]
            angle=np.arctan2(local[0]-center[0],-(local[1]-center[1]))
            desired=np.array([fraction,angle])
            source=self.parameters[k,[0,2]].copy()
            if k in self.uv_history and k in old_history:
                motion=source-self.source_history[k]
                motion[1]=(motion[1]+np.pi)%(2*np.pi)-np.pi
                prediction=self.uv_history[k]+motion
                correction=desired-prediction
                correction[1]=(correction[1]+np.pi)%(2*np.pi)-np.pi
                # Bounded local correction, with direct source feed-forward.
                desired=prediction+np.clip(.4*correction,[-.02,-.08],[.02,.08])
            desired=np.clip(desired,[.23,-1.25],[.9,1.25])
            try:
                point,n=self.surface._ray(self.geoms[0],desired[0]*self.ends[0],
                                         [np.sin(desired[1]),-np.cos(desired[1]),0.])
            except ValueError:
                if k in old_history:
                    point,n=old_history[k]
                    desired=self.uv_history.get(k,desired)
                else:
                    point,n=local,normal
            updated.append((point.copy(),n.copy()))
            if k in old_history:steps.append(np.linalg.norm(point-old_history[k][0]))
            self.local_history[k]=(point.copy(),n.copy())
            self.uv_history[k]=desired.copy();self.source_history[k]=source
        self.anchors=updated
        self.correspondence_step_m=float(max(steps,default=0))
        self.cached['local']=np.array([p for p,n in updated])
        self.cached['normals']=np.array([n for p,n in updated])
        for j,(k,(_,sample)) in enumerate(zip(self.sample_segments,self.samples)):
            if k not in self.increment_targets:continue
            # Retain incremental intent without indefinitely retaining previous
            # output error. Large correspondence residuals cannot dominate.
            error=self.previous_outputs[k]-self.previous_targets[k]
            ni=sample[1][2];error-=np.dot(error,ni)*ni
            error*=min(1.,.002/max(np.linalg.norm(error),1e-12))
            target=self.frame_targets[k]+.25*error
            self.increment_targets[k]=target
            body=sample[1][0]
            self.cached['increments'][j]=self.surface.data.xpos[body]+self.surface.data.xmat[body].reshape(3,3)@target

    def diagnostic(self):
        result=super().diagnostic()
        result.update(correspondence_source='continuous_on_mesh_uv',
                      correspondence_step_m=self.correspondence_step_m,
                      bounded_tangent_history=True)
        return result
