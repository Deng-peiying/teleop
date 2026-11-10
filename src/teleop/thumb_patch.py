"""Moving mesh patch representatives for a single confirmed self-touch episode."""
import mujoco as mj
import numpy as np

from .hand2_geometry import segment_segment


def mark_surfaces(scene, nearest, candidate=()):
    # Blue = intended surface samples. Amber = separated closest witnesses.
    # Red is reserved for actual mesh intersection/touch (signed gap <= 0).
    closest_color = [1., .18, .12, 1.] if nearest['gap_m'] <= 0 else [1., .66, .10, 1.]
    for points, color, radius in ((candidate, [.15,.8,1.,1.], .0015),
                                  (nearest['points'], closest_color, .0011)):
        for point in points:
            if scene.ngeom >= scene.maxgeom:
                return
            geom = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_SPHERE, np.full(3,radius),
                            np.asarray(point), np.eye(3).ravel(), np.array(color,np.float32))
            scene.ngeom += 1


def closest_surfaces(surface, finger='index_finger'):
    """Return the geometric witnesses, also when a positive gap separates them."""
    best = None
    witness = np.empty(6)
    pairs = [i for i,h in enumerate(surface.second)
             if finger in surface.model.body(surface.model.geom_bodyid[h]).name]
    for g, h in zip(surface.first[pairs], surface.second[pairs]):
        gap = mj.mj_geomDistance(surface.model, surface.data, int(g), int(h), .3, witness)
        if best is None or gap < best[0]:
            best = (float(gap), witness.copy(), int(g), int(h))
    gap, points, g, h = best
    return dict(gap_m=gap, points=points.reshape(2, 3).tolist(),
                bodies=[surface.model.body(surface.model.geom_bodyid[i]).name for i in (g, h)])


class MovingPatch:
    def __init__(self, method):
        self.method = method
        self.surface = method.surface
        self.bodies = [method.thumb_bodies[-1], *method.index_bodies]
        m = self.surface.model
        self.geoms = [int(np.flatnonzero((m.geom_bodyid == b) & (m.geom_contype != 0))[0]) for b in self.bodies]
        self.ends = [m.site(method.thumb_tip).pos, m.body(self.bodies[2]).pos,
                     m.body(self.bodies[3]).pos, m.site(method.index_tip).pos]
        self.confirmed = False
        self.parameters = None
        self.strength = 0.
        self.minimum_distance = np.inf
        self.samples = []
        self.sample_segments = []

    def confirm(self, enabled):
        self.confirmed = enabled
        self.minimum_distance = np.inf
        self.parameters = None

    def prepare(self, kp, rotations):
        # Three neighboring index bands contribute separate on-mesh residuals.
        # Their weights blend at joints; their world points are never averaged.
        thumb = kp[4] - kp[3]
        parameters = []
        distances = []
        for j in range(3):
            finger = kp[6+j] - kp[5+j]
            p, q, t, u = segment_segment(kp[3]+.35*thumb, kp[3]+.88*thumb,
                                         kp[5+j]+.23*finger, kp[5+j]+.85*finger)
            delta = p-q
            distances.append(np.linalg.norm(delta))
            outward_t = rotations[3].T @ -delta
            outward_i = rotations[5+j].T @ delta
            angles = [np.clip(np.arctan2(outward_t[0], max(0., -outward_t[1])), -1.25, 1.25),
                      np.arctan2(max(0., -outward_i[1]), max(0., outward_i[0]))]
            parameters.append([.35+.53*t, .23+.62*u, *angles])
        distance = min(distances)
        if self.confirmed:
            self.minimum_distance = min(distance, self.minimum_distance)
            opening = max(distance-.022, distance-self.minimum_distance-.003, 0.)
            progress = np.clip(1-opening/.006, 0., 1.)
            if progress == 0.:
                self.confirm(False)
        else:
            progress = 0.
        goal = progress**2*(3-2*progress)
        beta = -np.expm1(-self.method.dt/.045)
        self.strength += beta*(goal-self.strength)
        if not self.confirmed:
            # Immediately remove attraction on release or explicit object mode.
            self.strength = 0.
        raw = np.array(parameters)
        self.parameters = raw if self.parameters is None else self.parameters + beta*(raw-self.parameters)
        weights = np.exp(-(np.array(distances)-distance)/.003)
        self.samples = []
        self.sample_segments = []
        for j, ((t,u,a,b), weight) in enumerate(zip(self.parameters, weights/weights.sum())):
            tup = [t, u]
            rays = [[np.sin(a), -np.cos(a), 0.], [np.cos(b), -np.sin(b), 0.]]
            sample = []
            for body, fraction, ray in zip((0,j+1), tup, rays):
                try:
                    point, normal = self.surface._ray(self.geoms[body], fraction*self.ends[body], ray)
                except ValueError:
                    # A ray can pass through a recess in this articulated mesh.
                    # This pair contributes no residual on this frame.
                    break
                sample.append((self.bodies[body], point, normal))
            if len(sample) == 2:
                self.samples.append((weight, sample))
                self.sample_segments.append(j)

    def world(self, sample):
        result = []
        for body, point, normal in sample:
            rot = self.surface.data.xmat[body].reshape(3,3)
            result.append((self.surface.data.xpos[body]+rot@point, rot@normal))
        return result

    def cost(self):
        if not self.strength:
            return 0.
        total = 0.
        for weight, sample in self.samples:
            (t, nt), (i, ni) = self.world(sample)
            delta = t-i
            gap = delta @ ni
            tangent = delta-gap*ni
            total += weight*(tangent@tangent/.004**2 + ((gap-.0002)/.002)**2
                             + np.sum((nt+ni)**2)/.55**2)
        return self.strength*total

    def diagnostic(self):
        k = int(np.argmax([w for w, _ in self.samples])) if self.samples else None
        j = self.sample_segments[k] if k is not None else None
        sample = self.samples[k][1] if k is not None else []
        support = sum(w for w, _ in self.samples)
        return dict(confirmed=self.confirmed, strength=float(self.strength*support),
                    valid_index_segments=self.sample_segments.copy(), index_segment=j,
                    surface_parameters=self.parameters[j].tolist() if j is not None else None,
                    candidate=[p.tolist() for p,n in self.world(sample)],
                    nearest=closest_surfaces(self.surface))


class AutomaticPatch(MovingPatch):
    """Continuous four-finger surface observations with an explicit gap target.

    Skin offsets are model estimates. Activation is deliberately reported as
    inferred proximity, never as measured human contact.
    """
    def __init__(self, method):
        super().__init__(method)
        m = self.surface.model
        self.fingers = ('index_finger', 'middle_finger', 'ring_finger', 'pinky')
        self.bodies = [method.thumb_bodies[-1]]
        self.ends = [m.site(method.thumb_tip).pos.copy()]
        for finger in self.fingers:
            bodies = [m.body('l_'+finger+'_'+part).id for part in ('proximal_abd','middle','distal')]
            self.bodies.extend(bodies)
            self.ends.extend([m.body(bodies[1]).pos.copy(), m.body(bodies[2]).pos.copy(),
                              m.site('l_'+finger+'_tip').pos.copy()])
        self.geoms = [int(np.flatnonzero((m.geom_bodyid==b)&(m.geom_contype!=0))[0])
                      for b in self.bodies]
        self.gaps = np.zeros(12)
        self.weights = np.zeros(12)
        self.disabled = False
        self.skin_offset_m = .010
        self.automatic = True
        self.anchors = []

    def confirm(self, enabled):
        # Optional F8 override; the default is continuous automatic observation.
        self.disabled = not enabled

    def contact_progress(self, distances):
        return np.clip(1-np.maximum(distances-self.skin_offset_m,0.)/.006,0,1)

    def prepare(self, kp, rotations):
        thumb = kp[4]-kp[3]
        raw, distances, compatibility = [], [], []
        for f in range(4):
            start = 5+4*f
            for j in range(3):
                finger = kp[start+j+1]-kp[start+j]
                # Observe the entire bone, including both ends at every joint.
                p,q,t,u = segment_segment(kp[3]+.2*thumb, kp[4], kp[start+j], kp[start+j+1])
                delta = p-q
                distance = np.linalg.norm(delta)
                direction = delta/max(distance,1e-10)
                local_t = rotations[3].T @ -direction
                local_i = rotations[start+j].T @ direction
                a = np.clip(np.arctan2(local_t[0],-local_t[1]),-1.25,1.25)
                b = np.arctan2(-local_i[1],local_i[0])
                # Avoid hinge recesses only when sampling the actual mesh.
                # The observation and gate still span the complete bone.
                raw.append([np.clip(.2+.8*t,.23,.9), np.clip(u,.23,.85), a,b])
                distances.append(distance)
                # Edge contact can be nearly orthogonal to the central pad
                # normal. Retain support instead of rejecting that approach.
                compatibility.append(.5+.5*np.clip((local_t[1]*-1+.25)/.5,0,1))
        distances = np.array(distances)
        self.gaps = np.maximum(distances-self.skin_offset_m,0.)*self.method.morphology.palm_scale
        proximity = self.contact_progress(distances)
        proximity = proximity**2*(3-2*proximity)*np.array(compatibility)
        if self.disabled:
            proximity[:] = 0.
        self.confirmed = False
        beta = -np.expm1(-self.method.dt/.045)
        goal = float(proximity.max())
        self.strength = min(goal,self.strength+beta*(goal-self.strength))
        raw = np.array(raw)
        if self.parameters is None:
            self.parameters = raw
        else:
            # Unwrap circumferential coordinates before temporal interpolation.
            delta = raw-self.parameters
            delta[:,2:] = (delta[:,2:]+np.pi)%(2*np.pi)-np.pi
            self.parameters += beta*delta
        # Smoothly blend adjacent bones and competing fingers without latching.
        weights = np.exp(-(distances-distances.min())/.0025)*proximity
        self.weights = weights/max(weights.sum(),1e-12)
        self.samples, self.sample_segments = [], []
        for k, ((t,u,a,b), weight) in enumerate(zip(self.parameters,self.weights)):
            if weight < 1e-5:
                continue
            sample = []
            for body,fraction,ray in ((0,t,[np.sin(a),-np.cos(a),0.]),
                                      (k+1,u,[np.cos(b),-np.sin(b),0.])):
                try:
                    point,normal = self.surface._ray(self.geoms[body],fraction*self.ends[body],ray)
                except ValueError:
                    break
                sample.append((self.bodies[body],point,normal))
            if len(sample)==2:
                self.samples.append((weight,sample))
                self.sample_segments.append(k)
        self.update_anchors()

    def update_anchors(self):
        self.anchors = []
        thumb_body = self.bodies[0]
        rotation = self.surface.data.xmat[thumb_body].reshape(3,3)
        origin = self.surface.data.xpos[thumb_body]
        for k in self.sample_segments:
            witness = np.empty(6)
            gap = mj.mj_geomDistance(self.surface.model,self.surface.data,
                                    self.geoms[0],self.geoms[k+1],.3,witness)
            normal = (witness[:3]-witness[3:])/gap if abs(gap)>1e-9 else self.world(
                self.samples[len(self.anchors)][1])[1][1]
            normal /= max(np.linalg.norm(normal),1e-10)
            self.anchors.append((rotation.T@(witness[:3]-origin),witness[3:].copy(),normal))

    def cost(self):
        total = 0.
        for k,(weight,sample),anchor in zip(self.sample_segments,self.samples,self.anchors):
            (t,nt),(i,ni) = self.world(sample)
            local, index_point, normal = anchor
            body = self.bodies[0]
            point = self.surface.data.xpos[body]+self.surface.data.xmat[body].reshape(3,3)@local
            gap = (point-index_point)@normal
            # A sequential local contact model keeps each frame's objective
            # differentiable. Recompute the mesh correspondence next frame.
            delta = point-i
            tangent = delta-(delta@ni)*ni
            desired=max(self.gaps[k],.0002)
            # The thumb candidate is a region preference, not a pinned center.
            region_error = max(0.,np.linalg.norm(point-t)-.006)
            total += weight*(tangent@tangent/.006**2+((gap-desired)/.001)**2
                             +(region_error/.008)**2)
        return self.strength*total

    def diagnostic(self):
        result = super().diagnostic()
        k = result['index_segment']
        finger = self.fingers[k//3] if k is not None else 'index_finger'
        result['nearest'] = closest_surfaces(self.surface,finger)
        result.update(automatic=True, contact_source='bone_proximity_model',
                      target_finger=self.fingers[k//3] if k is not None else None,
                      target_segment=k%3 if k is not None else None,
                      estimated_gap_m=float(self.gaps[k]) if k is not None else None,
                      human_skin_offset_m=self.skin_offset_m)
        if k is not None:
            sample = self.samples[int(np.argmax([w for w,_ in self.samples]))][1]
            target = self.world(sample)
            actual = np.array(result['nearest']['points'])
            result['candidate_witness_error_m'] = np.linalg.norm(
                actual-np.array([p for p,n in target]),axis=1).tolist()
        return result
