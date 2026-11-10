import mujoco as mj
import nlopt
import numpy as np
from .hand2_geometry import nearest_arc, unit
from .wuji_hand2_vendor import ROOT, firmware_joint_names, retarget_frame

class HandSurfaces:
    """Mesh-attached landmarks and signed distances on the SDK preview MJCF."""

    def __init__(self):
        self.model = m = mj.MjModel.from_xml_path(str(ROOT / 'models/Wuji/hand2_beta1/body/mjcf/left.xml'))
        self.data = d = mj.MjData(m)
        joints = [m.joint(n).id for n in firmware_joint_names('left')]
        self.address = m.jnt_qposadr[joints]
        self.thumb_dof = m.jnt_dofadr[joints[:4]]
        self.bounds = m.jnt_range[joints[:4]].T.copy()
        mj.mj_forward(m, d)
        geoms = {m.body(b).name: g for g, b in enumerate(m.geom_bodyid) if m.geom_contype[g]}
        self.pad_body = m.body('l_thumb_distal').id
        pad = [self._ray(geoms['l_thumb_distal'], [x, 0, -0.019], [0, -1, 0]) for x in (-0.002, 0, 0.002)]
        self.pad_point = np.mean([p for p, n in pad], axis=0)
        self.pad_normal = unit(np.sum([n for p, n in pad], axis=0))
        rail = []
        for part in ('proximal', 'proximal_abd', 'middle', 'distal'):
            name = 'l_index_finger_' + part
            body = m.body(name).id
            end = m.site('l_index_finger_tip').pos if part == 'distal' else m.body_pos[np.flatnonzero(m.body_parentid == body)[0]]
            fractions = {'proximal': (0.16, 0.32, 0.48), 'distal': (0.4, 0.64, 0.87)}
            for fraction in fractions.get(part, (0.16, 0.5, 0.84)):
                p, n = self._ray(geoms[name], fraction * end, [1, 0, 0])
                rail.append((body, p, n))
        self.rail_bodies = np.array([b for b, p, n in rail])
        self.rail_points = np.stack([p for b, p, n in rail])
        self.rail_normals = np.stack([n for b, p, n in rail])
        pairs = [(g, h) for thumb, g in geoms.items() if 'thumb' in thumb for name, h in geoms.items() if 'thumb' not in name and (not (name == 'l_wrist' and thumb in ('l_thumb_proximal', 'l_thumb_proximal_abd')))]
        self.first, self.second = np.array(pairs).T
        self.index_pairs = np.array(['index' in m.body(m.geom_bodyid[h]).name for h in self.second])
        self.rbound = m.geom_rbound[self.first] + m.geom_rbound[self.second]

    def _ray(self, geom, center, outward):
        b = self.model.geom_bodyid[geom]
        rot = self.data.xmat[b].reshape(3, 3)
        source = np.asarray(center) + 0.04 * np.array(outward)
        normal = np.zeros(3)
        dist = mj.mj_rayMesh(self.model, self.data, geom, self.data.xpos[b] + rot @ source, -rot @ outward, normal)
        if dist < 0 or np.dot(rot.T @ normal, outward) <= 0:
            raise ValueError(f'Cannot locate outward-facing landmark on {self.model.body(b).name}')
        return (source - dist * np.array(outward), rot.T @ normal)

    def pose(self, q):
        self.data.qpos[self.address] = q
        mj.mj_kinematics(self.model, self.data)
        mj.mj_comPos(self.model, self.data)

    def pad(self):
        rot = self.data.xmat[self.pad_body].reshape(3, 3)
        return (self.data.xpos[self.pad_body] + rot @ self.pad_point, rot @ self.pad_normal)

    def rail(self):
        rot = self.data.xmat[self.rail_bodies].reshape(-1, 3, 3)
        points = self.data.xpos[self.rail_bodies] + np.einsum('bij,bj->bi', rot, self.rail_points)
        normals = np.einsum('bij,bj->bi', rot, self.rail_normals)
        lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        return (points, normals, lengths / lengths[-1])

    def target(self, arc):
        points, normals, knots = self.rail()
        i = np.clip(knots.searchsorted(arc) - 1, 0, len(knots) - 2)
        t = np.clip((arc - knots[i]) / (knots[i + 1] - knots[i]), 0.0, 1.0)
        return ((1 - t) * points[i] + t * points[i + 1], unit((1 - t) * normals[i] + t * normals[i + 1]))

    def distances(self, jac=None):
        cap = 0.004
        result = np.full(len(self.first), cap)
        centers = self.data.geom_xpos
        near = np.linalg.norm(centers[self.first] - centers[self.second], axis=1) - self.rbound < cap
        witness = np.empty(6)
        if jac is not None:
            jac[:] = 0.0
        for i in np.flatnonzero(near):
            g, h = (self.first[i], self.second[i])
            dist = mj.mj_geomDistance(self.model, self.data, g, h, cap, witness)
            result[i] = dist
            if jac is not None and 1e-12 < abs(dist) and (dist < cap):
                jp = np.empty((3, self.model.nv))
                mj.mj_jac(self.model, self.data, jp, None, witness[:3], self.model.geom_bodyid[g])
                jac[i] = (witness[:3] - witness[3:]) / dist @ jp[:, self.thumb_dof]
        return result

    def metrics(self, q, arc, human_arc=None):
        self.pose(q)
        pad, normal = self.pad()
        target, radial = self.target(arc)
        points, _, knots = self.rail()
        segments = np.diff(points, axis=0)
        t = np.clip(np.sum((pad - points[:-1]) * segments, axis=1) / np.sum(segments ** 2, axis=1), 0, 1)
        i = np.argmin(np.linalg.norm(pad - points[:-1] - t[:, None] * segments, axis=1))
        gap = self.distances()
        i_pair = np.flatnonzero(self.index_pairs)[np.argmin(gap[self.index_pairs])]
        g, h = (self.first[i_pair], self.second[i_pair])
        witness = np.empty(6)
        distance = mj.mj_geomDistance(self.model, self.data, g, h, 0.2, witness)
        patch_contact = -0.0001 <= distance < 0.002 and self.model.geom_bodyid[g] == self.pad_body and (np.linalg.norm(witness[:3] - pad) <= 0.008)
        return dict(side_error_mm=float(1000 * np.linalg.norm(pad - target)), pad_angle_deg=float(np.degrees(np.arccos(np.clip(-normal @ radial, -1, 1)))), arc_error=float(abs(knots[i] + t[i] * (knots[i + 1] - knots[i]) - (arc if human_arc is None else human_arc))), near_pad_witness=bool(patch_contact), index_penetration_mm=float(max(0.0, -1000 * min(gap[self.index_pairs]))), thumb_penetration_mm=float(max(0.0, -1000 * min(gap))))
