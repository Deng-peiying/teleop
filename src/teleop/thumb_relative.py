"""Continuous thumb/index geometry objective and optional mesh clearance."""
import time
import nlopt
import numpy as np
from scipy.spatial.transform import Rotation

from .hand2_geometry import unit, palm_basis
from .hand2_vendor_side import HandSurfaces
from .wuji_hand2_vendor import retarget_frame


def finger_frame(points, across):
    along = unit(points[-1] - points[-2])
    side = unit(across - along * (across @ along))
    return np.column_stack((along, side, np.cross(along, side)))


def first_thumb_frame(keypoints, orientation_xyzw):
    """Reconstruct the proximal bone frame from its axis and the MCP hinge.

    In the operator URDF, MCP flexion rotates around local X. The MCP's
    world X axis is therefore shared with its proximal parent. Landmark 1's
    quaternion does not directly describe this proximal bone frame.
    """
    z = -unit(np.asarray(keypoints)[2] - np.asarray(keypoints)[1])
    hinge = Rotation.from_quat(np.asarray(orientation_xyzw)[2]).as_matrix()[:, 0]
    x = unit(hinge - z * (hinge @ z))
    y = np.cross(z, x)
    return np.column_stack((x, y, z))


class ThumbRelative:
    """Track morphology-adapted bone geometry without surface attraction.

    Bone-plane orientation is a model proxy. Axial twist is unobserved in
    position-only recordings, so no measured pad-normal claim is made.
    """
    def __init__(self, session, morphology, dt, collision=True):
        self.session = session
        self.morphology = morphology
        self.dt = dt
        self.collision = collision
        self.surface = HandSurfaces()
        self.previous = None
        self.velocity = np.zeros(4)
        self.last_status = 0
        s = self.surface
        self.thumb_bodies = [s.model.body('l_thumb_' + name).id
                             for name in ('proximal_abd', 'middle', 'distal')]
        self.index_bodies = [s.model.body('l_index_finger_' + name).id
                             for name in ('proximal_abd', 'middle', 'distal')]
        self.thumb_tip = s.model.site('l_thumb_tip').id
        self.index_tip = s.model.site('l_index_finger_tip').id
        self.scale = float(morphology.lengths[1].sum())
        self.across = morphology.robot_basis[:, 0]
        self.use_rotations = False
        self.patch = None

    def features(self, thumb, index):
        frame = finger_frame(index, self.across)
        relative = (thumb[-1] - index[-1]) @ frame / self.scale
        directions = np.diff(thumb, axis=0)
        directions /= np.linalg.norm(directions, axis=1)[:, None]
        # Two nonparallel bone directions also describe the bending plane.
        return relative, directions @ frame

    def robot_features(self, command):
        s = self.surface
        s.pose(command)
        thumb = np.vstack((s.data.xpos[self.thumb_bodies], s.data.site_xpos[self.thumb_tip]))
        index = np.vstack((s.data.xpos[self.index_bodies], s.data.site_xpos[self.index_tip]))
        return self.features(thumb, index)

    def step(self, keypoints, orientations=None):
        return self.refine(retarget_frame(self.session, keypoints), keypoints, orientations)

    def refine(self, sdk, keypoints, orientations=None):
        started = time.perf_counter()
        sdk = np.asarray(sdk, float)
        keypoints = np.asarray(keypoints, float)
        if sdk.shape != (20,) or keypoints.shape != (21, 3) or not np.isfinite(sdk).all() or not np.isfinite(keypoints).all():
            raise ValueError('Expected finite SDK angles and skeleton positions')
        previous_sdk = getattr(self, "last_accepted_sdk", None)
        self.current_sdk = sdk.copy()
        # A single palm scale preserves cross-finger separation. Rebuilding
        # each chain independently changes aperture before IK even runs.
        aligned = self.morphology.align(keypoints) * self.morphology.palm_scale
        target_position, target_directions = self.features(aligned[1:5], aligned[5:9])
        pose_targets = None
        if self.use_rotations:
            if orientations is None:
                raise ValueError('Thumb pose method requires recorded skeleton quaternions')
            rotations = Rotation.from_quat(np.asarray(orientations)).as_matrix()
            if rotations.shape != (21, 3, 3):
                raise ValueError('Expected 21 skeleton rotations in XYZW order')
            alignment = self.morphology.robot_basis @ palm_basis(keypoints).T
            first = alignment @ first_thumb_frame(keypoints, orientations)
            index_frame = finger_frame(aligned[5:9], self.across)
            # Local -Y is the volar direction in the operator's human URDF.
            # This is model-estimated pad orientation, not a contact label.
            normal = alignment @ (-rotations[3, :, 1])
            pose_targets = (first.T, normal @ index_frame)
            if self.patch is not None:
                reference = sdk.copy()
                if getattr(self.patch,'automatic',False) and self.previous is not None:
                    reference[:4] = self.previous
                self.surface.pose(reference)
                self.patch.prepare(keypoints, rotations)
        previous = sdk[:4] if self.previous is None else self.previous
        lo, hi = self.surface.bounds.copy()
        if self.previous is not None:
            # Velocity is bounded. Acceleration is penalized in the objective,
            # since a joint stop can require an immediate velocity reduction.
            speed = 3.
            lo = np.maximum(lo, previous - speed * self.dt)
            hi = np.minimum(hi, previous + speed * self.dt)
        command = sdk.copy()
        geometry_weight = self.patch.strength if getattr(self.patch,'automatic',False) else 1.

        preserve_root = getattr(self.patch, 'preserve_sdk_root', False)
        # Regularize changes to the correction, so motion already present
        # in the SDK reference passes through at its original speed.
        last_offset = np.zeros(4) if previous_sdk is None else previous-previous_sdk[:4]
        correction_scale = np.array([.55, .55, .85, 1.25])*self.dt
        posture_scale = np.array([.12, .12, .22, .30])

        def pose_error():
            s = self.surface
            thumb = np.vstack((s.data.xpos[self.thumb_bodies], s.data.site_xpos[self.thumb_tip]))
            index = np.vstack((s.data.xpos[self.index_bodies], s.data.site_xpos[self.index_tip]))
            frame = finger_frame(index, self.across)
            z = -unit(thumb[1] - thumb[0])
            hinge = s.data.xmat[self.thumb_bodies[1]].reshape(3, 3)[:, 0]
            x = unit(hinge - z * (hinge @ z))
            first = np.column_stack((x, np.cross(z, x), z))
            # First-bone frame is expressed in the same aligned palm axes.
            first_error = first.T - pose_targets[0]
            _, normal = s.pad()
            normal_error = normal @ frame - pose_targets[1]
            return first_error, normal_error

        def value(x):
            command[:4] = x
            position, directions = self.robot_features(command)
            velocity = (x - previous) / self.dt
            orientation_cost = 0.
            if pose_targets is not None:
                first_error, normal_error = pose_error()
                orientation_cost = np.sum((first_error / .7)**2)
                if not getattr(self.patch,'automatic',False):
                    orientation_cost += np.sum((normal_error / .3)**2)
            patch_cost = self.patch.cost() if self.patch is not None else 0.
            root_cost = 0.
            if preserve_root:
                offset = x-sdk[:4]
                root_cost = np.sum((offset/posture_scale)**2 + ((offset-last_offset)/correction_scale)**2)
            return float(root_cost + patch_cost + geometry_weight*(orientation_cost
                         + np.sum(((position - target_position) / .04)**2)
                         + np.sum(((directions - target_directions) / .3)**2))
                         + np.sum((x - sdk[:4])**2)
                         + .005 * (velocity @ velocity)
                         + .01 * np.sum((velocity - self.velocity)**2))

        def objective(x, gradient):
            cost = value(x)
            evaluated.append((cost,x.copy()))
            if gradient.size:
                if getattr(self, 'analytic_gradient', False):
                    from .thumb_gradient import gradient as analytic
                    gradient[:] = analytic(self,x,sdk,previous,self.velocity,target_position,
                                           target_directions,pose_targets,pose_error,geometry_weight)
                    if preserve_root:
                        offset = x-sdk[:4]
                        gradient[:] += 2*offset/posture_scale**2 + 2*(offset-last_offset)/correction_scale**2
                    return cost
                for j in range(4):
                    step = np.zeros(4)
                    step[j] = 1e-5
                    gradient[j] = (value(x + step) - value(x - step)) / 2e-5
            return cost

        def clearance(result, x, gradient):
            command[:4] = x
            self.surface.pose(command)
            result[:] = 1000 * (.00015 - self.surface.distances(gradient if gradient.size else None))
            if gradient.size:
                gradient *= -1000

        optimizer = nlopt.opt(nlopt.LD_SLSQP, 4)
        evaluated = []
        optimizer.set_lower_bounds(lo)
        optimizer.set_upper_bounds(hi)
        optimizer.set_min_objective(objective)
        optimizer.set_maxeval(80)
        optimizer.set_ftol_abs(1e-6)
        if self.collision:
            optimizer.add_inequality_mconstraint(clearance, np.full(len(self.surface.first), .001))
        seed = np.clip(previous, lo, hi)
        free_pose_feasible = False
        if geometry_weight == 0.:
            reference = np.clip(sdk[:4],lo,hi)
            if preserve_root:
                remaining = np.sign(last_offset)*np.maximum(np.abs(last_offset)-correction_scale,0.)
                reference = np.clip(sdk[:4]+remaining,lo,hi)
            command[:4] = reference
            self.surface.pose(command)
            if np.all(self.surface.distances() >= .00015):
                seed = reference
                free_pose_feasible = True
        try:
            if free_pose_feasible:
                solution, status = seed.copy(), 1
            else:
                solution = optimizer.optimize(seed)
                status = optimizer.last_optimize_result()
        except nlopt.RoundoffLimited:
            # NLopt can terminate on roundoff after evaluating feasible poses.
            # Accept an evaluated current-frame candidate only after checking
            # exact bounds and all mesh clearances, and retain the exit status.
            solution = None
            for _, candidate in sorted(evaluated,key=lambda item:item[0]):
                if np.any(candidate<lo) or np.any(candidate>hi):
                    continue
                command[:4] = candidate
                self.surface.pose(command)
                if not self.collision or np.all(self.surface.distances()>=.000149):
                    solution = candidate.copy()
                    break
            if solution is None:
                raise RuntimeError('Roundoff termination without a feasible current-frame pose')
            status = -4
        command[:4] = solution
        if self.collision:
            # Sequential minimum-norm projection of residual clearance errors.
            # Joint/rate bounds remain active during every correction.
            for _ in range(12):
                self.surface.pose(command)
                jacobian = np.empty((len(self.surface.first),4))
                deficit = .00015-self.surface.distances(jacobian)
                pair = int(np.argmax(deficit))
                if deficit[pair] <= 1e-6:
                    break
                direction = jacobian[pair].copy()
                direction[((command[:4]<=lo+1e-10)&(direction<0)) |
                          ((command[:4]>=hi-1e-10)&(direction>0))] = 0.
                norm = direction@direction
                if norm < 1e-14:
                    break
                command[:4] = np.clip(command[:4]+(deficit[pair]+.00002)*direction/norm,lo,hi)
            solution = command[:4].copy()
        # On a nail-to-base release, unwinding the stored CMC correction can
        # reverse flexion even while the SDK flexion continues monotonically.
        # Keep part of the observed flexion step while paying back the offset.
        # Accept the first-axis substitution only after checking mesh clearance.
        if geometry_weight == 0 and getattr(self.patch, 'limit_return_rebound', False) and previous_sdk is not None:
            sdk_step = sdk[0] - previous_sdk[0]
            output_step = solution[0] - previous[0]
            if output_step*sdk_step < 0 and last_offset[0]*sdk_step > 0:
                command[0] = previous[0] + .25*sdk_step
                self.surface.pose(command)
                if lo[0] <= command[0] <= hi[0] and np.all(self.surface.distances() >= .000149):
                    solution[0] = command[0]
                else:
                    command[0] = solution[0]
            if abs(solution[0]-sdk[0]) < 1e-8:
                self.patch.index_return_release = False
        position, directions = self.robot_features(command)
        min_gap = float(self.surface.distances().min())
        if self.collision and min_gap < -.000001:
            raise RuntimeError(f'Relative thumb collision constraint failed: {min_gap:.6f} m')
        self.velocity = (solution - previous) / self.dt
        self.previous = solution.copy()
        self.last_accepted_sdk = sdk.copy()
        self.last_status = status
        self.side_diagnostics = dict(
            status=int(status), sdk_command_rad=sdk.tolist(),
            optimizer_objective_calls=len(evaluated),
            free_pose_fast_path=free_pose_feasible,
            solve_ms=1000*(time.perf_counter()-started),
            sdk_thumb_delta_rad=(solution-sdk[:4]).tolist(),
            joint_limit_margin_rad=np.minimum(solution-self.surface.bounds[0],
                                              self.surface.bounds[1]-solution).tolist(),
            frame_bound_margin_rad=np.minimum(solution-lo,hi-solution).tolist(),
            thumb_velocity_rad_s=self.velocity.tolist(),
            clearance_deficit_m=max(0., .00015-min_gap),
            relative_error_mm=float(np.linalg.norm(position - target_position) * self.scale * 1000),
            direction_error_deg=float(np.degrees(np.arccos(np.clip(
                np.sum(directions * target_directions, axis=1), -1, 1))).mean()),
            minimum_clearance_m=min_gap)
        if pose_targets is not None:
            first_error, normal_error = pose_error()
            self.side_diagnostics['first_frame_chord_error'] = float(np.linalg.norm(first_error))
            self.side_diagnostics['pad_normal_error_deg'] = float(np.degrees(
                np.arccos(np.clip(1 - .5 * (normal_error @ normal_error), -1, 1))))
        if self.patch is not None:
            if hasattr(self.patch, 'finalize'):
                self.patch.finalize()
            self.side_diagnostics['patch'] = self.patch.diagnostic()
        return command.copy()
