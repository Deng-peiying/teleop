"""Analytic objective gradient for the experimental surface variants only."""
import mujoco as mj
import numpy as np

from .thumb_relative import finger_frame


def gradient(method, x, sdk, previous, prior_velocity, position_target, direction_target,
             pose_targets, pose_error, geometry_weight):
    s=method.surface
    thumb=np.vstack((s.data.xpos[method.thumb_bodies],s.data.site_xpos[method.thumb_tip]))
    index=np.vstack((s.data.xpos[method.index_bodies],s.data.site_xpos[method.index_tip]))
    frame=finger_frame(index,method.across)
    jacs=[]
    for point,body in zip(thumb,[*method.thumb_bodies,method.thumb_bodies[-1]]):
        jp=np.empty((3,s.model.nv))
        mj.mj_jac(s.model,s.data,jp,None,point,body)
        jacs.append(jp[:,s.thumb_dof])
    position=(thumb[-1]-index[-1])@frame/method.scale
    result=2*(position-position_target)@(frame.T@jacs[-1]/method.scale)/.04**2
    for j in range(3):
        delta=thumb[j+1]-thumb[j];length=np.linalg.norm(delta);axis=delta/length
        derivative=(np.eye(3)-np.outer(axis,axis))@(jacs[j+1]-jacs[j])/length
        result+=2*(axis@frame-direction_target[j])@(frame.T@derivative)/.3**2
    if pose_targets is not None:
        first_error,_=pose_error()
        first=first_error.T+pose_targets[0].T
        angular=np.empty((3,s.model.nv))
        mj.mj_jac(s.model,s.data,None,angular,s.data.xpos[method.thumb_bodies[0]],method.thumb_bodies[0])
        # first.T rows are the frame's world-space basis vectors.
        derivative=np.cross(angular[:,s.thumb_dof].T[:,None,:],first.T[None,:,:])
        result+=2*np.einsum('ab,jab->j',first_error,derivative)/.7**2
    velocity=(x-previous)/method.dt
    return (geometry_weight*result+method.patch.gradient()+2*(x-sdk[:4])
            +.01*velocity/method.dt+.02*(velocity-prior_velocity)/method.dt)
