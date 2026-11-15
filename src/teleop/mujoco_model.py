"""Standalone RM75 joint preview using the retained official URDF."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
ROOT = Path(__file__).resolve().parents[2]
ARM_NAMES = ('joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint7')

def build_control_model(mode='arm', preview=False):
    if mode != 'arm':
        raise ValueError('Only standalone arm debugging is supported')
    folder = ROOT/'models/RM75'
    root = ET.parse(folder/'rm_75_6fb.urdf').getroot()
    assets = {}
    for mesh in root.iter('mesh'):
        path = folder/'meshes'/Path(mesh.get('filename')).name
        name = 'rm75_'+path.name
        assets[name] = path.read_bytes()
        mesh.set('filename',name)
    root.set('name','RM75_PREVIEW' if preview else 'RM75_LIVE')
    ET.SubElement(ET.SubElement(root,'mujoco'),'compiler',strippath='false')
    spec = mujoco.MjSpec.from_string(ET.tostring(root,encoding='unicode'),assets=assets)
    joints = {j.get('name'):j for j in root.findall('joint')}
    for i,name in enumerate(ARM_NAMES):
        limit = joints[name].find('limit')
        actuator = spec.add_actuator(name=f'ARM_J{i+1}_deg',target=name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,gear=[180/math.pi,0,0,0,0,0],
            ctrllimited=True,ctrlrange=[math.degrees(float(limit.get(k))) for k in ('lower','upper')])
        actuator.set_to_position(1)
    return spec.compile()

def display_targets(model,data):
    for i in range(model.nu):
        joint = model.actuator_trnid[i,0]
        data.qpos[model.jnt_qposadr[joint]] = data.ctrl[i]/model.actuator_gear[i,0]
    data.qvel[:] = 0
    mujoco.mj_forward(model,data)
