"""Portable arm/L20 slider models using the project's original URDF assets."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

from .mapping import CONTROLS, L20_URDF, DISPLAY_NAMES

ROOT = Path(__file__).resolve().parents[2]
ARM_NAMES = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint7")


def build_control_model(mode, preview=False):
    assets = {}
    hand = ET.parse(L20_URDF).getroot()
    for mesh in hand.iter("mesh"):
        path = L20_URDF.parent / mesh.get("filename")
        name = "l20_" + path.name
        assets[name] = path.read_bytes()
        mesh.set("filename", name)
    prefix = "" if mode == "hand" else "l20_"
    if mode == "hand":
        root = hand
    else:
        folder = ROOT / "models/RM75"
        root = ET.parse(folder / "rm_75_6fb.urdf").getroot()
        for mesh in root.iter("mesh"):
            path = folder / "meshes" / Path(mesh.get("filename")).name
            name = "rm75_" + path.name
            assets[name] = path.read_bytes()
            mesh.set("filename", name)
        for node in hand.iter():
            for key in ("name", "link", "joint"):
                if node.get(key):
                    node.set(key, prefix + node.get(key))
        root.extend(hand)
        mount = json.loads((folder / "mount.json").read_text(encoding="utf-8"))
        joint = ET.SubElement(root, "joint", name="l20_mount", type="fixed")
        ET.SubElement(joint, "parent", link=mount["parent"])
        ET.SubElement(joint, "child", link=mount["child"])
        ET.SubElement(joint, "origin", xyz=" ".join(map(str, mount["xyz_m"])),
                      rpy=" ".join(str(math.radians(a)) for a in mount["rpy_deg"]))
    root.set("name", "teleop_" + mode.replace("-", "_") + ("_PREVIEW" if preview else "_LIVE"))
    compiler = ET.SubElement(ET.SubElement(root, "mujoco"), "compiler")
    compiler.set("strippath", "false")
    # URDF importers can identify meshes by basename, even with absolute paths.
    # Distinct VFS asset names preserve both base_link.STL files.
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"), assets=assets)
    joints = {j.get("name"): j for j in root.findall("joint")}
    if mode != "hand":
        for i, name in enumerate(ARM_NAMES):
            limit = joints[name].find("limit")
            actuator = spec.add_actuator(
                name=f"ARM_J{i + 1}_deg", target=name,
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                gear=[180 / math.pi, 0, 0, 0, 0, 0], ctrllimited=True,
                ctrlrange=[math.degrees(float(limit.get(k))) for k in ("lower", "upper")],
                group=0,
            )
            actuator.set_to_position(1)
    if mode != "arm":
        for name, channel in CONTROLS:
            limit = joints[prefix + name].find("limit")
            actuator = spec.add_actuator(
                name="L20_" + DISPLAY_NAMES[channel] + "_rad", target=prefix + name,
                trntype=mujoco.mjtTrn.mjTRN_JOINT, ctrllimited=True,
                ctrlrange=[float(limit.get(k)) for k in ("lower", "upper")], group=1,
            )
            actuator.set_to_position(20, kv=1)
    model = spec.compile()
    for i in range(model.neq):
        slave = model.joint(model.eq_obj1id[i]).name
        mimic = joints[slave].find("mimic")
        model.eq_data[i, :2] = [
            float(mimic.get("offset", 0)), float(mimic.get("multiplier", 1))]
    return model


def display_targets(model, data):
    """Position preview only; device SDKs execute the corresponding targets."""
    for i in range(model.nu):
        joint = model.actuator_trnid[i, 0]
        data.qpos[model.jnt_qposadr[joint]] = data.ctrl[i] / model.actuator_gear[i, 0]
    for i in range(model.neq):
        child = model.jnt_qposadr[model.eq_obj1id[i]]
        parent = model.jnt_qposadr[model.eq_obj2id[i]]
        data.qpos[child] = model.eq_data[i, 0] + model.eq_data[i, 1] * data.qpos[parent]
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
