"""Launch SONIC MuJoCo with a grounded nominal G1 reset pose."""

from __future__ import annotations

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if __package__:
    from .paths import projects_root
else:
    sys.path.insert(0, str(PROJECT_ROOT))
    from retargeting.paths import projects_root


SONIC_ROOT = projects_root() / "GR00T-WholeBodyControl"
sys.path.insert(0, str(SONIC_ROOT))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import tyro  # noqa: E402

from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model  # noqa: E402
from gear_sonic.scripts.run_sim_loop import ArgsConfig, SimWrapper  # noqa: E402
from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory  # noqa: E402


CONTACT_CLEARANCE = 0.001
NOMINAL_JOINTS = {
    "left_hip_pitch_joint": -0.312,
    "left_knee_joint": 0.669,
    "left_ankle_pitch_joint": -0.363,
    "right_hip_pitch_joint": -0.312,
    "right_knee_joint": 0.669,
    "right_ankle_pitch_joint": -0.363,
    "left_shoulder_pitch_joint": 0.2,
    "left_shoulder_roll_joint": 0.2,
    "left_elbow_joint": 0.6,
    "right_shoulder_pitch_joint": 0.2,
    "right_shoulder_roll_joint": -0.2,
    "right_elbow_joint": 0.6,
}


def configure_grounded_reset(simulator) -> None:
    """Make initial startup and automatic fall resets use SONIC's standing pose."""
    environment = simulator.sim_env
    model = environment.mj_model
    qpos = model.qpos0.copy()
    qpos[:3] = (0.0, 0.0, 0.793)
    qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    for name, position in NOMINAL_JOINTS.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo model is missing required joint: {name}")
        qpos[model.jnt_qposadr[joint_id]] = position
    sole_bottoms = []
    environment.mj_data.qpos[:] = qpos
    mujoco.mj_forward(model, environment.mj_data)
    for body_name in ("left_ankle_roll_link", "right_ankle_roll_link"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        for geom_id in range(model.ngeom):
            if model.geom_bodyid[geom_id] != body_id or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
                continue
            rotation = environment.mj_data.geom_xmat[geom_id].reshape(3, 3)
            vertical_extent = float(np.abs(rotation[2]) @ model.geom_size[geom_id])
            sole_bottoms.append(float(environment.mj_data.geom_xpos[geom_id, 2] - vertical_extent))
    if len(sole_bottoms) != 2:
        raise RuntimeError(f"expected two sole collision boxes, found {len(sole_bottoms)}")
    qpos[2] += CONTACT_CLEARANCE - min(sole_bottoms)

    def grounded_reset() -> None:
        mujoco.mj_resetData(model, environment.mj_data)
        environment.mj_data.qpos[:] = qpos
        mujoco.mj_forward(model, environment.mj_data)

    environment.reset = grounded_reset
    grounded_reset()
    pelvis_height = float(qpos[2])
    if environment.elastic_band is not None:
        environment.elastic_band.point = np.array([0.0, 0.0, pelvis_height])
    print(f"Grounded reset pose: pelvis={pelvis_height:.3f} m; elastic anchor aligned", flush=True)


def main(config: ArgsConfig) -> None:
    wbc_config = config.load_wbc_yaml()
    wbc_config["ENV_NAME"] = config.env_name
    robot_model = instantiate_g1_robot_model()
    wrapper = SimWrapper(
        robot_model=robot_model,
        env_name=config.env_name,
        config=wbc_config,
        onscreen=wbc_config.get("ENABLE_ONSCREEN", True),
        offscreen=wbc_config.get("ENABLE_OFFSCREEN", False),
        enable_image_publish=config.enable_image_publish,
    )
    configure_grounded_reset(wrapper.sim)
    SimulatorFactory.start_simulator(
        wrapper.sim,
        as_thread=False,
        enable_image_publish=config.enable_image_publish,
        mp_start_method=config.mp_start_method,
        camera_port=config.camera_port,
    )


if __name__ == "__main__":
    main(tyro.cli(ArgsConfig))
