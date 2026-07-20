import sys
import numpy as np
import mujoco

path = sys.argv[1]
model = mujoco.MjModel.from_xml_path(path)
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)

for step in range(500):
    mujoco.mj_step(model, data)
    if not np.all(np.isfinite(data.qpos)):
        print(f"{path}: went unstable (NaN) at step {step}")
        sys.exit(1)

print(f"{path}: stable after 500 steps. root height = {data.qpos[2]:.3f}")
