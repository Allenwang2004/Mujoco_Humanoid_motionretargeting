import sys
from pathlib import Path

import mujoco
import PIL.Image

path = Path(sys.argv[1])
out_dir = Path("renders")
out_dir.mkdir(exist_ok=True)

for xml_path in sorted(path.glob("*.xml")):
    out = out_dir / f"{xml_path.stem}.png"

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print(xml_path, "nq =", model.nq, "total mass =", sum(model.body_mass))

    renderer = mujoco.Renderer(model, height=480, width=640)
    if xml_path.stem == "robot":
        # robot.xml has no <statistic> extent, so the default free camera
        # auto-scales to its 100x100 floor plane and renders the robot as a
        # speck. Frame it manually instead of touching the model file.
        camera = mujoco.MjvCamera()
        camera.lookat = [0, -0.2, 0.9]
        camera.distance = 3.0
        camera.azimuth = 90
        camera.elevation = -10
        renderer.update_scene(data, camera=camera)
    else:
        renderer.update_scene(data)
    img = renderer.render()
    PIL.Image.fromarray(img).save(out)
