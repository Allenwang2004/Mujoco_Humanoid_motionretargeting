import sys
import mujoco
import PIL.Image

path = sys.argv[1]
out = sys.argv[2]
model = mujoco.MjModel.from_xml_path(path)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
print(path, "nq =", model.nq, "total mass =", sum(model.body_mass))

renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data)
img = renderer.render()
PIL.Image.fromarray(img).save(out)
