import mujoco

model = mujoco.MjModel.from_xml_path("mjcf/humanoid_CMU.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
print("Loaded OK. nq =", model.nq, "nbody =", model.nbody)

renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data)
img = renderer.render()

import PIL.Image
PIL.Image.fromarray(img).save("renders/snapshot.png")
print("Saved renders/snapshot.png")
