"""Reference qpos poses for mjcf/humanoid.xml (nq=28: 3 root pos + 4 root
quat + 21 hinge angles), kept here in Python rather than as <keyframe>
entries in the MJCF so the model file stays a pure structural definition and
any reference motion (hand-picked poses, or later a converted SMPL
trajectory) is just a plain qpos array.

Values are the original DeepMind humanoid.xml keyframes.
"""
import numpy as np

POSES = {
    "squat": np.array([
        0, 0, 0.596,
        0.988015, 0, 0.154359, 0,
        0, 0.4, 0,
        -0.25, -0.5, -2.5, -2.65, -0.8, 0.56,
        -0.25, -0.5, -2.5, -2.65, -0.8, 0.56,
        0, 0, 0, 0, 0, 0,
    ]),
    "stand_on_left_leg": np.array([
        0, 0, 1.21948,
        0.971588, -0.179973, 0.135318, -0.0729076,
        -0.0516, -0.202, 0.23,
        -0.24, -0.007, -0.34, -1.76, -0.466, -0.0415,
        -0.08, -0.01, -0.37, -0.685, -0.35, -0.09,
        0.109, -0.067, -0.7, -0.05, 0.12, 0.16,
    ]),
    "prone": np.array([
        0.4, 0, 0.0757706,
        0.7325, 0, 0.680767, 0,
        0, 0.0729, 0,
        0.0077, 0.0019, -0.026, -0.351, -0.27, 0,
        0.0077, 0.0019, -0.026, -0.351, -0.27, 0,
        0.56, -0.62, -1.752,
        0.56, -0.62, -1.752,
    ]),
    "supine": np.array([
        -0.4, 0, 0.08122,
        0.722788, 0, -0.69107, 0,
        0, -0.25, 0,
        0.0182, 0.0142, 0.3, 0.042, -0.44, -0.02,
        0.0182, 0.0142, 0.3, 0.042, -0.44, -0.02,
        0.186, -0.73, -1.73,
        0.186, -0.73, -1.73,
    ]),
}

for _name, _qpos in POSES.items():
    assert _qpos.shape == (28,), f"pose {_name!r} has {_qpos.shape[0]} entries, expected 28"
