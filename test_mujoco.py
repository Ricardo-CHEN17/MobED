import mujoco
import time

XML_PATH = 'local_mtgvakn0_9w389c_mjcf_stl/robot.xml'
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
mujoco.mj_resetDataKeyframe(model, data, key_id)

for i in range(8):
    data.ctrl[i] = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, [
        'Steering_joint_LF', 'Steering_joint_RF', 'Steering_joint_LB', 'Steering_joint_RB',
        'Posture_control_joint_LF', 'Posture_control_joint_RF', 'Posture_control_joint_LB', 'Posture_control_joint_RB'
    ][i])]]

# run for a bit
for _ in range(500):
    mujoco.mj_step(model, data)

# check velocity of joints for 100 steps
vels = []
for _ in range(100):
    mujoco.mj_step(model, data)
    vels.append(abs(data.qvel[0]) + abs(data.qvel[1]) + abs(data.qvel[2])) # chassis velocities

print(f"Average chassis velocity: {sum(vels)/len(vels):.6f}")
print(f"Max chassis velocity: {max(vels):.6f}")
