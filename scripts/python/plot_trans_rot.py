import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_wheel_ctrl_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode_log.py <logfile>")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")

data = np.empty((0, 7), dtype=float)
init = 0
for event in log:
    if event.channel == "MBOT_WHEEL_CTRL":
        msg = mbot_wheel_ctrl_t.decode(event.data)
        if init == 0:
            start_utime = msg.utime
            init = 1
        data = np.append(
            data,
            np.array(
                [
                    [
                        (msg.utime - start_utime) / 1.0e6,
                        msg.left_motor_pwm,
                        msg.right_motor_pwm,
                        msg.left_motor_vel_cmd,
                        msg.right_motor_vel_cmd,
                        msg.left_motor_vel,
                        msg.right_motor_vel,
                    ]
                ]
            ),
            axis=0,
        )
# measured_vel_fwd = (measured_vel_l + measured_vel_r) / 2.0;
# measured_vel_turn = (measured_vel_r - measured_vel_l) / WHEEL_BASE;
plt.plot(data[:, 0], (data[:, 5]+data[:,6])/2)
plt.plot(data[:, 0], (data[:, 6]-data[:, 5])/0.155)
plt.legend(["TRANS_VEL", "ROT_VEL"])

plt.show()
