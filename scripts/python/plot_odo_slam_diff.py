import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import odometry_t, pose_xyt_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode_log.py <logfile>")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1],"r")

def wrap2Pi(input):
    phases =  (( -input + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0

    return phases

data1 = np.empty((0,4), dtype=float)
data2 = np.empty((0,4), dtype=float)
init1 = 0
init2 = 0
for event in log:
    if event.channel == "ODOMETRY":
        msg1 = odometry_t.decode(event.data)
        if init1==0:
            start_utime1 = msg1.utime
            init1 = 1
        data1 = np.append(data1, np.array([[ \
            (msg1.utime-start_utime1)/1.0E6, \
            msg1.x, \
            msg1.y, \
            msg1.theta
            ]]), axis=0)

    if event.channel == "SLAM_POSE":
        msg2 = pose_xyt_t.decode(event.data)
        if init2==0:
            start_utime2 = msg2.utime
            init2 = 1
        data2 = np.append(data2, np.array([[ \
            (msg2.utime-start_utime2)/1.0E6, \
            msg2.x, \
            msg2.y, \
            msg2.theta
            ]]), axis=0)

data3 = np.empty((0,4), dtype=float)
# print(data2[2,:])
for i in range(data2.shape[0]):
    for j in range(data1.shape[0]):
        if(abs(data2[i,0] - data1[j,0])<0.05):
            data3 = np.append(data3, data1[[j],:], axis=0)
            break

print(data2.shape, data3.shape)
plt.plot(data2[:,0], data2[:,1] - data3[:,1])
plt.plot(data2[:,0], data2[:,2] - data3[:,2])
plt.plot(data2[:,0], wrap2Pi(data2[:,3] - data3[:,3]))
plt.legend(["x position diff", "y position diff", "theta diff"])
plt.xlabel("time in seconds")
plt.ylabel("position difference")
plt.show()