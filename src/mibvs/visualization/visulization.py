import numpy as np
import matplotlib.pyplot as plt
import rosbag

# ------------------------------------------------------------------------------------------------------


def msg_to_np(topic_name, bag):

    norm = []
    timestamps = []
    for topic, msg, tt in bag.read_messages(topics=[topic_name]):
        timestamps.append(tt.to_sec())
        norm.append([msg.data])
    
    t = np.array(timestamps)
    # t = t - t[0]
    norm = np.array(norm)
    
    return t, norm



def msg_to_np2(topic_name, bag):
    w = []
    timestamps = []
    for topic, msg, tt in bag.read_messages(topics=[topic_name]):
        w.append([msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        timestamps.append(tt.to_sec())

    tt = np.array(timestamps)
    tt = tt - tt[0]
    w = np.array(w)
    return tt, w


# ------------------------------------------------------------------------------------------------------


bag4 = rosbag.Bag('/home/shayan/shayan_ros_/src/shayan_ibvs/rosbags/bag_2024-08-07-15-40-58.bag') #without Lidar
bag5 = rosbag.Bag('/home/shayan/shayan_ros_/src/shayan_ibvs/rosbags/bag_2024-08-07-15-30-41.bag') #with 300g lidar

t ,  norm = msg_to_np('/norm', bag4)
t1, norm1 = msg_to_np('/norm', bag5)


fig = plt.figure(figsize=(10,6))
ax = fig.add_subplot(111)

ax.plot(t, norm, color='blue')
ax.plot(t1, norm1, color='red', linestyle = 'dashed')
ax.set_xlabel('Time (seconds)', fontsize=14, weight='bold')
ax.set_ylabel(r'$\vert \vert q\vert \vert_1$', fontsize=14, weight='bold')

LL = 160; UL = 165
region_of_interest = (t > LL) & (t < UL)
region_of_interest1 = (t1 > LL) & (t1 < UL)
inset_ax = ax.inset_axes([0.3, 0.6, 0.2, 0.2])  # [left, bottom, width, height] in figure coordinates
inset_ax.plot(t[region_of_interest], norm[region_of_interest], color='blue')
inset_ax.plot(t1[region_of_interest1], norm1[region_of_interest1], color='red', linestyle = 'dashed')
ax.indicate_inset_zoom(inset_ax, edgecolor="black")


LL = 320; UL = 330
region_of_interest = (t > LL) & (t < UL)
region_of_interest1 = (t1 > LL) & (t1 < UL)
inset_ax = ax.inset_axes([0.56, 0.6, 0.2, 0.2])  # [left, bottom, width, height] in figure coordinates
inset_ax.plot(t[region_of_interest], norm[region_of_interest], color='blue')
inset_ax.plot(t1[region_of_interest1], norm1[region_of_interest1], color='red', linestyle = 'dashed')
ax.indicate_inset_zoom(inset_ax, edgecolor="black")

LL = 400; UL = 410
region_of_interest = (t > LL) & (t < UL)
region_of_interest1 = (t1 > LL) & (t1 < UL)
inset_ax = ax.inset_axes([0.84, 0.6, 0.15, 0.2])  # [left, bottom, width, height] in figure coordinates
inset_ax.plot(t[region_of_interest], norm[region_of_interest], color='blue')
inset_ax.plot(t1[region_of_interest1], norm1[region_of_interest1], color='red', linestyle = 'dashed')
ax.indicate_inset_zoom(inset_ax, edgecolor="black")

LL = 15; UL = 25
region_of_interest = (t > LL) & (t < UL)
region_of_interest1 = (t1 > LL) & (t1 < UL)
inset_ax = ax.inset_axes([0.1, 0.6, 0.15, 0.2])  # [left, bottom, width, height] in figure coordinates
inset_ax.plot(t[region_of_interest], norm[region_of_interest], color='blue')
inset_ax.plot(t1[region_of_interest1], norm1[region_of_interest1], color='red', linestyle = 'dashed')


# # Optionally, add a rectangle to show the region of interest on the main plot
ax.indicate_inset_zoom(inset_ax, edgecolor="black")
# ax.grid(visible=True)

plt.xlim([10,420])
plt.ylim([0,1])
plt.legend(['Without LiDAR','With LiDAR'], fontsize=14, ncol = 2, loc = 'upper center')
plt.savefig('norm.pdf', format='pdf')
plt.show()


