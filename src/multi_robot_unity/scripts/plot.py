import matplotlib.pyplot as plt
import rospy
import tf
from gazebo_msgs.msg import ModelStates
import numpy as np
from matplotlib.animation import FuncAnimation
import matpltlib.pyplot as plt

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = self.ax.scatter([], [], 'ro')
        self.x_data = []
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.odom_callback)

    def plot_init(self):
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        return self.ln  

    def odom_callback(self, data):
        drone01 = data.pose[1]
        d01_posX = drone01.position.x
        d01_posY = drone01.position.y
        d01_posZ = drone01.position.z
        inner_pos = np.vstack((d01_posX, d01_posY)).T
        self.data = inner_pos
    
    def update_plot(self, frame):
        self.ln.set_offsets(self.data)
        return self.ln




def listener():
    rrospy.init_node('lidar_visual_node')
    vis = Visualiser()
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.pause(0.1)
    rospy.spin()

if __name__ == '__main__':
    start = time.time()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
