# PID tunning of drone using GUI built in Python. Implemented in ROS (Robot Operating System) to tune pluto drone using whycon coordinates.
    
#     AUTHOR  : Nevil Pooniwala
#     EMAIL   : nevilpooniwala1997@gmail.com

#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
import rospy
import time
import matplotlib.pyplot as plt
from Tkinter import *

master = Tk()

x = y = z = yaw = 0

def callback(data):	#to get co-ordinates of whycon marker of drone.
	global x,y,z
	datao=data.poses[0]
	x=datao.position.x
	y=datao.position.y
	z=datao.position.z

x_pterm = x_iterm = x_dterm = y_pterm = y_iterm = y_dterm = z_pterm = z_iterm = z_dterm = yaw_pterm = yaw_iterm = yaw_dterm = SampleTimeTerm = 0

def x_p(event):
    global x_pterm
    x_pterm = pterm_x.get()

def x_i(event):
    global x_iterm
    x_iterm = iterm_x.get()

def x_d(event):
    global x_dterm
    x_dterm = dterm_x.get()

pterm_x = Scale(master, from_=0, to=5000, resolution=1, orient=HORIZONTAL, command=x_p, label = "x_p")
pterm_x.set(3.8)  	#75
pterm_x.pack(fill=BOTH, expand=YES)
iterm_x = Scale(master, from_=0, to=2, resolution=0.001, orient=HORIZONTAL, command=x_i, label = "x_i")
iterm_x.set(0.035) 	#0.25
iterm_x.pack(fill=BOTH, expand=YES)
dterm_x = Scale(master, from_=0, to=5000, resolution=1, orient=HORIZONTAL, command=x_d, label = "x_d")
dterm_x.set(30) 	#30
dterm_x.pack(fill=BOTH, expand=YES)


def y_p(event):
    global y_pterm
    y_pterm = pterm_y.get()

def y_i(event):
    global y_iterm
    y_iterm = iterm_y.get()

def y_d(event):
    global y_dterm
    y_dterm = dterm_y.get()

pterm_y = Scale(master, from_=0, to=5000, resolution=1, orient=HORIZONTAL, command=y_p, label = "y_p")
pterm_y.set(3.8)
pterm_y.pack(fill=BOTH, expand=YES)
iterm_y = Scale(master, from_=0, to=2, resolution=0.001, orient=HORIZONTAL, command=y_i, label = "y_i")
iterm_y.set(0.035)
iterm_y.pack(fill=BOTH, expand=YES)
dterm_y = Scale(master, from_=0, to=5000, resolution=1, orient=HORIZONTAL, command=y_d, label = "y_d")
dterm_y.set(30)
dterm_y.pack(fill=BOTH, expand=YES)


def z_p(event):
    global z_pterm
    z_pterm = pterm_z.get()

def z_i(event):
    global z_iterm
    z_iterm = iterm_z.get()

def z_d(event):
    global z_dterm
    z_dterm = dterm_z.get()

pterm_z = Scale(master, from_=0, to=5000, resolution=1, orient=HORIZONTAL, command=z_p, label = "z_p")
pterm_z.set(10)
pterm_z.pack(fill=BOTH, expand=YES)
iterm_z = Scale(master, from_=0, to=2, resolution=0.001, orient=HORIZONTAL, command=z_i, label = "z_i")
iterm_z.set(0.001)
iterm_z.pack(fill=BOTH, expand=YES)
dterm_z = Scale(master, from_=0, to=5000, resolution=1, orient=HORIZONTAL, command=z_d, label = "z_d")
dterm_z.set(30)
dterm_z.pack(fill=BOTH, expand=YES)


def yaw_p(event):
    global yaw_pterm
    yaw_pterm = pterm_yaw.get()

def yaw_i(event):
    global yaw_iterm
    yaw_iterm = iterm_yaw.get()

def yaw_d(event):
    global yaw_dterm
    yaw_dterm = dterm_yaw.get()

pterm_yaw = Scale(master, from_=0, to=300, resolution=1, orient=HORIZONTAL, command=yaw_p, label = "yaw_p")
pterm_yaw.set(75) 	#75
pterm_yaw.pack(fill=BOTH, expand=YES)
iterm_yaw = Scale(master, from_=0, to=2, resolution=0.001, orient=HORIZONTAL, command=yaw_i, label = "yaw_i")
iterm_yaw.set(0.07) 	#0.07
iterm_yaw.pack(fill=BOTH, expand=YES)
dterm_yaw = Scale(master, from_=0, to=300, resolution=1, orient=HORIZONTAL, command=yaw_d, label = "yaw_d")
dterm_yaw.set(5) 	#5
dterm_yaw.pack(fill=BOTH, expand=YES)



def s_t(event):
    global SampleTimeTerm
    SampleTimeTerm = sample_time_val.get()

sample_time_val = Scale(master, from_=0.0, to=8, resolution=0.01, orient=HORIZONTAL, command=s_t, label = "s_t")
sample_time_val.set(0.03)
sample_time_val.pack(fill=BOTH, expand=YES)



class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = SampleTimeTerm 	#0.03
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0  		#20.0

        self.output = 0.0

    def update(self, feedback_value):
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            """if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard"""

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
	    

class request_data():
	def __init__(self):
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)

	def access_data(self, req):
		 global yaw
		 yaw = req.yaw #magX
		 return PlutoPilotResponse(rcAUX2 =1500)

class send_data():
	def __init__(self):
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.cmd = PlutoMsg()
		self.cmd.rcRoll =1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcAUX1 =1500
		self.cmd.rcAUX2 =1500
		self.cmd.rcAUX3 =1500
		self.cmd.rcAUX4 =1000
		
	def arm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1000
		self.cmd.rcAUX4 =1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(.1)

	def reset(self):
		self.cmd.rcRoll =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcPitch =1500
		self.cmd.rcYaw = 1500
		self.command_pub.publish(self.cmd)

	def control_drone(self):
		count = 0; flag = 1; flag_1 = 1
		#command_graph = rospy.Publisher('/drone_graph', Int32, queue_size=1)
		
		
		#pid_yaw = PID()
	        #pid_yaw.Kp = 200; pid_yaw.Ki = 0; pid_yaw.Kd = 0; pid_yaw.SetPoint = 350

		#pid_x = PID()
		#pid_x.Kp = 100; pid_x.Ki = 0; pid_x.Kd = 0; pid_x.SetPoint = 4.07

		#pid_y = PID()
		#pid_y.Kp = 100; pid_y.Ki = 0; pid_y.Kd = 0; pid_y.SetPoint = -2

		#pid_z = PID()
		#pid_z.Kp = 100; pid_z.Ki = 0; pid_z.Kd = 0; pid_z.SetPoint = 26.5
		
		pid_yaw = PID()
	        pid_yaw.Kp = yaw_pterm; pid_yaw.Ki = yaw_iterm; pid_yaw.Kd = yaw_dterm; pid_yaw.SetPoint = 305

		pid_x = PID()
		pid_x.Kp = x_pterm; pid_x.Ki = x_iterm; pid_x.Kd = x_dterm; pid_x.SetPoint = 4

		pid_y = PID()
		pid_y.Kp = y_pterm; pid_y.Ki = y_iterm; pid_y.Kd = y_dterm; pid_y.SetPoint = 0

		pid_z = PID()
		pid_z.Kp = z_pterm; pid_z.Ki = z_iterm; pid_z.Kd = z_dterm; pid_z.SetPoint = 25

		while True:
			pid_yaw.Kp = yaw_pterm; pid_yaw.Ki = yaw_iterm; pid_yaw.Kd = yaw_dterm
			pid_x.Kp = x_pterm; pid_x.Ki = x_iterm; pid_x.Kd = x_dterm
			pid_y.Kp = y_pterm; pid_y.Ki = y_iterm; pid_y.Kd = y_dterm
			pid_z.Kp = z_pterm; pid_z.Ki = z_iterm; pid_z.Kd = z_dterm
			pid_yaw.sample_time = pid_x.sample_time = pid_y.sample_time = pid_z.sample_time = SampleTimeTerm
			#a = time.time()
			if flag == 1 and count > 35000:  #reduce 200000 with try and error
				self.arm()
				self.command_pub.publish(self.cmd)
				rospy.sleep(1)
				self.reset()
				self.command_pub.publish(self.cmd)
				flag = 2
							
			self.cmd.rcThrottle =1000
			pid_yaw.update(yaw)
			yaw_pid = 1500 + pid_yaw.output
			if (1000 <= yaw_pid <= 2000):	
				self.cmd.rcYaw = yaw_pid
			elif (1000 > yaw_pid):	
				self.cmd.rcYaw = 1000
			elif (yaw_pid > 2000):	
				self.cmd.rcYaw = 2000
			#self.command_pub.publish(self.cmd)
			pid_x.update(x)
			x_pid = 1500 + pid_x.output # +
			if (1000 <= x_pid <= 2000):
				self.cmd.rcPitch = x_pid # Roll
			elif (1000 > x_pid):	
				self.cmd.rcPitch = 1000
			elif (x_pid > 2000):	
				self.cmd.rcPitch = 2000
			#self.command_pub.publish(self.cmd)
			pid_y.update(y)
			y_pid = 1500 + pid_y.output  # -
			if (1000 <= y_pid <= 2000):
				self.cmd.rcRoll = y_pid # Pitch
			elif (1000 > y_pid):	
				self.cmd.rcRoll = 1000
			elif (y_pid > 2000):	
				self.cmd.rcRoll = 2000
			#self.command_pub.publish(self.cmd)
			pid_z.update(z)
			z_pid = 1500 - pid_z.output
			if (1000 <= z_pid <= 2000):	
				self.cmd.rcThrottle = z_pid
			elif (1000 > z_pid):	
				self.cmd.rcThrottle = 1000
			elif (z_pid > 2000):	
				self.cmd.rcThrottle = 2000
					

			#command_graph.publish(x_pid)
			"""command_graph.publish(x_pid)
			command_graph.publish(y_pid)
			command_graph.publish(z_pid)"""
			#plt.plot(count, yaw, yaw_pid, pid_yaw.PTerm, (pid_yaw.Ki *pid_yaw.ITerm), (pid_yaw.Kd * pid_yaw.DTerm))			
			print count, yaw, yaw_pid, pid_yaw.PTerm, (pid_yaw.Ki *pid_yaw.ITerm), (pid_yaw.Kd * pid_yaw.DTerm)
			#print x_pterm, x_iterm, x_dterm, y_pterm, y_iterm, y_dterm, z_pterm, z_iterm, z_dterm, yaw_pterm, yaw_iterm, yaw_dterm, SampleTimeTerm
			#pid_y.PTerm, (pid_y.Ki *pid_y.ITerm), (pid_y.Kd * pid_y.DTerm)
			self.command_pub.publish(self.cmd)
			count = count + 1
			master.update_idletasks()
    			master.update()
			#print time.time() - a
			#plt.ylabel('some numbers')
			#plt.show()
			

if __name__ == '__main__':
	rospy.init_node('drone_server')
	rospy.Subscriber('whycon/poses',PoseArray,callback)          #Subscriber function to receive whycon coordinates of the drone
	while not rospy.is_shutdown():
		test = send_data()
		test_2 = request_data()
		test.control_drone()
		rospy.spin()
		sys.exit(1)

