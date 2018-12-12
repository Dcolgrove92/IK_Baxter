import rospy
from scipy import io
import scipy.io as sio
import numpy as np
from collections import deque
from copy import deepcopy
import time
from rad_baxter_limb import RadBaxterLimb as baxter
from torque_controller.msg import q_cmd
import sys
from threading import RLock, Timer
from baxter_pykdl import baxter_kinematics as b_kin
#from rad_baxter_limb import Limb
from baxter_interface.limb import Limb
from baxter_interface import Gripper
from std_msgs.msg import UInt16


def __init__(self, limb_name, record = False, rate = 700):
        self.pub_joint_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size = 100)    #hz rate that joint states publish at
        self.rate = rate
        self.pub_joint_rate.publish(UInt16(self.rate))
        # if limb_name == 'left':
        #     self.Mb = M_left
        #     self.c = c_left
        #     self.f = f_left
        #     self.params = params_left
        #     self.fv = fv_left
        #     try:
        #         self.fc = fc_left
        #     except NameError:
        #         pass
        # else:
        #     self.M = M_right
        #     self.c = c_right
        #     self.params = params_right
        self.kin_kdl = b_kin(limb_name)
        self.joint_keys = { 0: 's0', 1: 's1', 2: 'e0', 3: 'e1', 4: 'w0', 5: 'w1', 6: 'w2'}
        self.n_jts = 7 #number of joints for baxter
        self.limb_name = limb_name
        self.q_cur = None
        self.qd_cur = None
        self.record = record
        self.q_history = deque()
        self.qd_history = deque()
        self.time_history = deque()
        self.qd_part_hist = deque()

        self.w = deque()
        self.w.append(np.array([0,0,0,0,0,0,0]))
        self.w.append(np.array([0,0,0,0,0,0,0]))
        self.w.append(np.array([0,0,0,0,0,0,0]))

        self.pos = deque()
        self.pos.append(np.array([0,0,0,0,0,0,0]))
        self.pos.append(np.array([0,0,0,0,0,0,0]))
        self.pos.append(np.array([0,0,0,0,0,0,0]))
        self.count = 0
             
        self.s1 = 0.003621681514929
        self.s2 = 1
        self.b1 = 1
        self.b2 = 2
        self.b3 = 1
        self.a1 = 1
        self.a2 = -1.822694925196308
        self.a3 = 0.837181651256023

        self.i = 0;

        self.pub_filt_vel = rospy.Publisher(self.limb_name + '/filterd_qd', q_cmd, queue_size = 100) 

        Limb.__init__(self, limb_name)


data = sio.loadmat('./joint.mat')
q = data['q_slns_pinv'] 
print q
#q1 = np.matrix([[0], [0], [0], [0], [0], [0], [0]])
#print q1
#q1 = (q[:,0])
#q2 = np.transpose(q[:,1])
#q3 = np.transpose(q[:,2])
#q4 = np.transpose(q[:,3])
#print [q1, q2, q3, q4]
print("Start Baxter?")
wait = raw_input("Press Enter to continue...")

arm = baxter('right')
gripper = Gripper('right')
gripper.calibrate()

print("Go to zero?")
wait = raw_input("Press Enter to continue...")

q_desi = np.array([0, 0, 0, 0, 0, 0, 0])

   # q_current = str(baxter(Limb).limb().get_joint_angles())#baxter(Limb).get_joint_angles()

q_current = arm.get_joint_angles()


q_diff = np.subtract(q_desi, q_current)
diff = np.abs(np.sum(q_diff))
while(diff > .01):
    arm.set_joint_positions_mod(q_desi)
    q_current = arm.get_joint_angles()
    diff = np.abs(np.sum(np.subtract(q_desi, q_current)))


print("Run Process?")
wait = raw_input("Press Enter to continue...")

gripper.command_position(100) # open gripper

q_desi = np.transpose(q[0:7,0])
while((np.abs(np.sum(q_desi - q_current)))> .01):
    arm.set_joint_positions_mod(q_desi)
    q_current = arm.get_joint_angles()
q_desi = np.transpose(q[0:7,1])
time.sleep(1)
while((np.abs(np.sum(q_desi - q_current)))> .01):
    arm.set_joint_positions_mod(q_desi)
    q_current = arm.get_joint_angles()
time.sleep(.5)
gripper.command_position(0) #  Close gripper

#time.sleep(.5)
q_desi = np.transpose(q[0:7,2])
while((np.abs(np.sum(q_desi - q_current)))> .01):
    arm.set_joint_positions_mod(q_desi)
    q_current = arm.get_joint_angles()
q_desi = np.transpose(q[0:7,3])
while((np.abs(np.sum(q_desi - q_current)))> .01):
    arm.set_joint_positions_mod(q_desi)
    q_current = arm.get_joint_angles()
time.sleep(1)
gripper.command_position(100) # open gripper
time.sleep(1)
q_desi = np.transpose(q[0:7,4])
while((np.abs(np.sum(q_desi - q_current)))> .01):
    arm.set_joint_positions_mod(q_desi)
    q_current = arm.get_joint_angles()

#time.sleep(2)
q_desi = np.array([0, 0, 0, 0, 0, 0, 0])
q_current = arm.get_joint_angles()


q_diff = np.subtract(q_desi, q_current)
diff = np.abs(np.sum(q_diff))
while(diff > .01):
    arm.set_joint_positions_mod(q_desi)
    q_current = arm.get_joint_angles()
    diff = np.abs(np.sum(np.subtract(q_desi, q_current)))



  # hlep from prof. killpack
  #
