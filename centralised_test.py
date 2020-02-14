#!/usr/bin/env python

import logging
import time

import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import rospy
from geometry_msgs.msg import Point

URI = 'radio://0/100/2M/E7E7E7E701'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)



# Position Logging
def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    msg.x = x
    msg.y = y
    msg.z = z
    rospy.loginfo(msg)
    pub.publish(msg)
    #print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=250)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


# Main Centralised algorithm

def centr_avoid(scf):
    cf = scf.cf

    # Set Initial Values
    cf.param.set_value('kalman.initialX', '0.5')
    time.sleep(0.1)
    cf.param.set_value('kalman.initialY', '0.0')
    time.sleep(1)

    # Reset the Kalman Filter (Important!)
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # Read the inputs
    # MPC application size
    nsim = 100


    ctrl_applied = inputReader(nsim)
    print(ctrl_applied[99, 0])
    print(ctrl_applied[99, 1])



    # Send the velocity commands
    # commander.send_hover_setpoint(vx,vy,yaw_rate,z)

    # start up
    for y in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.25)
        time.sleep(0.1)
    #
    for y in range(nsim):
         cf.commander.send_hover_setpoint(ctrl_applied[y, 0], ctrl_applied[y, 1], 0, 0.25)
         time.sleep(0.1)

    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.1)
        time.sleep(0.1)
    #
    cf.commander.send_stop_setpoint()

def inputReader(nsim):
    input_matrix = np.zeros((nsim, 2))
    f = open("/home/antonis/admm_collsion_avoidance/testinputs.txt", "r")
    for i, line in enumerate(f):
        a = line.split(",")
        input_matrix[i, :] = [a[0], a[1]]
    f.close()
    return input_matrix


if __name__ == '__main__':

    #ROS variables
    pub = rospy.Publisher('CF1_position', Point, queue_size=10)
    rospy.init_node('centr', anonymous=True)
    msg = Point()

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        start_position_printing(scf)
        centr_avoid(scf)





