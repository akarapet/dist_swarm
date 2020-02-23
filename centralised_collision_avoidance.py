#!/usr/bin/env python

import logging
import time


import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import rospy
import rosbag
from geometry_msgs.msg import Point

URI = 'radio://0/100/2M/E7E7E7E703'

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

    bag.write('CF1_position',msg)
    #print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
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
    cf.param.set_value('kalman.initialX', '1.0')
    time.sleep(0.1)
    cf.param.set_value('kalman.initialY', '1.0')
    time.sleep(1)

    # Reset the Kalman Filter (Important!)
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # Send the velocity commands
    # commander.send_hover_setpoint(vx,vy,yaw_rate,z)
    for y in range(50):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.25)
        time.sleep(0.1)

    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.1)
        time.sleep(0.1)
    bag.close()
    cf.commander.send_stop_setpoint()



if __name__ == '__main__':

    #ROS variables
    pub = rospy.Publisher('CF1_position', Point, queue_size=10)
    rospy.init_node('centr', anonymous=True)
    msg = Point()
    bag = rosbag.Bag('RosBags/test.bag', 'w')

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        start_position_printing(scf)
        centr_avoid(scf)





