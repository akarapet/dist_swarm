import time
import logging

import numpy as np

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger


import rospy
import rosbag
from geometry_msgs.msg import Point

logging.basicConfig(level=logging.ERROR)

URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://1/90/2M/E7E7E7E702'
URI3 = 'radio://2/100/2M/E7E7E7E703'

init_pos1 = np.array([0.6, 0.0])
init_pos2 = np.array([1.2, 0.6])
init_pos3 = np.array([1.2, 1.5])


est_args = {
    URI1: [init_pos1],
    URI2: [init_pos2],
    URI3: [init_pos3],
}

uris = {
    URI1,
    URI2,
    URI3,
}


def inputReader(nsim, doc):
    input_matrix = np.zeros((nsim, 2))
    f = open("/home/antonis/admm_collsion_avoidance/"+doc, "r")
    for i, line in enumerate(f):
        a = line.split(",")
        input_matrix[i, :] = [a[0], a[1]]
    f.close()
    return input_matrix

# MPC application size
nsim = 100

ctrl_applied1 = inputReader(nsim, 'testinputs1.txt')
ctrl_applied2 = inputReader(nsim, 'testinputs2.txt')
ctrl_applied3 = inputReader(nsim, 'testinputs3.txt')

seq_args = {
    URI1: [ctrl_applied1],
    URI2: [ctrl_applied2],
    URI3: [ctrl_applied3],
}


def position_callback(uri,timestamp, data, logconf):

    if uri[-2:] == '01':
        msg = msg1
        pub = pub1
        bag = bag1
    elif uri[-2:] == '02':
        msg = msg2
        pub = pub2
        bag = bag2
    else:
        msg = msg3
        pub = pub3
        bag = bag3

    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    msg.x = x
    msg.y = y
    msg.z = z

    #rospy.loginfo(msg)
    #pub.publish(msg)
    bag.write('CF1_position', msg)



def start_position_printing(scf):

    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')


    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda t, d, l: position_callback(scf.cf.link_uri, t, d, l))
    log_conf.start()

def reset_estimator(scf, init_pos):
    cf = scf.cf

    # Set Initial Values
    cf.param.set_value('kalman.initialX', str(init_pos[0]))
    time.sleep(0.1)
    cf.param.set_value('kalman.initialY', str(init_pos[1]))
    time.sleep(1)

    # Reset the Kalman Filter (Important!)
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)


def run_sequence(scf, sequence):
    try:
        cf = scf.cf
        #print(sequence[99, 0])
        #print(sequence[99, 1])
        # Send the velocity commands
        # commander.send_hover_setpoint(vx,vy,yaw_rate,z)

        # start up
        for y in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.15)
            time.sleep(0.1)

        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.2)
            time.sleep(0.1)

        #
        for y in range(nsim):
            cf.commander.send_hover_setpoint(sequence[y, 0], sequence[y, 1], 0, 0.2)
            time.sleep(0.1)

        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.1)
            time.sleep(0.1)

        cf.commander.send_stop_setpoint()
    except Exception as e:
        print(e)

if __name__ == '__main__':

    # ROS Variables
    rospy.init_node('centr', anonymous=True)
    msg1 = Point()
    msg2 = Point()
    msg3 = Point()
    pub1 = rospy.Publisher('CF1_position', Point, queue_size=10)
    pub2 = rospy.Publisher('CF2_position', Point, queue_size=10)
    pub3 = rospy.Publisher('CF3_position', Point, queue_size=10)

    bag1 = rosbag.Bag('RosBags/cf1_swarm.bag', 'w')
    bag2 = rosbag.Bag('RosBags/cf2_swarm.bag', 'w')
    bag3 = rosbag.Bag('RosBags/cf3_swarm.bag', 'w')

    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel(start_position_printing)
        #time.sleep(5)
        swarm.parallel(reset_estimator, args_dict=est_args)
        swarm.parallel(run_sequence, args_dict=seq_args)

    bag1.close()
    bag2.close()
    bag3.close()




