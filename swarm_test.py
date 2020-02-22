import time

import numpy as np

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger

import rospy
import rosbag
from geometry_msgs.msg import Point

URI1 = 'radio://0/100/2M/E7E7E7E701'
URI2 = 'radio://0/100/2M/E7E7E7E702'
URI3 = 'radio://0/100/2M/E7E7E7E703'

init_pos1 = np.array([0.5, 0.0])
init_pos2 = np.array([1.0, 0.5])
init_pos3 = np.array([1.0, 1.5])


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

def position_callback(uri,timestamp, data, logconf):
    msg = Point()
    pub = rospy.Publisher('CF'+ uri[-2:] + 'position', Point, queue_size=10)
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    msg.x = x
    msg.y = y
    msg.z = z
    rospy.loginfo(msg)
    pub.publish(msg)

    #bag.write('CF1_position', msg)
    #print('pos: ({}, {}, {})'.format(x, y, z))


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


if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel(reset_estimator, args_dict=est_args)
        swarm.parallel(start_position_printing)

