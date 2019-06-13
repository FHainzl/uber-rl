from math import pi

config = {
    "clock_freq": 10.0,
    "msg_proximity": 0.05,
    # Cannot be more than 20% of 1/clock_freq for skip checking to work
    "message_filter_q_size": 1,
    "verbose": False,

    "ROS_IP": "172.16.0.1",
    "ROS_PORT": 33333,
    "RL_IP": "172.16.0.3",
    "RL_PORT": 44444,
    "bufsize": 1024,

    "state_space_constraint": {'q1': (-pi / 10, pi / 10),
                               'q2': (-pi / 4, pi / 4),
                               'q3': (-pi / 0.1, pi / 0.1)},
}
