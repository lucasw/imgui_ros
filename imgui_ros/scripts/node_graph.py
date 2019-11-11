#!/usr/bin/env python
# Copyright 2019 Lucas Walter
# November 2019
# Get a list of all nodes, and what topics they publish and subscribe too.
# Advance work for generating a node graph.

import rosgraph
import rosnode
import rospy


if __name__ == '__main__':
    rospy.init_node('node_graph')
    node_names = rosnode.get_node_names()

    master = rosgraph.Master(rospy.get_name())
    print(master)
    try:
        state = master.getSystemState()
        pub_topics = master.getPublishedTopics('/')
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    for node_name in node_names:
        pubs = [t for t, l in state[0] if node_name in l]
        subs = [t for t, l in state[1] if node_name in l]
        srvs = [t for t, l in state[2] if node_name in l]

        rospy.loginfo(node_name)
        print('  pubs: {}'.format(pubs))
        print('  subs: {}'.format(subs))
