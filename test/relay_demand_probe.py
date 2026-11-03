#!/usr/bin/env python3
"""The relay subscribes its source only while /xgc/display<source> has consumers."""
import sys
import time

from probe_support import Probe, instance, require, wait


def relay(source, message_type):
    return dict(source=source, topic='/xgc/display' + source, messageType=message_type)


def main(binary):
    import rospy
    from nav_msgs.msg import Path

    probe = Probe(binary)
    try:
        probe.start_master()
        probe.start_server(world_clock='wall')
        rospy.init_node('relay_demand_probe', anonymous=True, disable_signals=True)
        source = '/probe/demand'
        displayed = []
        publisher = rospy.Publisher(source, Path, queue_size=10)
        probe.rpc('PUT', '/v1/instances/demand', instance(relays=[relay(source, 'nav_msgs/Path')]))

        def subscribed():
            return source in probe.registry()[1]

        def connections():
            return publisher.get_num_connections()

        time.sleep(0.3)
        require(not subscribed(), 'relay subscribed its source without a display consumer')
        require(connections() == 0, 'source had a relay connection without a display consumer')

        first = rospy.Subscriber('/xgc/display' + source, Path,
                                 lambda message: displayed.append(message), queue_size=20)
        wait(subscribed, 'relay did not subscribe after the first consumer')
        wait(lambda: connections() == 1, 'relay did not hold exactly one source connection')
        displayed[:] = []
        publisher.publish(Path())
        wait(lambda: displayed, 'first consumer did not receive the relay copy')

        second = rospy.Subscriber('/xgc/display' + source, Path,
                                  lambda message: displayed.append(message), queue_size=20)
        time.sleep(0.1)
        require(connections() == 1, 'second consumer duplicated the source subscription')
        second.unregister()
        time.sleep(0.1)
        require(connections() == 1, 'a remaining consumer lost the source subscription')
        first.unregister()
        wait(lambda: not subscribed() and connections() == 0,
             'last consumer did not release the source subscription')

        displayed[:] = []
        publisher.publish(Path())
        time.sleep(0.1)
        require(not displayed, 'relay published after its last consumer left')

        third = rospy.Subscriber('/xgc/display' + source, Path,
                                 lambda message: displayed.append(message), queue_size=20)
        wait(subscribed, 'reconnect did not subscribe the source again')
        wait(lambda: connections() == 1, 'reconnect did not hold one source connection')
        displayed[:] = []
        publisher.publish(Path())
        wait(lambda: displayed, 'reconnect did not receive the relay copy')
        third.unregister()
        wait(lambda: not subscribed() and connections() == 0,
             'reconnect release did not retract the source subscription')
    finally:
        probe.close()


if __name__ == '__main__':
    main(sys.argv[1])
