#!/usr/bin/env python3
"""Check static root ownership and unrelated dynamic TF on a private ROS master."""
import collections
import sys
import time
import xml.etree.ElementTree as ET

from probe_support import Probe, instance, require, robot, wait


def main(binary):
    probe = Probe(binary, prefix='xgc2-origin-probe-')
    try:
        probe.start_master()
        probe.start_server(world_clock='wall')
        import rospy
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
        rospy.init_node('origin_probe', anonymous=True, disable_signals=True)
        dynamic = collections.deque(maxlen=500)
        scene_tf = collections.deque(maxlen=500)
        static = collections.deque(maxlen=10)
        subscribers = [rospy.Subscriber('/tf', TFMessage, dynamic.append, queue_size=100),
                       rospy.Subscriber('/tf_static', TFMessage, static.append, queue_size=10),
                       rospy.Subscriber('/xgc/tf', TFMessage, scene_tf.append, queue_size=100)]
        described = robot('described')
        described['kind'] = 'description_only'
        described['visualization'].update(sceneClass='', sceneModel='', robotStatePublisher=True,
                                           descriptionPackage='scout_description', descriptionFile='urdf/scout_visual.urdf')
        request = instance([robot('uav1'), described])
        probe.rpc('PUT', '/v1/instances/origin', request)
        wait(lambda: bool(static), 'static root was not latched')
        joints = rospy.Publisher('/described/joint_states', JointState, queue_size=1)
        pose = rospy.Publisher('/uav1/mavros/local_position/pose', PoseStamped, queue_size=1)
        xml = rospy.get_param('/described/robot_description')
        names = [j.attrib['name'] for j in ET.fromstring(xml).findall('joint')
                 if j.attrib.get('type') in ('continuous', 'revolute', 'prismatic')]
        require(names, 'fixture requires an articulated URDF')
        wait(lambda: joints.get_num_connections() and pose.get_num_connections(), 'inputs did not connect')
        def feed(seconds):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                stamp = rospy.Time.now()
                p = PoseStamped(); p.header.stamp = stamp; p.header.frame_id = 'map'
                p.pose.position.x = 2; p.pose.orientation.w = 1; pose.publish(p)
                j = JointState(); j.header.stamp = stamp; j.name = names; j.position = [.1] * len(names); joints.publish(j)
                time.sleep(.025)
        feed(1.2)
        wait(lambda: any(t.child_frame_id.startswith('described/') for m in dynamic for t in m.transforms),
             'unrelated articulated joint TF was not published on /tf')
        wait(lambda: any(t.child_frame_id == 'xgc/robots/uav1/base_link' for m in scene_tf for t in m.transforms),
             'robot scene body TF was not published')
        require(not any(t.child_frame_id == 'xgc_origin' for m in dynamic for t in m.transforms),
                'xgc_origin was duplicated on dynamic /tf')
        roots = [t for t in static[-1].transforms if t.child_frame_id == 'xgc_origin']
        require(len(roots) == 1 and roots[0].header.frame_id == 'world' and roots[0].header.stamp == rospy.Time(0),
                'static snapshot does not contain exactly one timeless world root')
        t = roots[0].transform
        require((t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
                == (0, 0, 0, 0, 0, 0, 1), 'static world root is not identity')
        aliases = [t for t in static[-1].transforms if t.child_frame_id == 'map']
        require(len(aliases) == 1 and aliases[0].header.frame_id == 'world', 'map alias changed')
        late = collections.deque(maxlen=2)
        late_sub = rospy.Subscriber('/tf_static', TFMessage, late.append, queue_size=1)
        wait(lambda: bool(late), 'late subscriber did not receive latched static TF')
        require(sum(t.child_frame_id == 'xgc_origin' for t in late[-1].transforms) == 1,
                'late subscriber did not receive exactly one world root')
        status = probe.rpc('GET', '/v1/instances/origin')
        require(status['publicationCounters']['global']['tf_root'] == 0,
                'retained compatibility channel counted dynamic root publications')
        probe.rpc('DELETE', '/v1/instances/origin')
        late_sub.unregister()
        for subscriber in subscribers: subscriber.unregister()
        joints.unregister(); pose.unregister()
        require(probe.stop() == 0, 'server did not stop cleanly')
        print('PASS: one latched static origin; dynamic /tf joints and /xgc/tf pose preserved')
    except Exception:
        print('PRIVATE PROCESS LOG:\n' + probe.log_tail(), file=sys.stderr)
        raise
    finally:
        probe.close()


if __name__ == '__main__':
    main(sys.argv[1])
