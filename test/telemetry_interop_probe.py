#!/usr/bin/env python3
"""Private independent ROS masters: product Python sender -> Zenoh -> visualizer."""
import io
import json
import os
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

from probe_support import Probe, instance, require, robot, wait


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def emit(receipt):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State, ExtendedState
    from nav_msgs.msg import Path
    rospy.init_node('interop_robot', anonymous=True, disable_signals=True)
    local = rospy.Publisher('/uav1/mavros/local_position/pose', PoseStamped, queue_size=1)
    ar = rospy.Publisher('/uav1/pose', PoseStamped, queue_size=1)
    state = rospy.Publisher('/uav1/mavros/state', State, queue_size=1)
    ext = rospy.Publisher('/uav1/mavros/extended_state', ExtendedState, queue_size=1)
    pred = rospy.Publisher('/uav1/alg/prediction', Path, queue_size=1)
    class RawPath(rospy.AnyMsg):
        _type, _md5sum, _full_text, _has_header = Path._type, Path._md5sum, Path._full_text, Path._has_header
    def remember(message):
        with open(receipt, 'a', encoding='ascii') as log:
            log.write(message._buff.hex() + '\n')
    rospy.Subscriber('/uav1/alg/prediction', RawPath, remember, queue_size=10)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        a = PoseStamped(); a.header.frame_id = 'map'; a.header.stamp = stamp
        a.pose.position.x = 4.25; a.pose.orientation.w = 1
        b = PoseStamped(); b.header.frame_id = 'world'; b.header.stamp = stamp
        b.pose.position.x = 7.25; b.pose.orientation.w = 1
        flight = State(); flight.header.stamp = stamp; flight.armed = True
        landed = ExtendedState(); landed.landed_state = 2
        point = PoseStamped(); point.header.frame_id = 'world'; point.header.stamp = rospy.Time(41, 9)
        point.pose.position.x = 7.25; point.pose.orientation.w = 1
        path = Path(); path.header.frame_id = 'world'; path.header.stamp = rospy.Time(42, 7); path.poses.append(point)
        local.publish(a); ar.publish(b); state.publish(flight); ext.publish(landed); pred.publish(path)
        time.sleep(.05)


def main(binary, sender_root, sdk_site):
    probe = Probe(binary, prefix='xgc2-product-interop-')
    children = []
    try:
        probe.start_master()
        probe.start_server(world_clock='wall')
        import rospy
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        from tf2_msgs.msg import TFMessage
        rospy.init_node('interop_station', anonymous=True, disable_signals=True)
        robot_port, zenoh_port = free_port(), free_port()
        robot_uri = 'http://127.0.0.1:' + str(robot_port)
        env = dict(os.environ, ROS_MASTER_URI=robot_uri, ROS_IP='127.0.0.1')
        env.pop('ROS_HOSTNAME', None)
        master = subprocess.Popen(['roscore', '-p', str(robot_port)], env=env,
                                  stdout=probe.logfile, stderr=probe.logfile, start_new_session=True)
        children.append(master)
        def master_ready():
            try: return xmlrpc.client.ServerProxy(robot_uri).getPid('/interop')[0] == 1
            except OSError: return False
        wait(master_ready, 'robot master unavailable', 15)
        endpoint = 'tcp/127.0.0.1:' + str(zenoh_port)
        keys = ['interop/uav1/pose', 'interop/uav1/ar', 'interop/uav1/state', 'interop/uav1/extended', 'interop/uav1/prediction']
        item = robot('uav1')
        sources = {name: dict(listen=endpoint, key=keys[index]) for index, name in enumerate(('pose', 'arPose', 'state', 'extendedState'))}
        settings = dict(robotProfiles={'uav1': dict(state=dict(sources=sources, worldOffset=[1, 2, 3]))},
                        displayRelaySources={'/uav1/alg/prediction': dict(listen=endpoint, key=keys[4])})
        relay = dict(source='/uav1/alg/prediction', topic='/xgc/display/uav1/alg/prediction', messageType='nav_msgs/Path', role='prediction')
        paths, ar_paths, predictions, rotors = [], [], [], []
        def read_tf(message):
            for transform in message.transforms:
                if 'uav1/' in transform.child_frame_id and 'rotor' in transform.child_frame_id:
                    rotors.append(round(transform.transform.rotation.z, 4))
        probe.rpc('PUT', '/v1/instances/interop', instance([item], [relay], settings=settings))
        mappings = [('/uav1/mavros/local_position/pose', 'geometry_msgs/PoseStamped'),
                    ('/uav1/pose', 'geometry_msgs/PoseStamped'), ('/uav1/mavros/state', 'mavros_msgs/State'),
                    ('/uav1/mavros/extended_state', 'mavros_msgs/ExtendedState'), ('/uav1/alg/prediction', 'nav_msgs/Path')]
        config = {'connect': [endpoint], 'topics': [dict(topic=t, type=k, key=key, max_hz=20) for (t, k), key in zip(mappings, keys)]}
        env['PYTHONPATH'] = sender_root + os.pathsep + sdk_site + os.pathsep + env.get('PYTHONPATH', '')
        verify = subprocess.run([sys.executable, '-c', 'import importlib.metadata; assert importlib.metadata.version("eclipse-zenoh")=="1.5.1"'], env=env)
        require(verify.returncode == 0, 'product sender SDK is not 1.5.1')
        sender = subprocess.Popen([sys.executable, '-m', 'onboard_telemetry', '--config-json', json.dumps(config)], env=env,
                                  stdout=probe.logfile, stderr=probe.logfile, start_new_session=True)
        children.append(sender)
        # The sender must connect before any visualizer output is subscribed.
        # A client-mode Zenoh 1.5.1 sender exits immediately if there is no listener.
        time.sleep(1.5)
        require(sender.poll() is None, 'sender could not connect without a display consumer')
        path_sub = rospy.Subscriber('/uav1/path', Path, paths.append, queue_size=1)
        ar_sub = rospy.Subscriber('/uav1/ar_path', Path, ar_paths.append, queue_size=1)
        pred_sub = rospy.Subscriber('/xgc/display/uav1/alg/prediction', Path, predictions.append, queue_size=1)
        tf_sub = rospy.Subscriber('/xgc/tf', TFMessage, read_tf, queue_size=10)
        receipt = os.path.join(probe.work, 'prediction-wire.hex')
        pub = subprocess.Popen([sys.executable, __file__, '--emit', receipt], env=env,
                               stdout=probe.logfile, stderr=probe.logfile, start_new_session=True)
        children.append(pub)
        def received():
            return (paths and paths[-1].poses and abs(paths[-1].poses[-1].pose.position.x-4.25)<1e-6 and
                    ar_paths and ar_paths[-1].poses and abs(ar_paths[-1].poses[-1].pose.position.x-8.25)<1e-6 and
                    predictions and predictions[-1].header.stamp == rospy.Time(42, 7) and predictions[-1].poses and len(set(rotors)) >= 2)
        wait(lambda: sender.poll() is not None or received(), 'sender -> receiver data did not arrive', 25)
        require(sender.poll() is None and received(), 'sender exited or data is wrong')
        pubs, subs = probe.registry()
        for raw, _ in mappings:
            require(raw not in pubs and raw not in subs, 'robot source leaked into station ROS graph: '+raw)
        actual_bytes = io.BytesIO(); predictions[-1].serialize(actual_bytes)
        def saw_source_bytes():
            try:
                with open(receipt, encoding='ascii') as log: return actual_bytes.getvalue().hex() in log.read().splitlines()
            except FileNotFoundError: return False
        wait(saw_source_bytes, 'prediction ROS bytes changed in transit', 5)
        require(probe.server.poll() is None, 'visualizer exited')
        sender.send_signal(signal.SIGINT); sender.wait(timeout=8)
        require(sender.returncode == 0, 'product sender did not shut down')
        probe.rpc('DELETE', '/v1/instances/interop')
        path_sub.unregister(); ar_sub.unregister(); pred_sub.unregister(); tf_sub.unregister()
        require(probe.stop() == 0, 'visualizer did not stop')
        print('PASS: Python Zenoh 1.5.1 product sender -> Zenoh C 1.8.0 visualizer, isolated ROS masters')
    except Exception:
        print('PRIVATE PROCESS LOG:\n' + probe.log_tail(), file=sys.stderr)
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
                try: child.wait(timeout=5)
                except subprocess.TimeoutExpired: os.killpg(child.pid, signal.SIGKILL); child.wait()
        probe.close()


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--emit': emit(sys.argv[2])
    else: main(sys.argv[1], sys.argv[2], sys.argv[3])
