#!/usr/bin/env python3
"""Private loopback: actual ROS visualizer, peer Zenoh, profile switch and demand."""
import io
import os
import socket
import subprocess
import sys
import time

from probe_support import Probe, instance, require, robot, scout, wait


def wire(msg):
    data = io.BytesIO()
    msg.serialize(data)
    return data.getvalue().hex()


def main(binary, peer_binary):
    probe = Probe(binary, prefix='xgc2-zenoh-profile-')
    peer = None
    peer2 = None
    try:
        probe.start_master()
        import rospy
        from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
        from mavros_msgs.msg import State, ExtendedState
        from nav_msgs.msg import Path
        from tf2_msgs.msg import TFMessage
        rospy.init_node('zenoh_visualizer_probe', anonymous=True, disable_signals=True)
        with socket.socket() as s:
            s.bind(('127.0.0.1', 0))
            port = s.getsockname()[1]
        endpoint = 'tcp/127.0.0.1:' + str(port)
        keys = ['run/uav1/pose', 'run/uav1/ar', 'run/uav1/state', 'run/uav1/extended', 'run/uav1/prediction', 'run/ugv1/pose', 'run/ugv1/twist', 'run/ugv1/cmd_vel']
        probe.start_server(world_clock='wall')
        item = robot('uav1')
        sources = {name: dict(listen=endpoint, key=keys[index]) for index, name in enumerate(('pose', 'arPose', 'state', 'extendedState'))}
        overrides = {'uav1': dict(state=dict(sources=sources, worldOffset=[1, 2, 3]))}
        item2 = robot('uav2')
        overrides['uav2'] = dict(state=dict(sources=dict(pose=dict(listen=endpoint, key='run/uav2/pose'))))
        ground = scout('ugv1')
        overrides['ugv1'] = dict(state=dict(sources={name: dict(listen=endpoint, key=keys[index]) for name, index in [('pose', 5), ('twist', 6), ('cmdVel', 7)]}))
        relay = dict(source='/uav1/alg/prediction', topic='/xgc/display/uav1/alg/prediction', messageType='nav_msgs/Path',
                     role='prediction')
        relay_sources = {'/uav1/alg/prediction': dict(listen=endpoint, key=keys[4])}
        paths, paths2, ar_paths, predictions, rotor, wheel = [], [], [], [], [], []
        def collect_tf(message):
            for transform in message.transforms:
                if 'uav1/' in transform.child_frame_id and 'rotor' in transform.child_frame_id:
                    rotor.append(transform.transform.rotation.z)
                if 'ugv1/' in transform.child_frame_id and 'front_left_wheel_link' in transform.child_frame_id:
                    wheel.append(transform.transform.rotation.z)
        probe.rpc('PUT', '/v1/instances/zenoh', instance([item, item2, ground], [relay], settings=dict(robotProfiles=overrides, displayRelaySources=relay_sources)))
        def listener_open():
            with socket.socket() as sock:
                sock.settimeout(.2)
                return sock.connect_ex(('127.0.0.1', port)) == 0
        require(listener_open(), 'configured listener was not opened before consumers')
        peer = subprocess.Popen([peer_binary, str(port)] + keys, stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=probe.logfile, text=True, bufsize=1)
        require(peer.stdout.readline().strip() == 'READY', 'Zenoh peer failed to start')
        peer2 = subprocess.Popen([peer_binary, str(port), 'run/uav2/pose'], stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, stderr=probe.logfile, text=True, bufsize=1)
        require(peer2.stdout.readline().strip() == 'READY', 'second Zenoh peer failed to start')
        def command2(cmd):
            peer2.stdin.write(cmd + '\n'); peer2.stdin.flush()
            return peer2.stdout.readline().strip()

        def command(cmd):
            peer.stdin.write(cmd + '\n'); peer.stdin.flush()
            line = peer.stdout.readline().strip()
            require(bool(line), 'Zenoh peer closed')
            return line
        def matched(index):
            return command('status ' + str(index)) == 'STATUS 0 1'
        def put(index, payload):
            require(command('put %d %s' % (index, payload)) == 'PUT 0', 'Zenoh put failed')
        time.sleep(.3)
        for index in range(8):
            require(not matched(index), 'remote input subscribed without a display consumer')
        path_sub = rospy.Subscriber('/uav1/path', Path, paths.append, queue_size=10)
        wait(lambda: matched(0), 'member pose was not subscribed for its path', 8)
        require(not any(matched(index) for index in (1, 2, 3)), 'one path subscribed unused member inputs')
        require(not any(matched(index) for index in (5, 6, 7)), 'one robot path subscribed another robot')
        require(command2('status 0') == 'STATUS 0 0', 'one robot path subscribed the second sender')
        ar_sub = rospy.Subscriber('/uav1/ar_path', Path, ar_paths.append, queue_size=10)
        path2_sub = rospy.Subscriber('/uav2/path', Path, paths2.append, queue_size=10)
        tf_sub = rospy.Subscriber('/xgc/tf', TFMessage, collect_tf, queue_size=10)
        for index in (1, 2, 3, 5, 6, 7):
            wait(lambda index=index: matched(index), 'scene source not subscribed', 8)
        wait(lambda: command2('status 0') == 'STATUS 0 1', 'second sender not connected to shared listener', 8)
        require(not matched(4), 'prediction subscribed without a display consumer')
        local = rospy.Publisher('/uav1/mavros/local_position/pose', PoseStamped, queue_size=1)
        local_pose = PoseStamped(); local_pose.header.frame_id = 'map'; local_pose.pose.position.x = 19; local_pose.pose.orientation.w = 1
        for _ in range(5):
            local_pose.header.stamp = rospy.Time.now(); local.publish(local_pose); time.sleep(.03)
        require(not any(message.poses for message in paths), 'silent remote input fell back to local ROS')
        pred_sub = rospy.Subscriber('/xgc/display/uav1/alg/prediction', Path, predictions.append, queue_size=10)
        wait(lambda: matched(4), 'prediction input not enabled by consumer', 8)
        pose = PoseStamped(); pose.header.frame_id = 'map'; pose.pose.position.x = 4; pose.pose.orientation.w = 1
        ar = PoseStamped(); ar.header.frame_id = 'world'; ar.pose.position.x = 7; ar.pose.orientation.w = 1
        state = State(); state.armed = True
        extended = ExtendedState(); extended.landed_state = 2
        prediction = Path(); prediction.header.frame_id = 'world'; prediction.poses.append(ar)
        ground_pose = PoseStamped(); ground_pose.header.frame_id = 'world'; ground_pose.pose.orientation.w = 1
        twist = TwistStamped(); twist.header.frame_id = 'world'; twist.twist.linear.x = 0.5
        command_vel = Twist(); command_vel.linear.x = 0.5
        for _ in range(35):
            pose.header.stamp = ar.header.stamp = rospy.Time.now()
            ground_pose.header.stamp = twist.header.stamp = rospy.Time.now()
            put(0, wire(pose)); put(1, wire(ar)); put(2, wire(state)); put(3, wire(extended)); put(4, wire(prediction))
            put(5, wire(ground_pose)); put(6, wire(twist)); put(7, wire(command_vel))
            require(command2('put 0 '+wire(pose)) == 'PUT 0', 'second sender put failed')
            if paths and paths[-1].poses and paths2 and paths2[-1].poses and ar_paths and ar_paths[-1].poses and predictions and len(set(round(v, 4) for v in rotor)) >= 2 and len(set(round(v, 4) for v in wheel)) >= 2:
                break
            time.sleep(.05)
        require(paths and paths[-1].poses and abs(paths[-1].poses[-1].pose.position.x-4)<1e-6, 'remote pose not consumed')
        require(paths2 and paths2[-1].poses, 'second remote pose not consumed')
        require(ar_paths and ar_paths[-1].poses and abs(ar_paths[-1].poses[-1].pose.position.x-8)<1e-6, 'AR offset was not applied exactly once')
        require(predictions and predictions[-1].header.frame_id == 'world', 'remote Path not displayed')
        require(len(set(round(v, 4) for v in rotor)) >= 2, 'remote armed state did not animate rotors')
        require(len(set(round(v, 4) for v in wheel)) >= 2, 'remote ground twist did not animate wheels')
        peer2.stdin.close(); require(peer2.wait(timeout=5) == 0, 'second sender failed to stop')
        peer2 = subprocess.Popen([peer_binary, str(port), 'run/uav2/pose'], stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, stderr=probe.logfile, text=True, bufsize=1)
        require(peer2.stdout.readline().strip() == 'READY', 'second sender failed to restart')
        wait(lambda: command2('status 0') == 'STATUS 0 1', 'restarted sender failed to reconnect', 8)
        pose.pose.position.x = 9
        for _ in range(30):
            pose.header.stamp = rospy.Time.now()
            require(command2('put 0 '+wire(pose)) == 'PUT 0', 'restarted sender put failed')
            time.sleep(.05)
            if paths2 and paths2[-1].poses and abs(paths2[-1].poses[-1].pose.position.x-9)<1e-6: break
        require(paths2 and paths2[-1].poses and abs(paths2[-1].poses[-1].pose.position.x-9)<1e-6, 'restarted sender did not resume data')
        _, subscribers = probe.registry()
        require('/uav1/mavros/local_position/pose' not in subscribers and '/raw/uav1' not in subscribers and
                '/uav1/alg/prediction' not in subscribers, 'remote input unexpectedly republished/subscribed via ROS')
        # Short payload cannot replace the most recent valid pose or crash the server.
        for _ in range(5): put(0, '0102')
        require(probe.server.poll() is None, 'malformed payload stopped server')
        pred_sub.unregister()
        wait(lambda: not matched(4), 'prediction key survives last consumer', 8)
        pred_sub = rospy.Subscriber('/xgc/display/uav1/alg/prediction', Path, predictions.append, queue_size=10)
        wait(lambda: matched(4), 'prediction failed to resume after consumer reconnect', 8)
        pred_sub.unregister()
        wait(lambda: not matched(4), 'prediction key survived repeated consumer disconnect', 8)
        # Switching just this profile back to ROS retracts all four Zenoh keys.
        overrides.pop('uav1')
        probe.rpc('PUT', '/v1/instances/zenoh', instance([item, item2, ground], [relay], settings=dict(robotProfiles=overrides, displayRelaySources=relay_sources)))
        for index in range(4):
            wait(lambda index=index: not matched(index), 'profile replacement retained Zenoh subscription', 8)
        wait(lambda: '/uav1/mavros/local_position/pose' in probe.registry()[1], 'local ROS input was not restored')
        for _ in range(30):
            local_pose.header.stamp = rospy.Time.now(); local.publish(local_pose); time.sleep(.05)
            if paths and paths[-1].poses and abs(paths[-1].poses[-1].pose.position.x-19)<1e-6: break
        require(paths and paths[-1].poses and abs(paths[-1].poses[-1].pose.position.x-19)<1e-6, 'local ROS did not resume after profile switch')
        path_sub.unregister(); path2_sub.unregister(); ar_sub.unregister(); tf_sub.unregister(); local.unregister()
        for index in (5, 6, 7):
            wait(lambda index=index: not matched(index), 'ground input survives last display consumer', 8)
        wait(lambda: command2('status 0') == 'STATUS 0 0', 'second input survives last display consumer', 8)
        probe.rpc('DELETE', '/v1/instances/zenoh')
        wait(lambda: not listener_open(), 'listener survives last instance deletion', 8)
        require(probe.stop() == 0, 'server Stop failed')
    except Exception:
        print('PRIVATE PROCESS LOG:\n' + probe.log_tail(), file=sys.stderr)
        raise
    finally:
        for process in (peer, peer2):
            if process is not None:
                process.stdin.close()
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
        probe.close()


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
