#!/usr/bin/env python3
"""Private ROS start-order regressions; --long also holds each ROS peer down for 60 s."""
import collections
import json
import os
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

from probe_support import Probe, instance, require, robot, wait


def resources(pid):
    with open('/proc/%d/stat' % pid) as stream:
        fields = stream.read().rsplit(')', 1)[1].split()
    with open('/proc/%d/status' % pid) as stream:
        threads = next(int(line.split()[1]) for line in stream if line.startswith('Threads:'))
    return dict(cpu=(int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK'),
                threads=threads, fds=len(os.listdir('/proc/%d/fd' % pid)))


def source(probe, value):
    process = subprocess.Popen([sys.executable, __file__, '--publisher', str(value)],
                               stdout=probe.logfile, stderr=probe.logfile, start_new_session=True)
    probe.processes.append(process)
    return process


def stop_source(process):
    os.killpg(process.pid, signal.SIGTERM)
    process.wait(timeout=5)


def publish(value):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rosgraph_msgs.msg import Clock
    rospy.init_node('start_order_source')
    pose = rospy.Publisher('/uav1/mavros/local_position/pose', PoseStamped, queue_size=1)
    path = rospy.Publisher('/late/path', Path, queue_size=1)
    clock = rospy.Publisher('/clock', Clock, queue_size=1)
    started = time.monotonic()
    while not rospy.is_shutdown():
        stamp = rospy.Time.from_sec(100 + time.monotonic() - started)
        clock.publish(Clock(clock=stamp))
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = 'world'
        message.pose.orientation.w = 1
        message.pose.position.x = value
        pose.publish(message)
        message_path = Path()
        message_path.header = message.header
        message_path.poses = [message]
        path.publish(message_path)
        time.sleep(.02)


def rows():
    relay = dict(source='/late/path', topic='/xgc/display/late/path', messageType='nav_msgs/Path')
    return instance([robot('uav1')], relays=[relay])


def peer(probe):
    return probe.rpc('GET', '/v1/instances/run/robots/uav1')['input']['state']


def bound(probe):
    return probe.rpc('GET', '/v1/describe')['facts']['ros_master_state'] == 'connected'


def outage(probe, peer_type):
    """No traffic or test polling during the wall-clock measurement window."""
    before = resources(probe.server.pid)
    attempts = probe.rpc('GET', '/v1/describe')['facts']['ros_master_connect_attempts']
    probe.logfile.flush()
    log_start = os.path.getsize(probe.logfile.name)
    started = time.monotonic()
    time.sleep(60)
    elapsed = time.monotonic() - started
    after = resources(probe.server.pid)
    described = probe.rpc('GET', '/v1/describe')
    attempts = described['facts']['ros_master_connect_attempts'] - attempts
    with open(probe.logfile.name) as stream:
        stream.seek(log_start)
        lines = stream.read().splitlines()
    cpu = after['cpu'] - before['cpu']
    require(cpu < 1.2, '%s outage used %.3f CPU seconds' % (peer_type, cpu))
    require(after['threads'] <= before['threads'] and after['fds'] <= before['fds'],
            '%s outage grew threads/fds: %s -> %s' % (peer_type, before, after))
    require(len(lines) <= 7, '%s outage logged %d lines' % (peer_type, len(lines)))
    require(described['ready'], 'data peer outage changed own readiness')
    if peer_type == 'master':
        require(attempts <= 16, 'master exceeded the full-jitter 250 ms to 30 s retry budget: %d' % attempts)
        require(all('ROS master unreachable' in line for line in lines), 'unbounded XMLRPC diagnostics')
    return dict(peer=peer_type, wall_seconds=round(elapsed, 3), cpu_seconds=round(cpu, 3),
                master_attempts=attempts, log_lines=len(lines), before=before, after=after)


def run(probe, long_test):
    started = time.monotonic()
    probe.start_server(world_clock='simulation')
    require(time.monotonic() - started < 1, 'native startup waited for the absent master')
    described = probe.rpc('GET', '/v1/describe?wait_ready_ms=30000', bound=False)
    require(described['ready'] and described['facts']['ros_master_state'] == 'waiting for master',
            'describe depends on the master or topics')
    require(probe.rpc('GET', '/v1/status')['instanceCount'] == 0, 'initial roster is not empty')
    request = rows()
    first = probe.rpc('PUT', '/v1/instances/run', request)
    require(first['created'] and first['configuration']['appliedRevision'] == 1,
            'PUT could not load rows before the master')
    require(probe.rpc('PUT', '/v1/instances/run', request)['unchanged'], 'identical pending PUT rebuilt rows')
    require(peer(probe) == 'waiting for topic', 'missing input topic was not reported')
    time.sleep(max(0, started + 2 - time.monotonic()))
    probe.start_master()
    wait(lambda: bound(probe), 'late master was not bound', 10)
    wait(lambda: '/late/path' in probe.registry()[1], 'pending relay was not activated')
    require(peer(probe) == 'waiting for topic', 'topic absence stopped the applied robot')

    import rospy
    from nav_msgs.msg import Path
    rospy.init_node('start_order_probe', anonymous=True, disable_signals=True)
    displayed = collections.deque(maxlen=64)
    history = collections.deque(maxlen=64)
    observers = [rospy.Subscriber('/xgc/display/late/path', Path, displayed.append, queue_size=1),
                 rospy.Subscriber('/uav1/path', Path, history.append, queue_size=1)]

    def received(value):
        return (any(m.poses and m.poses[-1].pose.position.x == value for m in displayed)
                and any(m.poses and m.poses[-1].pose.position.x == value for m in history))

    publisher = source(probe, 11)
    wait(lambda: received(11) and peer(probe) == 'connected', 'late publisher data was not picked up', 10)
    original = probe.rpc('GET', '/v1/instances/run/robots/uav1')
    stop_source(publisher)
    wait(lambda: peer(probe) == 'waiting for topic', 'publisher loss was not reported')
    measurements = []
    if long_test:
        # Stop clock-dependent rendering too; the ROS subscriber remains active.
        time.sleep(.3)
        measurements.append(outage(probe, 'publisher'))
    publisher = source(probe, 22)
    wait(lambda: received(22) and peer(probe) == 'connected', 'restarted publisher did not reconnect', 10)
    current = probe.rpc('GET', '/v1/instances/run/robots/uav1')
    require(current['generation'] == original['generation'] == 1
            and current['configuration'] == original['configuration'], 'reconnect rebuilt desired state')
    require(probe.rpc('GET', '/v1/instances/run')['configuration']['appliedRevision'] == 1,
            'publisher events changed the applied revision')

    previous = probe.instance_id
    require(probe.stop() == 0, 'service stop failed')
    probe.start_server(world_clock='simulation')
    require(probe.instance_id != previous, 'service restart retained its incarnation')
    require(probe.rpc('GET', '/v1/describe', bound=False)['ready']
            and probe.rpc('GET', '/v1/status')['instanceCount'] == 0, 'restart persisted display rows')
    second = probe.rpc('PUT', '/v1/instances/run', request)
    for field in ('created', 'unchanged', 'changes', 'robotCount', 'descriptionCount', 'relayCount'):
        require(second[field] == first[field], 'identical PUT after restart changed ' + field)
    for before, after in zip(first['robots'], second['robots']):
        for field in ('id', 'kind', 'scene', 'configuration', 'generation'):
            require(before[field] == after[field], 'identical robot PUT after restart changed ' + field)
    require(second['configuration']['desiredRevision'] == second['configuration']['appliedRevision'] == 1
            and second['configuration']['persistedRevision'] is None, 'restart rejected/reset revisions incorrectly')
    require(probe.rpc('PUT', '/v1/instances/run', request)['unchanged'], 'replayed rows were not idempotent')
    wait(lambda: bound(probe) and peer(probe) == 'connected', 'replayed rows were not activated', 10)
    require(probe.rpc('GET', '/v1/instances/run/robots/uav1')['generation'] == 1,
            'row replay activated more than once')
    stop_source(publisher)
    for observer in observers:
        observer.unregister()
    rospy.signal_shutdown('private start-order probe complete')
    # No configured graph resources may need the replaced master during cleanup.
    probe.rpc('DELETE', '/v1/instances/run')
    probe.kill_master()
    wait(lambda: probe.rpc('GET', '/v1/describe')['facts']['ros_master_state'] == 'unreachable',
         'master loss was not reported', 8)
    if long_test:
        measurements.append(outage(probe, 'master'))
    probe.start_master()
    wait(lambda: probe.server.poll() is not None, 'replaced master did not end the service', 35 if long_test else 10)
    require(probe.server.returncode == 1 and not os.path.exists(probe.socket), 'master replacement left a stale binding')
    print(json.dumps(dict(ok=True, late_master_seconds=2, late_topic=True, publisher_restart=True,
                          ephemeral_identical_put=True, master_replacement=True, outages=measurements), sort_keys=True))


def bare_master(binary):
    probe = Probe(binary, prefix='xgc2-bare-master-')
    try:
        probe.start_server(world_clock='wall')
        started = time.monotonic()
        require(probe.stop() == 0 and time.monotonic() - started < 2,
                'SIGTERM waited for the absent master')
        # A listening peer that never replies exercises the XMLRPC time bound.
        # The kernel holds these connections; no fixture thread is needed.
        with socket.socket() as silent:
            silent.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            silent.bind(('127.0.0.1', probe.port))
            silent.listen(16)
            probe.start_server(world_clock='wall')
            wait(lambda: probe.rpc('GET', '/v1/describe')['facts']['ros_master_connect_attempts'] >= 2,
                 'unresponsive master monopolized the watcher', 3)
            started = time.monotonic()
            require(probe.rpc('PUT', '/v1/instances/silent', instance())['ready']
                    and time.monotonic() - started < 1, 'unresponsive master blocked control')
            started = time.monotonic()
            require(probe.stop() == 0 and time.monotonic() - started < 2,
                    'unresponsive master blocked SIGTERM')

        def start():
            master = subprocess.Popen(['rosmaster', '--core', '-p', str(probe.port)],
                                      stdout=probe.logfile, stderr=probe.logfile, start_new_session=True)
            probe.processes.append(master)
            probe.master = master

            def available():
                try:
                    return xmlrpc.client.ServerProxy(probe.master_uri).getPid('/probe')[0] == 1
                except (OSError, xmlrpc.client.Error):
                    return False
            wait(available, 'private bare master did not start')

        start()
        probe.start_server(world_clock='wall')
        wait(lambda: bound(probe), 'bare master was not bound')
        facts = probe.rpc('GET', '/v1/describe')['facts']
        require(facts['ros_master_pid'] and not facts['ros_master_run_id'], 'bare master identity is wrong')
        probe.logfile.flush()
        log_start = os.path.getsize(probe.logfile.name)
        started = time.monotonic()
        for _ in range(3):
            os.killpg(probe.master.pid, signal.SIGSTOP)
            wait(lambda: probe.rpc('GET', '/v1/describe')['facts']['ros_master_state'] == 'unreachable',
                 'paused master was not reported', 4)
            os.killpg(probe.master.pid, signal.SIGCONT)
            wait(lambda: bound(probe), 'same master did not recover', 4)
        with open(probe.logfile.name) as stream:
            stream.seek(log_start)
            failures = sum('ROS master unreachable' in line for line in stream)
        require(failures <= 1 + int((time.monotonic() - started) / 10),
                'master interruptions reset the log rate limit')
        probe.kill_master()
        start()
        wait(lambda: probe.server.poll() is not None, 'changed bare master PID was ignored', 10)
        require(probe.server.returncode == 1 and not os.path.exists(probe.socket),
                'bare master replacement retained its binding')
        print(json.dumps(dict(ok=True, bare_master_pid_fenced=True, stop_before_master=True,
                              unresponsive_master_bounded=True, interruptions_rate_limited=True)))
    except Exception:
        print('PRIVATE PROCESS LOG:\n' + probe.log_tail(), file=sys.stderr)
        raise
    finally:
        probe.close()


def main(binary, long_test):
    probe = Probe(binary, prefix='xgc2-start-order-')
    try:
        run(probe, long_test)
    except Exception:
        print('PRIVATE PROCESS LOG:\n' + probe.log_tail(), file=sys.stderr)
        raise
    finally:
        probe.close()
    bare_master(binary)


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--publisher':
        publish(float(sys.argv[2]))
    else:
        require(len(sys.argv) in (2, 3) and (len(sys.argv) == 2 or sys.argv[2] == '--long'),
                'usage: start_order_probe.py <node> [--long]')
        main(os.path.abspath(sys.argv[1]), len(sys.argv) == 3)
