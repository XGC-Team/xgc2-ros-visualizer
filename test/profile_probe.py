#!/usr/bin/env python3
"""Per-robot profiles and rates on a finite, private ROS graph.

Starts its own roscore, runs the actual server executable and feeds it fake robot
state topics. It checks, through the master's own registry, the peer events of the
fake sources and the server's receipts, that
  * changing the profile of one robot rebuilds exactly that robot's resources;
  * a scene switch, a rate change and an unrelated robot never touch them;
  * per-robot rate overrides and the per-kind table take effect.
With a process definition as second argument the first server is started exactly as
that definition says, and what it declares is checked against the running process.
Usage: profile_probe.py /path/to/xgc2_ros_visualizer_node [process-definition.json]
"""
import collections
import copy
import json
import os
import signal
import sys
import time

from probe_support import Probe, instance, load_definition, require, robot, scout, wait


def main(binary, definition=None):
    probe = Probe(binary)
    try:
        probe.start_master()
        run(probe, load_definition(definition) if definition else None)
    except Exception:
        print('PRIVATE PROCESS LOG (last 5000 chars):\n' + probe.log_tail(), file=sys.stderr)
        raise
    finally:
        probe.close()


class Peers(object):
    """Subscribe and unsubscribe events of one fake source topic."""

    def __init__(self, rospy):
        class Listener(rospy.SubscribeListener):
            def __init__(self, owner):
                super(Listener, self).__init__()
                self.owner = owner

            def peer_subscribe(self, topic_name, topic_publish, peer_publish):
                self.owner.subscribed += 1

            def peer_unsubscribe(self, topic_name, num_peers):
                self.owner.unsubscribed += 1
        self.subscribed = 0
        self.unsubscribed = 0
        self.listener = Listener(self)


def run(probe, definition):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rosgraph_msgs.msg import Clock

    rospy.init_node('profile_probe', anonymous=True, disable_signals=True)
    clock = rospy.Publisher('/clock', Clock, queue_size=1)
    names = ['uav1', 'uav2', 'uav3']
    peers = {name: Peers(rospy) for name in names}
    sources = {name: rospy.Publisher('/%s/mavros/local_position/pose' % name, PoseStamped, queue_size=1,
                                     subscriber_listener=peers[name].listener) for name in names}
    paths = collections.defaultdict(list)
    for topic in ['/uav1/path', '/uav2/path', '/uav3/path', '/uav2/track']:
        rospy.Subscriber(topic, Path, lambda message, topic=topic: paths[topic].append(message), queue_size=100)

    state = {'time': 100.0}

    def pump(seconds, sources_to_feed=names):
        until = time.monotonic() + seconds
        previous = time.monotonic()
        while time.monotonic() < until:
            current = time.monotonic()
            state['time'] += current - previous
            previous = current
            stamp = rospy.Time.from_sec(state['time'])
            clock.publish(Clock(clock=stamp))
            for index, name in enumerate(sources_to_feed):
                message = PoseStamped()
                message.header.stamp = stamp
                message.header.frame_id = 'map'
                message.pose.position.x = state['time'] / 10 + index
                message.pose.position.z = 1
                message.pose.orientation.w = 1
                sources[name].publish(message)
            time.sleep(.02)

    def robots_status(run_id, name):
        return probe.rpc('GET', '/v1/instances/%s/robots/%s' % (run_id, name))

    def counters(run_id, name, channel):
        return robots_status(run_id, name)['publicationCounters'][channel]

    def rate_of(run_id, name, channel, seconds=2.0):
        """Publications per second of simulated time, from the server's own counter."""
        before, started = counters(run_id, name, channel), state['time']
        pump(seconds)
        return (counters(run_id, name, channel) - before) / (state['time'] - started)

    if definition:
        probe.start_from_definition(definition)
    else:
        probe.start_server()

    # ---- Describe: the readiness contract ---------------------------------------------------
    described = probe.rpc('GET', '/v1/describe')
    require(described['service'] == 'xgc2.visualization' and described['api_version'] == '1' and described['ready'],
            'describe lost the readiness envelope')
    require(described['facts']['world_clock'] == 'simulation' and described['facts']['instances'] == 0,
            'describe facts are wrong')
    started = time.monotonic()
    require(probe.rpc('GET', '/v1/describe?wait_ready_ms=2000', bound=False)['ready'] and time.monotonic() - started < 1,
            'a ready service held a describe call')
    probe.rpc('GET', '/v1/describe?wait_ready_ms=30001', expected=400)
    probe.rpc('GET', '/v1/health', expected=404)

    # ---- Create: three robots, one revision each --------------------------------------------
    run_id = 'run1'
    fleet = instance([robot(name) for name in names])
    receipt = probe.rpc('PUT', '/v1/instances/' + run_id, fleet)
    require(receipt['created'] and not receipt['unchanged'] and receipt['changes']['added'] == names,
            'creation receipt does not name the robots')
    require(receipt['configuration']['desiredRevision'] == receipt['configuration']['appliedRevision'] == 1
            and receipt['configuration']['persistedRevision'] is None, 'creation revisions')
    for name in names:
        status = robots_status(run_id, name)
        require(status['configuration']['desiredRevision'] == status['configuration']['appliedRevision'] == 1
                and status['generation'] == 1 and status['kind'] == 'fs150', 'robot %s revisions' % name)
    profile = robots_status(run_id, 'uav2')['profile']
    require(profile['state']['poseTopic'] == '/uav2/mavros/local_position/pose' and profile['path']['topic'] == 'path'
            and profile['state']['arPoseTopic'] == '/raw/uav2', 'resolved profile does not carry the derived sources')
    wait(lambda: all(p.subscribed >= 1 for p in peers.values()), 'robot inputs missing')
    pump(1.5)
    wait(lambda: all(paths['/uav%d/path' % i] for i in (1, 2, 3)), 'paths of all three robots missing')
    published, _ = probe.registry()
    require({'/uav1/path', '/uav2/path', '/uav3/path'} <= published, 'path topics are not advertised')
    require(all(p.subscribed == 1 and p.unsubscribed == 0 for p in peers.values()), 'unexpected input churn')

    # ---- Rebuild one robot: its saved profile changes ---------------------------------------
    profile['path']['topic'] = 'track'
    profile['labels']['color'] = '#ff0000'
    marks = {name: (peers[name].subscribed, peers[name].unsubscribed) for name in names}
    before = {name: len(paths['/%s/path' % name]) for name in ('uav1', 'uav3')}
    receipt = probe.rpc('PUT', '/v1/instances/%s/robots/uav2' % run_id, dict(profile=profile, expectedRevision=1))
    require(receipt['changes']['rebuilt'] == ['uav2'] and receipt['changes']['unchanged'] == ['uav1', 'uav3']
            and not receipt['changes']['added'] and not receipt['changes']['removed'] and not receipt['unchanged'],
            'robot receipt does not name the rebuilt robot only: %s' % receipt['changes'])
    require(receipt['configuration']['desiredRevision'] == receipt['configuration']['appliedRevision'] == 2
            and receipt['generation'] == 2 and receipt['profile']['path']['topic'] == 'track'
            and receipt['profile']['labels']['color'] == '#ff0000', 'robot revisions after rebuild')
    for name in ('uav1', 'uav3'):
        status = robots_status(run_id, name)
        require(status['configuration']['desiredRevision'] == 1 and status['generation'] == 1,
                '%s was rebuilt' % name)
    require(probe.rpc('GET', '/v1/instances/' + run_id)['configuration']['desiredRevision'] == 2, 'instance revision')
    wait(lambda: '/uav2/track' in probe.registry()[0] and '/uav2/path' not in probe.registry()[0],
         'the path output of the rebuilt robot did not move')
    wait(lambda: peers['uav2'].subscribed == marks['uav2'][0] + 1 and peers['uav2'].unsubscribed == marks['uav2'][1] + 1,
         'the rebuilt robot did not resubscribe its input')
    pump(1.5)
    require(all((peers[n].subscribed, peers[n].unsubscribed) == marks[n] for n in ('uav1', 'uav3')),
            'an unchanged robot lost its inputs')
    wait(lambda: paths['/uav2/track'], 'the rebuilt robot publishes no path')
    require(all(len(paths['/%s/path' % n]) > before[n] for n in ('uav1', 'uav3')), 'unchanged paths stopped')
    published, _ = probe.registry()
    require({'/uav1/path', '/uav3/path'} <= published, 'unchanged path outputs were withdrawn')

    # ---- Errors change nothing --------------------------------------------------------------
    stale = copy.deepcopy(profile)
    probe.rpc('PUT', '/v1/instances/%s/robots/uav2' % run_id, dict(profile=stale, expectedRevision=1), 409)
    broken = copy.deepcopy(profile)
    broken['labels']['fontSize'] = 99
    probe.rpc('PUT', '/v1/instances/%s/robots/uav2' % run_id, dict(profile=broken), 400)
    broken = copy.deepcopy(profile)
    broken['model']['description']['package'] = 'no_such_description'
    probe.rpc('PUT', '/v1/instances/%s/robots/uav2' % run_id, dict(profile=broken), 400)
    probe.rpc('PUT', '/v1/instances/%s/robots/uav2' % run_id, dict(profile=profile, extra=1), 400)
    probe.rpc('GET', '/v1/instances/%s/robots/ghost' % run_id, expected=404)
    probe.rpc('GET', '/v1/instances/nowhere/robots/uav1', expected=404)
    require(robots_status(run_id, 'uav2')['configuration']['desiredRevision'] == 2, 'a rejected profile changed a revision')
    same = probe.rpc('PUT', '/v1/instances/%s/robots/uav2' % run_id, dict(profile=profile, expectedRevision=2))
    require(same['unchanged'] and same['configuration']['desiredRevision'] == 2 and same['generation'] == 2,
            'an identical profile was not a no-op')

    # ---- The instance desired state replaces online adjustments -----------------------------
    receipt = probe.rpc('PUT', '/v1/instances/' + run_id, fleet)
    require(receipt['changes']['rebuilt'] == ['uav2'] and receipt['configuration']['desiredRevision'] == 3
            and not receipt['unchanged'], 'the unchanged frozen input did not restore the profile')
    wait(lambda: '/uav2/path' in probe.registry()[0] and '/uav2/track' not in probe.registry()[0], 'path output not restored')
    again = probe.rpc('PUT', '/v1/instances/' + run_id, fleet)
    require(again['unchanged'] and again['changes']['rebuilt'] == [] and again['configuration']['desiredRevision'] == 3,
            'an identical instance PUT was not a no-op')

    # ---- Per-robot saved profile and a scene switch -----------------------------------------
    saved = copy.deepcopy(fleet)
    saved['robots'][0]['visualization']['profile'] = dict(labels=dict(color='#00ff00'))
    marks = {name: (peers[name].subscribed, peers[name].unsubscribed) for name in names}
    receipt = probe.rpc('PUT', '/v1/instances/' + run_id, saved)
    require(receipt['changes']['rebuilt'] == ['uav1'], 'a saved profile rebuilt more than its robot')
    switched = copy.deepcopy(saved)
    switched['settings']['publication'] = dict(markers=True)
    marks['uav1'] = (peers['uav1'].subscribed, peers['uav1'].unsubscribed)
    receipt = probe.rpc('PUT', '/v1/instances/' + run_id, switched)
    require(receipt['changes']['scene'] and receipt['changes']['rebuilt'] == [] and receipt['changes']['added'] == [],
            'a scene switch is not a robot rebuild: %s' % receipt['changes'])
    wait(lambda: '/markers' in probe.registry()[0], 'the scene switch did not advertise the marker output')
    require(all((peers[n].subscribed, peers[n].unsubscribed) == marks[n] for n in names), 'a scene switch touched robot inputs')
    receipt = probe.rpc('PUT', '/v1/instances/' + run_id, saved)
    require(receipt['changes']['scene'] and receipt['changes']['rebuilt'] == [], 'the scene switch did not revert')
    wait(lambda: '/markers' not in probe.registry()[0], 'the marker output outlived its switch')
    pump(.5)

    # ---- Rates: defaults from the per-kind table, overrides per robot -----------------------
    table = probe.rpc('GET', '/v1/rates')
    require(table['desiredRevision'] == 1 and table['rates']['fs150']['path'] == 10, 'unexpected default table')
    base1 = rate_of(run_id, 'uav1', 'path')
    base2 = rate_of(run_id, 'uav2', 'path')
    require(7 <= base1 <= 12 and 7 <= base2 <= 12, 'default path rates %.1f %.1f' % (base1, base2))
    generation = robots_status(run_id, 'uav2')['generation']
    marks = {name: (peers[name].subscribed, peers[name].unsubscribed) for name in names}
    receipt = probe.rpc('PUT', '/v1/instances/%s/robots/uav2/rates' % run_id, dict(rates=dict(path=2)))
    require(receipt['rates']['overrides'] == {'path': 2} and receipt['rates']['effective']['path'] == 2
            and receipt['rates']['effective']['markers'] == 30 and not receipt['unchanged'], 'override receipt')
    require(receipt['configuration']['desiredRevision'] == receipt['configuration']['appliedRevision'] == 4,
            'a rate change is a revision of the robot')
    require(robots_status(run_id, 'uav2')['generation'] == generation, 'a rate change rebuilt the robot')
    low, other = rate_of(run_id, 'uav2', 'path', 3.0), rate_of(run_id, 'uav1', 'path')
    require(1.2 <= low <= 3.0 and 7 <= other <= 12, 'override rates %.1f %.1f' % (low, other))
    changed = probe.rpc('PUT', '/v1/instances/%s/robots/uav2/rates' % run_id, dict(rates=dict(path=2)))
    require(changed['unchanged'] and changed['configuration']['desiredRevision'] == 4, 'identical overrides')
    # The table is the default of everyone without an override.
    new_table = copy.deepcopy(table['rates'])
    new_table['fs150']['path'] = 5
    probe.rpc('PUT', '/v1/rates', dict(expectedRevision=1, rates=new_table))
    probe.rpc('PUT', '/v1/rates', dict(expectedRevision=1, rates=new_table), 409)
    mid1, mid2 = rate_of(run_id, 'uav1', 'path'), rate_of(run_id, 'uav2', 'path', 3.0)
    require(3.5 <= mid1 <= 6.5 and 1.2 <= mid2 <= 3.0, 'table rates %.1f %.1f' % (mid1, mid2))
    probe.rpc('PUT', '/v1/instances/%s/robots/uav2/rates' % run_id, dict(rates={}))
    follows = rate_of(run_id, 'uav2', 'path')
    require(3.5 <= follows <= 6.5, 'cleared override does not follow the table: %.1f' % follows)
    probe.rpc('PUT', '/v1/rates', dict(expectedRevision=2, rates=table['rates']))
    require(all((peers[n].subscribed, peers[n].unsubscribed) == marks[n] for n in names), 'rates touched robot inputs')
    # Validation and CAS.
    probe.rpc('PUT', '/v1/instances/%s/robots/uav2/rates' % run_id, dict(rates=dict(tf_root=5)), 400)
    probe.rpc('PUT', '/v1/instances/%s/robots/uav2/rates' % run_id, dict(rates=dict(path=0)), 400)
    probe.rpc('PUT', '/v1/instances/%s/robots/uav2/rates' % run_id, dict(rates=dict(path=1001)), 400)
    probe.rpc('PUT', '/v1/instances/%s/robots/uav2/rates' % run_id, dict(rates=dict(path=3), expectedRevision=1), 409)
    probe.rpc('PUT', '/v1/instances/%s/robots/ghost/rates' % run_id, dict(rates={}), 404)
    require(probe.rpc('GET', '/v1/instances/%s/robots/uav2/rates' % run_id)['rates']['overrides'] == {}, 'overrides leaked')

    # ---- Membership: remove one robot, add it back ------------------------------------------
    marks = {name: (peers[name].subscribed, peers[name].unsubscribed) for name in names}
    removed = probe.rpc('DELETE', '/v1/instances/%s/robots/uav3' % run_id)
    require(removed['removed'], 'robot removal')
    wait(lambda: '/uav3/path' not in probe.registry()[0], 'the removed robot still publishes')
    wait(lambda: peers['uav3'].unsubscribed == marks['uav3'][1] + 1, 'the removed robot still subscribes')
    require(probe.rpc('DELETE', '/v1/instances/%s/robots/uav3' % run_id)['removed'], 'removal is idempotent')
    probe.rpc('GET', '/v1/instances/%s/robots/uav3' % run_id, expected=404)
    listed = [row['id'] for row in probe.rpc('GET', '/v1/instances/' + run_id)['robots']]
    require(listed == ['uav1', 'uav2'], 'roster after removal: %s' % listed)
    require(all((peers[n].subscribed, peers[n].unsubscribed) == marks[n] for n in ('uav1', 'uav2')), 'removal touched siblings')
    added = probe.rpc('PUT', '/v1/instances/%s/robots/uav3' % run_id,
                      dict(profile=dict(model=dict(kind='fs150', description=dict(package='fs150_description',
                           file='urdf/fs150_visual.urdf')), state=dict(arPoseTopic='/raw/uav3'))))
    require(added['changes']['added'] == ['uav3'] and added['configuration']['desiredRevision'] == 1
            and added['generation'] == 1, 'robot addition receipt')
    wait(lambda: '/uav3/path' in probe.registry()[0], 'the added robot publishes no path')
    probe.rpc('PUT', '/v1/instances/%s/robots/uav3' % run_id, dict(profile={}, expectedRevision=1), 400)

    # ---- Another kind: a panel setting reaches only the robots it names ---------------------
    mixed = instance([robot('uav1'), robot('uav2'), scout('ugv1')])
    receipt = probe.rpc('PUT', '/v1/instances/' + run_id, mixed)
    require(receipt['changes']['added'] == ['ugv1'] and receipt['changes']['removed'] == ['uav3'], 'mixed membership')
    mixed['settings']['scoutLabelOffset'] = 0.9
    receipt = probe.rpc('PUT', '/v1/instances/' + run_id, mixed)
    require(receipt['changes']['rebuilt'] == ['ugv1'] and receipt['changes']['unchanged'] == ['uav1', 'uav2'],
            'a scout setting rebuilt other robots: %s' % receipt['changes'])
    require(robots_status(run_id, 'ugv1')['profile']['frames']['labelOffset'] == 0.9, 'scout label offset not applied')
    probe.rpc('PUT', '/v1/instances/%s/robots/ugv1/rates' % run_id, dict(rates=dict(height_projection=5)), 400)
    probe.rpc('PUT', '/v1/instances/%s/robots/ugv1/rates' % run_id, dict(rates=dict(scene=3)))

    # ---- Outputs are claimed per instance ---------------------------------------------------
    relay = [dict(source='/probe/map', topic='/xgc/display/probe/map', messageType='nav_msgs/OccupancyGrid')]
    probe.rpc('PUT', '/v1/instances/other', instance([robot('uav9')]), 409)
    probe.rpc('PUT', '/v1/instances/other', instance(relays=relay, publication=dict(transforms=False, scene=False)))
    uav9 = dict(profile=dict(model=dict(kind='fs150', description=dict(package='fs150_description',
                                                                       file='urdf/fs150_visual.urdf'))))
    probe.rpc('PUT', '/v1/instances/other/robots/uav9', uav9, 409)
    probe.rpc('GET', '/v1/instances/other/robots/uav9', expected=404)
    require(probe.rpc('GET', '/v1/instances/other')['relayCount'] == 1, 'the relay-only instance changed')
    probe.rpc('DELETE', '/v1/instances/other')

    # ---- Stop -------------------------------------------------------------------------------
    started = time.monotonic()
    require(probe.stop() == 0 and time.monotonic() - started < 3, 'Stop was not prompt and clean')
    require(not os.path.exists(probe.socket), 'the control socket outlived the server')

    # ---- The master is part of the binding: unreachable is reported, replaced ends the server
    probe.start_server()
    wait(lambda: probe.rpc('GET', '/v1/describe')['facts']['ros_master_state'] == 'connected',
         'server did not bind the private master')
    facts = probe.rpc('GET', '/v1/describe')['facts']
    require(facts['ros_master_uri'] == probe.master_uri and facts['ros_master_run_id'], 'describe lost the master identity')
    probe.kill_master()
    wait(lambda: probe.rpc('GET', '/v1/describe')['facts']['ros_master_state'] == 'unreachable',
         'a dead master was not reported', 8)
    require(probe.rpc('GET', '/v1/describe')['facts']['reason'] == 'ros master unreachable', 'wrong reason')
    require(probe.server.poll() is None, 'an unreachable master ended the server')
    started = time.monotonic()
    require(probe.rpc('GET', '/v1/describe?wait_ready_ms=200', bound=False)['ready'],
            'master absence changed own readiness')
    require(time.monotonic() - started < 1, 'ready unbound discovery waited for the master')
    probe.start_master()  # a new run on the same address
    wait(lambda: probe.server.poll() is not None, 'a replaced master did not end the server', 10)
    require(probe.server.returncode == 1 and not os.path.exists(probe.socket), 'a replaced master must fail the server')
    require('replaced' in probe.log_tail(), 'the reason was not logged')
    print(json.dumps(dict(ok=True, scope='private ROS graph; not station acceptance',
                          default_path_hz=round(base1, 1), override_path_hz=round(low, 1))))


if __name__ == '__main__':
    require(len(sys.argv) in (2, 3), 'expected the server executable and optionally its process definition')
    main(os.path.abspath(sys.argv[1]), sys.argv[2] if len(sys.argv) == 3 else None)
