#!/usr/bin/env python3
"""Finite, private ROS graph acceptance for the actual single server executable.
Run only in an independent build/install container. Never uses a station master.
"""
import collections
import copy
import json
import os
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

from probe_support import Probe, require, wait


def resources(pid):
    values = {}
    with open('/proc/%d/status' % pid) as stream:
        for line in stream:
            if line.startswith(('VmRSS:', 'Threads:')):
                key, value = line.split(':', 1)
                values[key] = int(value.split()[0])
    children = []
    for tid in os.listdir('/proc/%d/task' % pid):
        with open('/proc/%d/task/%s/children' % (pid, tid)) as stream:
            children.extend(stream.read().split())
    values['children'] = children
    require(not children, 'server forked a child process')
    return values


def robot(name, ar=False):
    return dict(slotId=name, namespace='/' + name, kind='px4_multirotor',
                visualization=dict(sceneClass='fs150', sceneModel=name,
                    descriptionPackage='fs150_description', descriptionFile='urdf/fs150_visual.urdf',
                    robotStatePublisher=False, jointStateTopic='joint_states', pathTopic='path'),
                hybridSource='physical' if ar else 'simulation')


def frozen_robot(row, ar=False):
    name = row['slotId']
    return dict(slotId=name, namespace=row['namespace'], source='physical' if ar else 'simulation',
                profile=dict(id='px4-multirotor.physical.vrpn' if ar else 'px4-multirotor.xsim.ros',
                    parameters=dict(localization_pose_topic=(
                        '/vrpn_client_node_physical/' + name + '/pose' if ar else '/raw/' + name))))


def instance(count=0, relays=None, scene=True):
    rows = [robot('uav%d' % (i + 1), i == 0) for i in range(count)]
    return dict(robots=rows,
                context=dict(runMode='hybrid', worldBoundary=None, scene=dict(simulator='xsim'),
                             localizationOffset=dict(x=1, y=2, z=3),
                             robots=[frozen_robot(row, i == 0) for i, row in enumerate(rows)]),
                settings=dict(publication=dict(transforms=scene, scene=scene, markers=scene,
                                               scenePaths=scene, paths=scene)), displayRelays=relays or [])


def main(binary):
    probe = Probe(binary, 'xgc2-single-server-probe-')
    try:
        probe.start_master()
        run(probe)
    except Exception:
        print('PRIVATE PROCESS STATES: ' + repr([(p.pid, p.poll()) for p in probe.processes]), file=sys.stderr)
        print('PRIVATE PROCESS LOG (last 5000 chars):\n' + probe.log_tail(), file=sys.stderr)
        raise
    finally:
        probe.close()


def run(probe):
    work, master_uri = probe.work, probe.master_uri
    rpc = probe.rpc
    import rospy
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Pose, PoseStamped, PoseArray
    from nav_msgs.msg import OccupancyGrid, Path
    from sensor_msgs.msg import PointCloud2, JointState
    from std_msgs.msg import Empty, Header
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import MarkerArray, Marker
    from foxglove_msgs.msg import SceneUpdate
    rospy.init_node('visualizer_private_probe', anonymous=True, disable_signals=True)
    clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
    # The private Python source exposes 100 topics through one TCPROS
    # listener (rospy defaults to backlog 5). Size this test fixture for
    # the simultaneous connection burst; production server is untouched.
    from rospy.impl import tcpros_base
    private_source = tcpros_base.init_tcpros_server()
    private_source.start_server()
    private_source.tcp_ros_server.server_sock.listen(256)

    # ---- Startup: explicit options only -----------------------------------------------------
    def reject_startup(arguments, environment=None, message=None):
        rejected = subprocess.run([probe.binary] + arguments, env=environment,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        require(rejected.returncode != 0 and not os.path.exists(probe.socket),
                'invalid/retired startup input created a provider')
        if message:
            require(message in rejected.stderr.decode(), 'unexpected startup rejection reason: ' + rejected.stderr.decode())
    base = ['--socket-path', probe.socket]
    reject_startup([], message='--socket-path is required')
    for arguments in (["--bootstrap-input", os.path.join(work, 'bootstrap.json')], ["--rates-json", "{}"],
                      ["--target-id", "duplicate"], ["--socket", probe.socket], ["--initial-instance-file", "/dev/null"],
                      ["--callback-workers", "0"], ["--callback-workers", "33"], ["--callback-workers", "many"],
                      ["--world-clock", "guess"], ["--world-clock", "wall", "--world-clock", "wall"],
                      ["--socket-path", probe.socket], ["_server_instance_id:=old"], ["__log:=" + work + "/unallocated.log"],
                      ["positional"]):
        reject_startup(base + arguments)
    missing_allocation = dict(os.environ)
    missing_allocation.pop('ROS_HOME', None)
    reject_startup(base, environment=missing_allocation, message='ROS_HOME requires')
    nonprivate = os.path.join(work, 'nonprivate')
    os.mkdir(nonprivate, 0o750)
    reject_startup(['--socket-path', os.path.join(nonprivate, 'control.sock')])
    require(not os.path.exists(os.path.join(nonprivate, 'control.sock')), 'nonprivate runtime directory got an endpoint')

    server = probe.start_server(world_clock='simulation', arguments=['--callback-workers', '2',
                                                                      '/use_sim_time:=/xgc2_ros_visualizer/use_sim_time'])
    description = rpc('GET', '/v1/describe')
    require(description['ready'] and description['facts']['callback_workers'] == 2, 'wrong input pool size')
    require(rpc('GET', '/v1/status')['instanceCount'] == 0, 'startup implicitly activated an instance')
    legacy = dict(instanceId='old', robots=[], context=dict(runMode='simulation', worldBoundary=None), settings={}, displayRelays=[])
    rpc('PUT', '/v1/instances/retired-input', legacy, 400)
    projected = dict(robots=[], descriptions=[], worldBoundary=None, settings={}, displayRelays=[])
    rpc('PUT', '/v1/instances/retired-input', projected, 400)
    rpc('GET', '/v1/instances/retired-input', expected=404)
    receipt = rpc('PUT', '/v1/instances/explicit-zero', instance())
    require(receipt['ready'] and receipt['configuration']['desiredRevision'] == 1 and
            receipt['configuration']['appliedRevision'] == 1 and
            receipt['configuration']['persistedRevision'] is None, 'zero activation receipt invalid')
    ready = []
    rospy.Subscriber('/xgc/robot_scene/ready', Empty, lambda _: ready.append(True), queue_size=1)
    sim_time = 100.0

    def stamp():
        return rospy.Time.from_sec(sim_time)

    def pump(seconds, publishers=(), ar_publisher=None, joint_publisher=None, frame='map'):
        nonlocal sim_time
        until = time.monotonic() + seconds
        previous = time.monotonic()
        while time.monotonic() < until:
            current = time.monotonic()
            sim_time += 3 * (current - previous)
            previous = current  # Actual RTF=3 even when publishing 100 slots costs CPU.
            clock_pub.publish(Clock(clock=stamp()))
            for index, publisher in enumerate(publishers):
                msg = PoseStamped(); msg.header.stamp = stamp(); msg.header.frame_id = frame
                msg.pose.position.x = sim_time / 10 + index; msg.pose.position.z = 1
                msg.pose.orientation.w = 1; publisher.publish(msg)
            if ar_publisher:
                msg = PoseStamped(); msg.header.stamp = stamp(); msg.header.frame_id = 'frozen'
                msg.pose.position.x = 1; msg.pose.position.y = 2; msg.pose.position.z = .5
                msg.pose.orientation.w = 2; ar_publisher.publish(msg)
            if joint_publisher:
                joint_publisher[0].publish(JointState(header=Header(stamp=stamp()),
                    name=joint_publisher[1], position=[.5] * len(joint_publisher[1])))
            time.sleep(.02)
    pump(.15)
    wait(lambda: bool(ready), 'zero-robot explicit activation readiness missing')
    require(rospy.get_param('/xgc2_ros_visualizer/use_sim_time'), 'clock remap not honored')
    rpc('DELETE', '/v1/instances/explicit-zero')
    rpc('GET', '/v1/instances/explicit-zero', expected=404)
    baseline = resources(server.pid)

    # ---- The rate table: complete replacement under a revision ------------------------------
    rate_revision = 1

    def put_rates(table, expected=200, revision=None):
        nonlocal rate_revision
        result = rpc('PUT', '/v1/rates', dict(expectedRevision=rate_revision if revision is None else revision, rates=table), expected)
        if expected == 200:
            rate_revision = result['appliedRevision']
            require(result['desiredRevision'] == rate_revision and result['persistedRevision'] is None,
                    'rate receipt conflates desired/applied/persisted')
        return result
    default_rates = rpc('GET', '/v1/rates')['rates']
    require(default_rates['fs150']['path'] == 10, 'unexpected default table')
    rpc('PUT', '/v1/rates', dict(expectedRevision=1, rates={'fs150': {'path': 7}}), 400)
    require(rpc('GET', '/v1/rates')['rates'] == default_rates, 'partial runtime PUT changed rates')
    rpc('PUT', '/v1/rates', default_rates, 400)
    require(rpc('GET', '/v1/rates')['appliedRevision'] == rate_revision, 'retired flat rate body mutated state')
    put_rates(default_rates)
    put_rates(default_rates, 409, revision=1)
    require(rpc('GET', '/v1/rates')['appliedRevision'] == 2, 'stale CAS changed rates')
    bad = copy.deepcopy(default_rates); bad['fs150']['path'] = 0
    put_rates(bad, 400)
    require(rpc('GET', '/v1/rates')['rates'] == default_rates, 'bad rate changed configuration')
    rpc('GET', '/health', expected=404)
    rpc('GET', '/v1/xrpc/policy', expected=404)

    # Four full-state types: subscribe without peers, forward original bytes,
    # map/grid retain one snapshot for a late viewer; path/poses do not replay.
    # All four stay subscribed after display peers leave.
    types = [PointCloud2, OccupancyGrid, Path, PoseArray]
    type_names = ['sensor_msgs/PointCloud2', 'nav_msgs/OccupancyGrid', 'nav_msgs/Path', 'geometry_msgs/PoseArray']
    sources = ['/probe/cloud', '/probe/grid', '/probe/path', '/probe/poses']
    relay_specs = [dict(source=s, topic='/xgc/display' + s, messageType=t) for s, t in zip(sources, type_names)]
    source_pubs = [rospy.Publisher(s, t, queue_size=1) for s, t in zip(sources, types)]
    original = [collections.deque(maxlen=256) for _ in types]
    displayed = [collections.deque(maxlen=256) for _ in types]
    for i, s in enumerate(sources):
        rospy.Subscriber(s, rospy.AnyMsg, lambda m, i=i: original[i].append(bytes(m._buff)), queue_size=100)
    foreign = instance(relays=relay_specs, scene=False)
    rpc('PUT', '/v1/instances/foreign-relay', foreign)
    wait(lambda: all(p.get_num_connections() >= 2 for p in source_pubs), 'relay did not subscribe before a viewer')
    rospy.set_param('/foreign/visual_robot_description', 'foreign-owned')

    def packets(count):
        for n in range(count):
            for i, publisher in enumerate(source_pubs):
                msg = types[i](); msg.header.seq = n + 31; msg.header.stamp = stamp(); msg.header.frame_id = 'original'
                if i == 0:
                    msg.width = 256; msg.height = 1; msg.point_step = 4; msg.row_step = 1024; msg.data = bytes([n % 256]) * 1024
                elif i == 1:
                    msg.info.width = 2; msg.info.height = 1; msg.info.resolution = .125; msg.data = [-1, n % 100]
                elif i == 2:
                    for x in (n * .01, n * .01 + 1):
                        sample = PoseStamped(); sample.header = copy.deepcopy(msg.header)
                        sample.pose.position.x = x; sample.pose.orientation.w = 1; msg.poses.append(sample)
                else:
                    for x in (n * .01, n * .01 + 1):
                        sample = Pose(); sample.position.x = x; sample.orientation.w = 1; msg.poses.append(sample)
                publisher.publish(msg)
            time.sleep(.01)
    packets(20); time.sleep(.25)
    peers = [rospy.Subscriber('/xgc/display' + s, rospy.AnyMsg, lambda m, i=i: displayed[i].append(bytes(m._buff)), queue_size=100) for i, s in enumerate(sources)]
    wait(lambda: all(peer.get_num_connections() for peer in peers), 'display publishers missing')
    wait(lambda: len(displayed[0]) == 1 and len(displayed[1]) == 1,
         'late display did not receive exactly one retained map/grid snapshot')
    time.sleep(.15)
    require(len(displayed[0]) == 1 and len(displayed[1]) == 1, 'snapshot replay repeated without a source update')
    require(all(displayed[i][0] in original[i] for i in (0, 1)), 'retained snapshot changed serialized payload')
    require(not displayed[2] and not displayed[3], 'path or poses replayed an already-published sample')
    for samples in displayed:
        samples.clear()
    rate_table = copy.deepcopy(default_rates)
    for key in ['display_pointcloud', 'display_grid', 'display_path', 'display_pose_array']:
        rate_table['global'][key] = 5
    put_rates(rate_table)
    packets(80); time.sleep(.25)
    for i in range(4):
        require(len(original[i]) >= 40, 'original recorder was throttled')
        require(1 <= len(displayed[i]) <= 8, 'relay ceiling/delivery failed')
        require(all(sample in original[i] for sample in displayed[i]), 'relay changed raw serialized payload')
    relay_counts = [len(samples) for samples in displayed]
    for peer in peers:
        peer.unregister()
    wait(lambda: all(p.get_num_connections() >= 2 for p in source_pubs), 'disconnect removed source subscription')
    # A changed desired state is applied: one relay more, the others untouched.
    extra = dict(source='/probe/extra', topic='/xgc/display/probe/extra', messageType='nav_msgs/Path')
    changed = copy.deepcopy(foreign); changed['displayRelays'].append(extra)
    receipt = rpc('PUT', '/v1/instances/foreign-relay', changed)
    require(not receipt['unchanged'] and receipt['changes']['relaysAdded'] == 1 and receipt['changes']['relaysRemoved'] == 0
            and receipt['configuration']['desiredRevision'] == 2 and receipt['relayCount'] == 5, 'changed relay set not applied')
    wait(lambda: all(p.get_num_connections() >= 2 for p in source_pubs), 'a relay change withdrew an unchanged relay')
    receipt = rpc('PUT', '/v1/instances/foreign-relay', foreign)
    require(receipt['changes']['relaysRemoved'] == 1 and receipt['relayCount'] == 4, 'relay set not restored')
    require(rpc('PUT', '/v1/instances/foreign-relay', foreign)['unchanged'], 'retry was not a no-op')
    invalid = instance(1); invalid['robots'][0]['namespace'] = '/bad_slot'
    rpc('PUT', '/v1/instances/invalid-label', invalid, 400)
    rpc('GET', '/v1/instances/invalid-label', expected=404)
    require(rpc('GET', '/v1/instances/foreign-relay')['ready'], 'invalid Scene request stopped foreign instance')
    put_rates(default_rates)

    tf_messages = collections.deque(maxlen=20)
    paths = collections.deque(maxlen=10)
    ar_paths = collections.deque(maxlen=10)
    markers = collections.deque(maxlen=10)
    scenes = collections.deque(maxlen=10)
    standard_tf = collections.deque(maxlen=20)

    def observe(paths_enabled=True):
        subscriptions = [rospy.Subscriber('/xgc/tf', TFMessage, tf_messages.append, queue_size=1),
            rospy.Subscriber('/markers', MarkerArray, markers.append, queue_size=1),
            rospy.Subscriber('/xgc/scene', SceneUpdate, scenes.append, queue_size=1),
            rospy.Subscriber('/tf', TFMessage, standard_tf.append, queue_size=1)]
        if paths_enabled:
            subscriptions += [rospy.Subscriber('/uav1/path', Path, paths.append, queue_size=1),
                rospy.Subscriber('/uav1/ar_path', Path, ar_paths.append, queue_size=1)]
        return subscriptions
    observers = observe()
    pose_pubs = [rospy.Publisher('/uav%d/mavros/local_position/pose' % (i + 1), PoseStamped, queue_size=1) for i in range(100)]
    ar_pub = rospy.Publisher('/vrpn_client_node_physical/uav1/pose', PoseStamped, queue_size=1)
    rpc('PUT', '/v1/instances/robots20', instance(20))
    wait(lambda: all(p.get_num_connections() for p in pose_pubs[:20]), '20 robot inputs missing')
    pump(.7, pose_pubs[:20], ar_pub)
    wait(lambda: bool(paths) and bool(tf_messages), '20 representative pose/path delivery missing')
    twenty = resources(server.pid)
    require(rpc('GET', '/v1/status')['publisherWorkers'] == 1, 'publisher count changed')
    started = time.monotonic(); rpc('DELETE', '/v1/instances/robots20'); delete20 = time.monotonic() - started
    require(delete20 < 2, 'DELETE20 exceeded bounded completion')
    require(rpc('GET', '/v1/instances/foreign-relay')['ready'], 'DELETE removed foreign instance')
    require(rospy.get_param('/foreign/visual_robot_description') == 'foreign-owned', 'global URDF purge occurred')
    for observer in observers:
        observer.unregister()
    retained_topics = set(sources + [spec['topic'] for spec in relay_specs] + ['/rosout', '/clock'])

    def discovery_released():
        _, _, state = xmlrpc.client.ServerProxy(master_uri).getSystemState('/private_probe')
        return not any('/xgc2_ros_visualizer' in nodes and topic not in retained_topics
                       for entries in state[:2] for topic, nodes in entries)
    wait(discovery_released, 'deleted robot publications/subscriptions remained in master registry')
    # Exercise the publication/DELETE fence repeatedly, using the real RPC.
    for i in range(25):
        rpc('PUT', '/v1/instances/delete-race', instance(1))
        clock_pub.publish(Clock(clock=stamp()))
        rpc('DELETE', '/v1/instances/delete-race')
        # ROS master unregistration is asynchronous inside roscpp.
        # Wait for discovery to settle before reusing the same namespace.
        wait(discovery_released, 'deleted resources remained in private master registry')
    hundred_request = instance(100)
    description = robot('described'); description['kind'] = 'description_only'
    description['visualization'].update(sceneClass='', sceneModel='', robotStatePublisher=True,
        descriptionPackage='scout_description', descriptionFile='urdf/scout_visual.urdf')
    hundred_request['robots'].append(description)
    low = {kind: {channel: .1 for channel in channels} for kind, channels in default_rates.items()}
    put_rates(low)
    rpc('PUT', '/v1/instances/robots100', hundred_request)
    wait(lambda: all(p.get_num_connections() for p in pose_pubs), '100 robot inputs missing')
    observers = observe()
    wait(lambda: all(observer.get_num_connections() for observer in observers), '100 robot outputs missing')
    paths.clear(); markers.clear(); scenes.clear(); ar_paths.clear()
    pump(1.35, pose_pubs, ar_pub)
    high = copy.deepcopy(default_rates); high['fs150'].update(path=20, ar_path=20, markers=20, scene_path=20)
    put_rates(high)
    pump(.2, pose_pubs, ar_pub)
    wait(lambda: any(20 <= len(p.poses) <= 61 for p in paths), 'source history was sampled by low publication cadence')
    path_size = max(len(p.poses) for p in paths)
    require(any(len(m.points) >= 20 for a in markers for m in a.markers if m.type == Marker.LINE_STRIP), 'Marker history lost low-rate source samples')
    require(any(len(line.points) >= 20 for update in scenes for entity in update.entities for line in entity.lines), 'Scene history lost low-rate source samples')
    require(any(p.poses and p.poses[-1].pose.orientation.w == 2 and abs(p.poses[-1].pose.position.z - 3.5) < 1e-9 for p in ar_paths), 'AR raw quaternion/offset-once changed')
    hundred = resources(server.pid)
    require(hundred['Threads'] <= twenty['Threads'] + 2, 'OS threads grew with robot count')
    require(hundred['Threads'] <= baseline['Threads'] + 3, 'unexpected application thread growth')
    require(hundred['VmRSS'] - twenty['VmRSS'] < 96 * 1024, 'representative membership memory growth excessive')
    require(rpc('GET', '/v1/status')['callbackWorkers'] == 2, 'input pool grew with robot count')
    xml = rospy.get_param('/described/robot_description')
    names = [j.attrib['name'] for j in ET.fromstring(xml).findall('joint') if j.attrib.get('type') in ('continuous', 'revolute', 'prismatic')]
    require(names, 'URDF fixture has no movable joints')
    joints = rospy.Publisher('/described/joint_states', JointState, queue_size=1)
    wait(lambda: joints.get_num_connections() > 0, 'in-process joint subscriber missing')
    pump(.2, pose_pubs[:1], ar_pub, (joints, names))
    require(any(t.child_frame_id.startswith('described/') for batch in standard_tf for t in batch.transforms), 'in-process URDF joint TF absent')
    rospy.set_param('/described/visual_robot_description', 'foreign-replacement')
    started = time.monotonic(); rpc('DELETE', '/v1/instances/robots100'); delete100 = time.monotonic() - started
    require(delete100 < 2, 'DELETE100 exceeded bounded completion')
    require(rospy.get_param('/described/visual_robot_description') == 'foreign-replacement', 'DELETE erased a replaced foreign parameter')
    require(not rospy.has_param('/described/robot_description'), 'owned URDF parameter leaked')
    rpc('DELETE', '/v1/instances/robots100')
    wait(discovery_released, '100 robot discovery did not settle after DELETE')
    for observer in observers:
        observer.unregister()
    path_only = instance(1); path_only['robots'] = [robot('uav101')]
    path_only['context']['robots'] = [frozen_robot(path_only['robots'][0])]
    path_only['settings']['publication']['paths'] = False
    markers.clear(); scenes.clear()
    rpc('PUT', '/v1/instances/path-only', path_only)
    observers = observe(paths_enabled=False)
    independent = rospy.Publisher('/uav101/mavros/local_position/pose', PoseStamped, queue_size=1)
    wait(lambda: independent.get_num_connections() > 0, 'independent Marker/Scene path input absent')
    pump(.6, [independent])
    wait(lambda: any(m.ns == 'uav101_actual_path' and len(m.points) >= 5
                     for update in markers for m in update.markers), 'Marker path depended on nav Path enable')
    require(any('uav101' in entity.id and len(line.points) >= 5
                for update in scenes for entity in update.entities for line in entity.lines),
            'Scene path depended on nav Path enable')
    _, _, state = xmlrpc.client.ServerProxy(master_uri).getSystemState('/private_probe')
    require(not any(topic == '/uav101/path' for topic, _ in state[0]), 'disabled nav Path was advertised')
    rpc('DELETE', '/v1/instances/path-only')
    wait(discovery_released, 'path-only discovery did not settle after DELETE')
    disabled_ground = instance()
    ground_row = robot('ugv102')
    ground_row['kind'] = 'scout_mini'
    ground_row['visualization'].update(sceneClass='scout', descriptionPackage='scout_description',
        descriptionFile='urdf/scout_visual.urdf')
    disabled_ground['robots'] = [ground_row]
    disabled_ground['settings']['publication']['groundScene'] = False
    ground_source = rospy.Publisher('/ugv102/pose', PoseStamped, queue_size=1)
    status = rpc('PUT', '/v1/instances/ground-disabled', disabled_ground)
    require(status['robotCount'] == 0 and status['descriptionCount'] == 1,
            'track_ugv false did not separate ground scene from descriptions')
    pump(.2, [ground_source], frame='world')
    require(ground_source.get_num_connections() == 0, 'disabled ground scene subscribed to scientific pose')
    require(rospy.has_param('/ugv102/visual_robot_description'), 'ground flag disabled independent description')
    _, _, state = xmlrpc.client.ServerProxy(master_uri).getSystemState('/private_probe')
    require(not any(topic == '/ugv102/path' for topic, _ in state[0]), 'disabled ground scene advertised path')
    rpc('DELETE', '/v1/instances/ground-disabled')
    require(not rospy.has_param('/ugv102/visual_robot_description'), 'independent ground description leaked')
    put_rates(low)
    # Lowest legal rates must not make SIGTERM wait ten seconds.
    started = time.monotonic(); server.send_signal(signal.SIGTERM)
    server.wait(timeout=3)
    stop_seconds = time.monotonic() - started
    require(server.returncode == 0 and stop_seconds < 2, 'low-rate Stop failed')
    require(not os.path.exists(probe.socket), 'owned RPC socket leaked')
    wait(lambda: all(p.get_num_connections() == 1 for p in source_pubs), 'server source subscriptions leaked')
    require(rospy.get_param('/foreign/visual_robot_description') == 'foreign-owned', 'Stop erased foreign parameter')

    # ---- Restart and crash: no state outlives a process -------------------------------------
    # Stale master parameters and retired files cannot restore native membership.
    rospy.set_param('/xgc2_ros_visualizer/initial_instance_file', '/nonexistent/old.json')
    rospy.set_param('/xgc2_ros_visualizer/server_instance_id', 'old-provider')
    previous_incarnation = probe.instance_id
    restarted = probe.start_server(world_clock='simulation', arguments=['/use_sim_time:=/xgc2_ros_visualizer/use_sim_time'])
    require(probe.instance_id != previous_incarnation, 'restart retained old provider incarnation')
    stale = probe.instance_id
    probe.instance_id = previous_incarnation
    rpc('GET', '/v1/status', expected=409)
    probe.instance_id = stale
    restored = rpc('GET', '/v1/rates')
    require(restored['appliedRevision'] == 1 and restored['rates'] == default_rates,
            'ephemeral rate state was falsely persisted')
    require(rpc('GET', '/v1/status')['instanceCount'] == 0, 'restart resurrected old native instances')
    displayed[0].clear()
    crash_viewer = rospy.Subscriber('/xgc/display' + sources[0], rospy.AnyMsg,
        lambda message: displayed[0].append(bytes(message._buff)), queue_size=100)
    rpc('PUT', '/v1/instances/crash-owned', instance(relays=relay_specs[:1], scene=False))
    wait(lambda: source_pubs[0].get_num_connections() >= 2 and crash_viewer.get_num_connections(),
         'crash fixture did not activate its native relay')
    packets(5)
    wait(lambda: bool(displayed[0]), 'native relay carried no bytes before crash')
    require(all(sample in original[0] for sample in displayed[0]), 'pre-crash relay changed source bytes')
    crash_incarnation = probe.instance_id
    restarted.send_signal(signal.SIGKILL); restarted.wait(timeout=3)
    require(restarted.returncode == -signal.SIGKILL, 'crash fixture did not kill its owned provider')
    require(os.path.exists(probe.socket), 'crash fixture did not retain the stale socket')
    wait(lambda: source_pubs[0].get_num_connections() == 1 and not crash_viewer.get_num_connections(),
         'killed native relay retained live data connections')
    recovered = probe.launch(world_clock='simulation', arguments=['/use_sim_time:=/xgc2_ros_visualizer/use_sim_time'])
    probe.instance_id = None

    def recovery_ready():
        require(recovered.poll() is None, 'crash replacement exited')
        try:
            probe.instance_id = rpc('GET', '/v1/describe', bound=False)['instance_id']
            return True
        except (OSError, ConnectionError):
            return False
    wait(recovery_ready, 'crash replacement did not acquire the stale endpoint')
    require(probe.instance_id != crash_incarnation, 'crash replacement reused provider incarnation')
    require(rpc('GET', '/v1/status')['instanceCount'] == 0, 'crash recovery resurrected native membership')
    restored = rpc('GET', '/v1/rates')
    require(restored['appliedRevision'] == 1 and restored['rates'] == default_rates,
            'crash recovery imported ephemeral rate state')
    require(source_pubs[0].get_num_connections() == 1 and not crash_viewer.get_num_connections(),
            'replacement silently reactivated the old native relay')
    displayed[0].clear()
    rpc('PUT', '/v1/instances/recovered-relay', instance(relays=relay_specs[:1], scene=False))
    wait(lambda: source_pubs[0].get_num_connections() >= 2 and crash_viewer.get_num_connections(),
         'replacement failed explicit native relay activation')
    time.sleep(.1)
    require(not displayed[0], 'replacement replayed a source sample from the killed provider')
    packets(5)
    wait(lambda: bool(displayed[0]), 'replacement native relay did not deliver fresh bytes')
    require(all(sample in original[0] for sample in displayed[0]), 'replacement relay changed fresh bytes')
    rpc('DELETE', '/v1/instances/recovered-relay')
    wait(lambda: source_pubs[0].get_num_connections() == 1 and not crash_viewer.get_num_connections(),
         'replacement DELETE did not fence its native relay')
    crash_viewer.unregister()
    recovered.send_signal(signal.SIGTERM)
    recovered.wait(timeout=3)
    require(recovered.returncode == 0 and not os.path.exists(probe.socket),
            'crash replacement failed native operation or endpoint cleanup')
    print(json.dumps(dict(ok=True, scope='private ROS graph; not station/scientific acceptance',
        relay_received=relay_counts, robots20=twenty, robots100=hundred,
        source_history_points=path_size, simulated_rtf=3, delete20_seconds=delete20,
        delete100_seconds=delete100, low_rate_stop_seconds=stop_seconds,
        zero_robot_readiness=True, explicit_activation=True,
        retired_wire_envelopes_rejected=True, fixed_callback_workers=2, publisher_workers=1,
        restart_incarnation_fenced=True, rate_cas_conflict=True, ephemeral_restart=True,
        sigkill_stale_endpoint_recovered=True, crash_incarnation_fenced=True,
        sigkill_native_relay_fenced=True, crash_relay_no_replay=True,
        no_child_processes=True), sort_keys=True))
    rospy.signal_shutdown('private probe complete')


if __name__ == '__main__':
    require(len(sys.argv) == 2, 'expected the actual server executable')
    main(os.path.abspath(sys.argv[1]))
