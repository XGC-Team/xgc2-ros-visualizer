#!/usr/bin/env python3
"""Private ROS check for the display-copy ceiling.

Singular Marker counts received messages, not publish cycles. A 5 Hz ceiling
that burst every coalesced id into a queue of one would fail the distinct-id
and DELETE-then-ADD checks. The probe starts its own master.
"""
import os
import sys
import time

from probe_support import Probe, instance, require, robot, wait


def threads(pid):
    return len(os.listdir('/proc/%d/task' % pid))


def children(pid):
    found = []
    for tid in os.listdir('/proc/%d/task' % pid):
        with open('/proc/%d/task/%s/children' % (pid, tid)) as stream:
            found.extend(stream.read().split())
    return found


def relay(source, message_type, max_rate=None, role=None):
    row = dict(source=source, topic='/xgc/display' + source, messageType=message_type)
    if max_rate is not None:
        row['maxRateHz'] = max_rate
    if role is not None:
        row['role'] = role
    return row


def main(binary):
    import rospy
    from visualization_msgs.msg import Marker, MarkerArray

    probe = Probe(binary)
    try:
        probe.start_master()
        server = probe.start_server(world_clock='wall')
        rpc = probe.rpc
        rospy.init_node('marker_rate_probe', anonymous=True, disable_signals=True)
        source = '/probe/marks'
        displayed = []
        originals = []

        def on_display(message):
            displayed.append((time.monotonic(), int(message.id), int(message.action), message.lifetime.to_sec(), message.ns))

        def on_source(message):
            originals.append(message)

        pub = rospy.Publisher(source, Marker, queue_size=50)
        rospy.Subscriber(source, Marker, on_source, queue_size=1000)
        sub = rospy.Subscriber('/xgc/display' + source, Marker, on_display, queue_size=1000)
        body = instance(relays=[relay(source, 'visualization_msgs/Marker', 5)])
        rpc('PUT', '/v1/instances/marks', body)
        wait(lambda: pub.get_num_connections() >= 2 and sub.get_num_connections() >= 1, 'marker relay did not connect')

        def publish_marker(marker_id, action=Marker.ADD, lifetime=0.0, stamp=None, ns='probe'):
            message = Marker()
            message.header.stamp = stamp or rospy.Time.now()
            message.header.frame_id = 'world'
            message.ns = ns
            message.id = marker_id
            message.action = action
            message.lifetime = rospy.Duration(lifetime)
            message.pose.orientation.w = 1
            message.scale.x = message.scale.y = message.scale.z = 0.1
            pub.publish(message)

        def until_copy(publish, message):
            displayed[:] = []

            def seen():
                if displayed:
                    return True
                publish()
                return False

            wait(seen, message)

        def pace(seconds, marker_id):
            displayed[:] = []
            originals[:] = []
            started = time.monotonic()
            next_tick = started
            while time.monotonic() - started < seconds:
                publish_marker(marker_id)
                next_tick += 0.02
                time.sleep(max(0.0, next_tick - time.monotonic()))
            time.sleep(0.15)
            return len(displayed), len(originals)

        # One warm publish so the following window is paced by the ceiling.
        publish_marker(1)
        wait(lambda: any(item[1] == 1 for item in displayed), 'first marker copy missing')
        rate5, source5 = pace(2.5, 1)
        require(8 <= rate5 <= 16, '5 Hz ceiling was not a message rate: %d' % rate5)
        require(source5 >= 80, 'source was slowed: %d' % source5)

        body['displayRelays'] = [relay(source, 'visualization_msgs/Marker', 2)]
        rpc('PUT', '/v1/instances/marks', body)
        until_copy(lambda: publish_marker(1), '2 Hz relay produced no copy')
        rate2, source2 = pace(2.5, 1)
        require(3 <= rate2 <= 8, '2 Hz ceiling was not a message rate: %d' % rate2)
        require(rate5 > rate2, '5 Hz did not deliver more Marker messages than 2 Hz')
        require(source2 >= 80, 'source was slowed at 2 Hz: %d' % source2)

        body['displayRelays'] = [relay(source, 'visualization_msgs/Marker', 100)]
        rpc('PUT', '/v1/instances/marks', body)
        until_copy(lambda: publish_marker(1), 'uncapped relay produced no copy')
        rate10, source10 = pace(2.5, 1)
        require(18 <= rate10 <= 32, 'maxRate above the channel raised or dropped the copy: %d' % rate10)
        require(rate10 > rate5, 'channel default did not stay above the 5 Hz ceiling')
        require(source10 >= 80 and rate10 < source10 / 2, 'copy followed the source instead of the channel')

        body['displayRelays'] = [relay(source, 'visualization_msgs/Marker', 5)]
        rpc('PUT', '/v1/instances/marks', body)
        # A rebuilt subscriber drops anything published before its TCP connection exists.
        # Repeat a throwaway namespace until one copy comes back, then count the burst.
        displayed[:] = []

        def warm_connected():
            if any(item[1] == 0 and item[4] == 'warm' for item in displayed):
                return True
            publish_marker(0, ns='warm')
            return False

        wait(warm_connected, 'id drain relay produced no copy')
        burst_at = time.monotonic()
        for marker_id in range(1, 7):
            publish_marker(marker_id)

        def drained():
            return [item for item in displayed if item[0] >= burst_at and item[4] == 'probe']

        wait(lambda: len(drained()) >= 6, 'distinct marker ids were dropped', seconds=3)
        got = drained()
        ids = [item[1] for item in got[:6]]
        require(ids == [1, 2, 3, 4, 5, 6], 'drain lost ids: %s' % ids)
        require(got[5][0] - got[0][0] >= 0.7, 'ids arrived as one burst, not one message per period')

        displayed[:] = []
        publish_marker(1)
        wait(lambda: any(item[1] == 1 and item[2] == Marker.ADD for item in displayed), 'delete setup add missing')
        displayed[:] = []
        publish_marker(1, action=Marker.DELETE)
        publish_marker(2, action=Marker.ADD)
        wait(lambda: any(item[2] == Marker.DELETE and item[1] == 1 for item in displayed)
             and any(item[2] == Marker.ADD and item[1] == 2 for item in displayed),
             'DELETE then ADD was not both delivered', seconds=2)
        require(not any(item[2] == Marker.ADD and item[1] == 1 for item in displayed),
                'deleted id was added again')

        body['displayRelays'] = [relay(source, 'visualization_msgs/Marker', 2)]
        rpc('PUT', '/v1/instances/marks', body)
        displayed[:] = []

        def saw_immortal():
            if any(item[1] == 7 for item in displayed):
                return True
            publish_marker(7, lifetime=0.0)
            return False

        wait(saw_immortal, 'lifetime warmup missing')
        displayed[:] = []
        publish_marker(99, lifetime=0.05)
        time.sleep(1.0)
        late = [item for item in displayed if item[1] == 99 and item[2] == Marker.ADD]
        require(not late, 'short lifetime was forwarded after it should have died: %s' % late)

        body = instance(relays=[relay(source, 'visualization_msgs/Marker', role='prediction')],
                        settings=dict(predictionMaxRateHz=5))
        rpc('PUT', '/v1/instances/marks', body)
        until_copy(lambda: publish_marker(1), 'prediction relay produced no copy')
        pred5, _ = pace(2.5, 1)
        require(8 <= pred5 <= 16, 'prediction cap was not 5 Hz of messages: %d' % pred5)
        tuned = instance(relays=[relay(source, 'visualization_msgs/Marker', role='prediction')],
                         settings=dict(predictionMaxRateHz=2))
        receipt = rpc('PUT', '/v1/instances/marks', tuned)
        require(receipt['changes']['relaysAdded'] == 0 and receipt['changes']['relaysRemoved'] == 0,
                'prediction ceiling rebuilt the relay')
        pred2, _ = pace(2.5, 1)
        require(3 <= pred2 <= 8, 'lowered prediction cap was not 2 Hz of messages: %d' % pred2)
        require(pred5 > pred2, 'prediction cap change did not reduce Marker messages')

        obstacle = instance(relays=[relay(source, 'visualization_msgs/Marker', 8, 'obstacle')],
                            settings=dict(predictionMaxRateHz=2))
        rpc('PUT', '/v1/instances/marks', obstacle)
        until_copy(lambda: publish_marker(1), 'obstacle relay produced no copy')
        obstacle_rate, _ = pace(2.5, 1)
        require(16 <= obstacle_rate <= 26, 'obstacle followed the prediction cap: %d' % obstacle_rate)

        batch_source = '/probe/batch'
        batches = []
        batch_pub = rospy.Publisher(batch_source, MarkerArray, queue_size=50)
        rospy.Subscriber('/xgc/display' + batch_source, MarkerArray,
                         lambda message: batches.append(message), queue_size=200)
        both = instance(relays=[relay(source, 'visualization_msgs/Marker', 5),
                                relay(batch_source, 'visualization_msgs/MarkerArray', 5)])
        rpc('PUT', '/v1/instances/marks', both)

        def batch_marker(marker_id):
            message = Marker()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'world'
            message.ns = 'batch'
            message.id = marker_id
            message.action = Marker.ADD
            message.pose.orientation.w = 1
            return message

        def batch_connected():
            if batches:
                return True
            array = MarkerArray()
            array.markers = [batch_marker(1), batch_marker(2)]
            batch_pub.publish(array)
            return False

        wait(batch_connected, 'marker array relay did not connect')
        batches[:] = []
        started = time.monotonic()
        next_tick = started
        while time.monotonic() - started < 2.0:
            array = MarkerArray()
            array.markers = [batch_marker(1), batch_marker(2)]
            batch_pub.publish(array)
            next_tick += 0.02
            time.sleep(max(0.0, next_tick - time.monotonic()))
        time.sleep(0.3)
        require(8 <= len(batches) <= 16, 'MarkerArray message rate left the 5 Hz ceiling: %d' % len(batches))
        require(any(sorted(marker.id for marker in message.markers) == [1, 2] for message in batches),
                'MarkerArray batch dropped one of the ids')

        before = threads(server.pid)
        many = [relay('/probe/extra%d' % i, 'visualization_msgs/Marker', 5) for i in range(16)]
        rpc('PUT', '/v1/instances/marks', instance(relays=many))
        time.sleep(0.4)
        require(rpc('GET', '/v1/status')['publisherWorkers'] == 1, 'publisher workers changed')
        require(not children(server.pid), 'relay spawn forked a child')
        require(threads(server.pid) - before < 8, 'relays added a thread each')

        # History LINE_STRIP on /markers follows historyMaxRateHz. Meshes stay
        # on the markers channel. One scheduler thread, same process.
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as NavPath
        history_rows = []
        path_lengths = []
        rospy.Subscriber('/markers', MarkerArray, lambda message: history_rows.append((
            time.monotonic(),
            sum(1 for marker in message.markers
                if marker.ns == 'uav1_actual_path' and marker.type == Marker.LINE_STRIP
                and marker.action == Marker.ADD),
            sum(1 for marker in message.markers
                if marker.ns == 'uav1_actual_path' and marker.type == Marker.LINE_STRIP
                and marker.action == Marker.DELETE),
            any(marker.ns != 'uav1_actual_path' or marker.type != Marker.LINE_STRIP
                for marker in message.markers),
            max([len(marker.points) for marker in message.markers
                 if marker.ns == 'uav1_actual_path' and marker.type == Marker.LINE_STRIP
                 and marker.action == Marker.ADD] or [0]))), queue_size=500)
        rospy.Subscriber('/uav1/path', NavPath,
                         lambda message: path_lengths.append(len(message.poses)), queue_size=20)
        pose_pub = rospy.Publisher('/uav1/mavros/local_position/pose', PoseStamped, queue_size=1)

        def history_body(rate, visible):
            return instance(robots=[robot('uav1')], settings=dict(
                historyMaxRateHz=rate, historyVisible=visible,
                publication=dict(markers=True, transforms=True, scene=True,
                                 scenePaths=True, paths=True)))

        def pump_pose():
            message = PoseStamped()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'map'
            message.pose.position.x = time.monotonic()
            message.pose.orientation.w = 1
            pose_pub.publish(message)

        rpc('DELETE', '/v1/instances/marks')
        rpc('PUT', '/v1/instances/history', history_body(5, True))
        wait(lambda: pose_pub.get_num_connections() > 0, 'history pose subscriber missing')

        def line_ready():
            pump_pose()
            return any(row[1] and row[4] >= 2 for row in history_rows)

        wait(line_ready, 'history line was not published', seconds=4)

        def pace_history(seconds):
            history_rows[:] = []
            started = time.monotonic()
            next_tick = started
            while time.monotonic() - started < seconds:
                pump_pose()
                next_tick += 0.02
                time.sleep(max(0.0, next_tick - time.monotonic()))
            time.sleep(0.15)
            return time.monotonic()  # rows carry their own stamps; the clear is the window

        def tally():
            lines = sum(1 for row in history_rows if row[1])
            deletes = sum(row[2] for row in history_rows)
            others = sum(1 for row in history_rows if row[3])
            points = max([row[4] for row in history_rows] or [0])
            return lines, deletes, others, points

        pace_history(2.5)
        line5, _, mesh5, points5 = tally()
        require(8 <= line5 <= 16, 'history line was not 5 Hz: %d' % line5)
        require(mesh5 >= 40, 'non-history markers were capped with the line: %d' % mesh5)
        require(mesh5 > line5 * 2, 'history line repeated with the markers channel: %d vs %d' % (line5, mesh5))
        ring = max(path_lengths or [0])
        require(20 <= points5 <= 61 and 20 <= ring <= 61,
                'source ring was coarsened: line %d path %d' % (points5, ring))
        tuned = rpc('PUT', '/v1/instances/history', history_body(2, True))
        require(tuned['changes']['settings'] and tuned['changes']['rebuilt'] == []
                and tuned['changes']['relaysAdded'] == 0,
                'history rate rebuilt the robot or a relay')
        pace_history(2.5)
        line2, _, mesh2, points2 = tally()
        require(3 <= line2 <= 8, 'history line was not 2 Hz: %d' % line2)
        require(line5 > line2, 'history 5 Hz did not deliver more line ADDs than 2 Hz')
        require(mesh2 >= 40, 'non-history markers followed the history cap: %d' % mesh2)
        require(20 <= points2 <= 61, 'source ring changed at 2 Hz: %d' % points2)
        history_rows[:] = []
        path_lengths[:] = []
        rpc('PUT', '/v1/instances/history', history_body(2, False))
        started = time.monotonic()
        next_tick = started
        while time.monotonic() - started < 1.0:
            pump_pose()
            next_tick += 0.02
            time.sleep(max(0.0, next_tick - time.monotonic()))
        time.sleep(0.15)
        hidden_lines, hidden_deletes, hidden_mesh, _ = tally()
        require(hidden_deletes == 1, 'history hide did not delete the line once: %d' % hidden_deletes)
        require(hidden_lines == 0, 'history hide kept publishing the line: %d' % hidden_lines)
        require(hidden_mesh >= 15, 'hide capped non-history markers: %d' % hidden_mesh)
        require(any(length == 0 for length in path_lengths), 'hide did not clear the latched path')
        require(rpc('GET', '/v1/status')['publisherWorkers'] == 1, 'history pacing added a publisher thread')
        require(not children(server.pid), 'history pacing forked a child')
        print('marker-rate-probe %s' % dict(
            marker_5hz=rate5, marker_2hz=rate2, marker_channel=rate10,
            source_5hz=source5, prediction_5hz=pred5, prediction_2hz=pred2,
            obstacle_8hz=obstacle_rate, marker_array_5hz=len(batches),
            history_line_5hz=line5, history_line_2hz=line2,
            history_mesh_5hz=mesh5, history_mesh_2hz=mesh2,
            history_ring_points=ring, history_hide_deletes=hidden_deletes))
    finally:
        probe.close()


if __name__ == '__main__':
    require(len(sys.argv) == 2, 'expected the server executable')
    main(os.path.abspath(sys.argv[1]))
