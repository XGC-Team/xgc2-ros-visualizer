"""Shared fixture of the private ROS graph probes.

A probe starts its own ROS master on a free loopback port, runs the actual server
executable against it and talks to the control socket like a Core client does.
Never uses a station master.
"""
import http.client
import json
import os
import random
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid
import xmlrpc.client


class UnixHTTP(http.client.HTTPConnection):
    def __init__(self, path):
        super().__init__("localhost", timeout=5)
        self.path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


def require(value, message):
    if not value:
        raise AssertionError(message)


def wait(predicate, message, seconds=6):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError(message)


def load_definition(path):
    """The one process definition a product ships, read as its supervisor reads it."""
    with open(path) as stream:
        catalog = json.load(stream)
    require(set(catalog) == {'apiVersion', 'definitions'}, 'unexpected definition envelope field')
    require(catalog['apiVersion'] == 'xgc.execution.process/v1', 'unexpected definition schema')
    require(len(catalog['definitions']) == 1, 'a product file holds the definitions it ships')
    definition = catalog['definitions'][0]
    for location, fields, allowed in (
            ('definition', definition, {'id', 'version', 'label', 'description', 'parameters',
                'setupFiles', 'command', 'services', 'readiness', 'stop', 'logs'}),
            ('parameters', definition['parameters'], {'properties', 'required', 'groups'}),
            ('command', definition['command'], {'executable', 'args', 'workDir', 'env', 'stdinParameter'}),
            ('readiness', definition['readiness'], {'kind', 'startGraceMs', 'timeoutMs', 'address', 'masterUri'}),
            ('stop', definition['stop'], {'graceMs', 'rpc'}),
            ('logs', definition['logs'], {'maxBytes', 'files'})):
        require(set(fields) <= allowed, location + ' has unsupported fields: ' + str(sorted(set(fields) - allowed)))
    for name, parameter in definition['parameters']['properties'].items():
        require(set(parameter) <= {'type', 'description', 'default', 'enum', 'minimum', 'maximum', 'sensitive',
                'output', 'fixedOnly', 'x-xgc-path-kind', 'x-xgc-file-extensions'},
                'unsupported fields in parameter ' + name)
    for group in definition['parameters'].get('groups', []):
        require(set(group) <= {'id', 'label', 'collapsed', 'parameters'}, 'unsupported parameter group field')
    for service in definition['services']:
        require(set(service) == {'service', 'api_version', 'profile', 'endpointParameter'},
                'unexpected service fields')
    require(definition['stop'] == {'graceMs': 5000}, 'visualizer stop budget must be 5000 ms')
    return definition


def render(definition, binary, allocations):
    """argv and the environment overlay the definition asks for, parameters from defaults and allocations.

    Only the templated variables are overlaid: the fixed ROS variables describe the
    image the definition is installed in, not the machine a probe may run on.
    """
    command = definition['command']
    require(os.path.basename(command['executable']) == os.path.basename(binary),
            'definition names another executable than the probe runs')
    properties = definition['parameters']['properties']
    parameters = {name: spec['default'] for name, spec in properties.items() if 'default' in spec}
    parameters.update({name: value for name, value in allocations.items() if name in properties})
    require(all(name in parameters for name in definition['parameters']['required']),
            'definition requires an input the owner did not supply')

    def value(text):
        def replace(match):
            require(match.group(1) in parameters, 'unresolved definition parameter: ' + match.group(1))
            return str(parameters[match.group(1)])
        rendered = re.sub(r'\$\{([^{}]+)\}', replace, text)
        require('${' not in rendered, 'unresolved definition template')
        return rendered
    argv = [binary] + [value(argument) for argument in command['args']]
    overlay = {name: value(entry) for name, entry in command['env'].items() if '${' in entry}
    return argv, overlay


def robot(name, kind='px4_multirotor', scene_class='fs150', package='fs150_description',
          urdf='urdf/fs150_visual.urdf'):
    """A projected display Robot row: exactly the five fields the display workflow publishes."""
    return dict(slotId=name, namespace='/' + name, kind=kind,
                visualization=dict(sceneClass=scene_class, sceneModel=name,
                    descriptionPackage=package, descriptionFile=urdf,
                    robotStatePublisher=False, jointStateTopic='joint_states', pathTopic='path'),
                hybridSource='simulation')


def scout(name):
    return robot(name, kind='scout_mini', scene_class='scout', package='scout_description',
                 urdf='urdf/scout_visual.urdf')


def frozen_robot(robot_row, source='simulation', profile_id='px4-multirotor.xsim.ros',
                 localization=None):
    row = dict(slotId=robot_row['slotId'], namespace=robot_row['namespace'], source=source,
               profile=dict(id=profile_id, parameters={}))
    if localization:
        row['profile']['parameters']['localization_pose_topic'] = localization
    return row


def instance(robots=(), relays=None, publication=None, settings=None):
    panel = dict(settings or {})
    if publication is not None:
        panel['publication'] = publication
    return dict(robots=list(robots),
                context=dict(runMode='simulation', worldBoundary=None, scene=dict(simulator='xsim'),
                             localizationOffset=dict(x=1, y=2, z=3),
                             robots=[frozen_robot(item, localization='/raw/' + item['slotId'])
                                     for item in robots]),
                settings=panel, displayRelays=relays or [])


class Probe:
    """A private master, one server executable and its control socket."""

    def __init__(self, binary, prefix='xgc2-visualizer-probe-'):
        self.binary = binary
        self._work = tempfile.TemporaryDirectory(prefix=prefix)
        self.work = self._work.name
        os.chmod(self.work, 0o700)
        self.socket = os.path.join(self.work, 'control.sock')
        reserve = socket.socket()
        try:
            for port in random.sample(range(20000, 40001), 100):
                try:
                    reserve.bind(('127.0.0.1', port))
                    break
                except OSError:
                    continue
            else:
                raise RuntimeError('no private ROS master port available in 20000-40000')
            self.port = reserve.getsockname()[1]
        finally:
            reserve.close()
        self.master_uri = 'http://127.0.0.1:%d' % self.port
        self.log_directory = os.path.join(self.work, 'roslog')
        os.makedirs(self.log_directory, mode=0o700)
        os.environ.update(ROS_MASTER_URI=self.master_uri, ROS_IP='127.0.0.1', ROS_HOME=self.work,
                          ROS_LOG_DIR=self.log_directory)
        os.environ.pop('ROS_HOSTNAME', None)
        self.logfile = open(os.path.join(self.work, 'process.log'), 'w')
        self.processes = []
        self.instance_id = None
        self.server = None

    def start_master(self):
        master = subprocess.Popen(['roscore', '-p', str(self.port)], stdout=self.logfile,
                                  stderr=self.logfile, start_new_session=True)
        self.processes.append(master)
        self.master = master

        def ready():
            try:
                return xmlrpc.client.ServerProxy(self.master_uri).getPid('/probe')[0] == 1
            except (OSError, xmlrpc.client.Error):
                return False
        wait(ready, 'private ROS master failed')
        return master

    def kill_master(self):
        os.killpg(self.master.pid, signal.SIGTERM)
        self.master.wait(timeout=10)

    def launch(self, arguments=None, world_clock='simulation', environment=None):
        argv = [self.binary, '--socket-path', self.socket, '--world-clock', world_clock]
        argv += arguments or []
        self.server = subprocess.Popen(argv, env=environment, stdout=self.logfile, stderr=self.logfile,
                                       start_new_session=True)
        self.processes.append(self.server)
        return self.server

    def start_from_definition(self, definition, world_clock='simulation'):
        """Start the server exactly as the shipped definition says, and check what it declares."""
        argv, overlay = render(definition, self.binary, dict(
            socketPath=self.socket, rosHome=self.work, rosLogDir=self.log_directory,
            rosMasterUri=self.master_uri, rosIp='127.0.0.1', worldClock=world_clock))
        environment = dict(os.environ)
        environment.update(overlay)
        self.server = subprocess.Popen(argv, env=environment, stdout=self.logfile, stderr=self.logfile,
                                       start_new_session=True)
        self.processes.append(self.server)
        self.await_socket()
        self.instance_id = None
        described = self.rpc('GET', '/v1/describe')
        self.instance_id = described['instance_id']
        service, = definition['services']
        endpoint = definition['parameters']['properties'][service['endpointParameter']]
        require(service['service'] == described['service'] and service['api_version'] == described['api_version']
                and service['profile'] == 'http.v1' and endpoint['type'] == 'string' and endpoint['fixedOnly'],
                'the definition declares another service than the process hosts')
        readiness = definition['readiness']
        require(readiness['kind'] == 'describe' and 0 < readiness['startGraceMs'] <= readiness['timeoutMs'] <= 60000,
                'readiness must be the describe contract with a finite budget')
        require(self.rpc('GET', '/v1/describe?wait_ready_ms=%d' % min(readiness['timeoutMs'], 30000), bound=False)['ready'],
                'the process is not ready under its own definition')
        return self.server

    def await_socket(self):
        wait(lambda: os.path.exists(self.socket) or self.server.poll() is not None, 'server socket absent')
        require(self.server.poll() is None, 'server exited during native startup')

    def start_server(self, **options):
        server = self.launch(**options)
        self.await_socket()
        self.instance_id = None
        self.instance_id = self.rpc('GET', '/v1/describe')['instance_id']
        return server

    def rpc(self, method, path, body=None, expected=200, bound=True):
        headers = {'Content-Type': 'application/json', 'X-Request-ID': uuid.uuid4().hex,
                   'X-Xrpc-Timeout-Ms': '5000'}
        if self.instance_id is not None and bound:
            headers['X-Xrpc-Instance-ID'] = self.instance_id
        client = UnixHTTP(self.socket)
        try:
            client.request(method, path, None if body is None else json.dumps(body), headers)
            response = client.getresponse()
            status = response.status
            require(response.getheader('X-Request-ID') == headers['X-Request-ID'], 'response request identity lost')
            if self.instance_id is not None and status != 409:
                require(response.getheader('X-Xrpc-Instance-ID') == self.instance_id, 'response instance identity lost')
            result = json.loads(response.read(1024 * 1024 + 1))
        finally:
            client.close()
        require(status == expected, '%s %s: %s %s' % (method, path, status, result))
        return result

    def registry(self, node='/xgc2_ros_visualizer'):
        """Topics this node publishes and subscribes to, from the master's own books."""
        _, _, state = xmlrpc.client.ServerProxy(self.master_uri).getSystemState('/probe')
        publishers = {topic for topic, nodes in state[0] if node in nodes}
        subscribers = {topic for topic, nodes in state[1] if node in nodes}
        return publishers, subscribers

    def log_tail(self, characters=5000):
        self.logfile.flush()
        with open(self.logfile.name) as stream:
            return stream.read()[-characters:]

    def stop(self, signal_number=signal.SIGTERM, timeout=5):
        """Stop the server like its supervisor does and return the exit code."""
        self.server.send_signal(signal_number)
        return self.server.wait(timeout=timeout)

    def close(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
        self.logfile.close()
        self._work.cleanup()
