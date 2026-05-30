"""Targeted tests for ``navigation2.py`` action-server behavior."""

from types import SimpleNamespace

from rclpy.action import CancelResponse

from topological_navigation.navigation_graph import ActionSegment, NavState
from topological_navigation.scripts.navigation2 import (
    TopologicalNavServer,
    _make_ros_param_value,
    _ros_param_value,
)


class _DummyLogger:
    """Minimal logger stub for unit tests."""

    def debug(self, _msg):
        pass

    def info(self, _msg):
        pass

    def warning(self, _msg):
        pass

    def error(self, _msg):
        pass


class _DummyStateMachine:
    """Small state-machine stub used by action callback tests."""

    def __init__(self, terminal=False):
        self.terminal = terminal
        self.reset_called = False
        self.state = None
        self.transitions = []

    def transition(self, state):
        self.transitions.append(state)
        self.state = state
        return True

    def is_terminal(self):
        return self.terminal

    def reset(self):
        self.reset_called = True
        return True


class _FakeGotoGoalHandle:
    """Simple goal handle double for ``GotoNode`` callbacks."""

    def __init__(self, target='WP2', no_orientation=False):
        self.request = SimpleNamespace(
            target=target,
            no_orientation=no_orientation,
        )
        self.feedback = []
        self.final_state = None

    def publish_feedback(self, msg):
        self.feedback.append(msg)

    def succeed(self):
        self.final_state = 'succeeded'

    def abort(self):
        self.final_state = 'aborted'

    def canceled(self):
        self.final_state = 'canceled'


class _FakePolicyGoalHandle:
    """Simple goal handle double for ``ExecutePolicyMode`` callbacks."""

    def __init__(self, route):
        self.request = SimpleNamespace(route=route)
        self.feedback = []
        self.final_state = None

    def publish_feedback(self, msg):
        self.feedback.append(msg)

    def succeed(self):
        self.final_state = 'succeeded'

    def abort(self):
        self.final_state = 'aborted'

    def canceled(self):
        self.final_state = 'canceled'


def _make_server():
    """Create a partially constructed ``TopologicalNavServer`` for unit tests."""
    server = TopologicalNavServer.__new__(TopologicalNavServer)
    server.get_logger = lambda: _DummyLogger()
    server._sm = _DummyStateMachine()
    server._navigation_activated = False
    server._cancelled = False
    server._preempted = False
    server._no_orientation = False
    server._target = 'none'
    server._current_target = 'none'
    server._current_node = 'WP1'
    server._closest_node = 'WP1'
    server._tmap = {
        'nodes': [
            {
                'node': {
                    'name': 'WP1',
                    'edges': [
                        {'edge_id': 'WP1_WP2', 'node': 'WP2'},
                    ],
                },
            },
        ],
    }
    server._graph = {}
    server._topol_map = 'test_map'
    server._map_actions = {}
    server._map_definitions = {}
    server._bt_files = {}
    server._action_clients = {}
    server._xy_tolerance_param = 'goal_checker.xy_goal_tolerance'
    server._yaw_tolerance_param = 'goal_checker.yaw_goal_tolerance'
    server._publish_status = lambda *_args, **_kwargs: None
    server._publish_route = lambda *_args, **_kwargs: None
    server._execute_route = lambda *_args, **_kwargs: True
    server._cancel_nav2_goal = lambda *_args, **_kwargs: None
    server._navigate = lambda *_args, **_kwargs: True
    return server


def test_cancel_goto_callback_accepts_cancel():
    """Cancel requests should be accepted and forwarded to Nav2."""
    server = _make_server()
    calls = []
    server._cancel_nav2_goal = lambda *_args, **_kwargs: calls.append('cancel')

    result = server._cancel_goto_cb(None)

    assert result == CancelResponse.ACCEPT
    assert server._cancelled is True
    assert calls == ['cancel']


def test_exec_goto_success_publishes_feedback_and_succeeds():
    """Successful ``GotoNode`` execution should publish feedback and succeed."""
    server = _make_server()
    goal_handle = _FakeGotoGoalHandle(target='WP2', no_orientation=True)

    result = server._exec_goto_cb(goal_handle)

    assert result.success is True
    assert goal_handle.final_state == 'succeeded'
    assert goal_handle.feedback
    assert goal_handle.feedback[0].route == 'Planning...'
    assert goal_handle.feedback[0].__class__.__name__.endswith('Feedback')


def test_exec_goto_cancelled_marks_goal_canceled():
    """Cancelled ``GotoNode`` executions should not be reported as aborted."""
    server = _make_server()
    goal_handle = _FakeGotoGoalHandle(target='WP2')

    def _cancelled_nav(_target):
        server._cancelled = True
        return False

    server._navigate = _cancelled_nav

    result = server._exec_goto_cb(goal_handle)

    assert result.success is False
    assert goal_handle.final_state == 'canceled'


def test_exec_policy_invalid_route_aborts_goal():
    """Invalid policy routes should abort the action instead of succeeding false."""
    server = _make_server()
    route = SimpleNamespace(source=['WP1', 'WP2'], edge_id=['edge_only_once'])
    goal_handle = _FakePolicyGoalHandle(route)

    result = server._exec_policy_cb(goal_handle)

    assert result.success is False
    assert goal_handle.final_state == 'aborted'


def test_exec_policy_success_enters_planning_and_sets_target():
    """Valid policy execution should enter PLANNING before running the route."""
    server = _make_server()
    route = SimpleNamespace(source=['WP1'], edge_id=['WP1_WP2'])
    goal_handle = _FakePolicyGoalHandle(route)
    observed = {}

    def _execute_route(route_nodes, target):
        observed['route_nodes'] = route_nodes
        observed['target'] = target
        observed['server_target'] = server._target
        return True

    server._execute_route = _execute_route

    result = server._exec_policy_cb(goal_handle)

    assert result.success is True
    assert goal_handle.final_state == 'succeeded'
    assert observed['route_nodes'] == ['WP1', 'WP2']
    assert observed['target'] == 'WP2'
    assert observed['server_target'] == 'WP2'
    assert NavState.PLANNING in server._sm.transitions


def test_load_map_config_clears_stale_action_clients_when_actions_missing():
    """Maps without ``actions`` should not retain stale clients from older maps."""
    server = _make_server()
    server._tmap = {}
    server._action_clients = {'stale': object()}

    server._load_map_config()

    assert server._action_clients == {}
    assert server._bt_files == {}


# =====================================================================
# ROS 2 parameter value helpers
# =====================================================================


class _FakeGraph:
    """Minimal networkx-compatible graph stub for unit tests."""

    def __init__(self, node_data):
        self.nodes = node_data

    def __contains__(self, item):
        return item in self.nodes


class TestRosParamHelpers:
    """Unit tests for _ros_param_value and _make_ros_param_value."""

    def _pv(self, value):
        """Round-trip: build ParameterValue then read it back."""
        from rcl_interfaces.msg import ParameterValue, ParameterType
        pv = _make_ros_param_value(value)
        return _ros_param_value(pv), pv

    def test_double_roundtrip(self):
        val, pv = self._pv(3.14)
        assert abs(val - 3.14) < 1e-9

    def test_int_roundtrip(self):
        val, pv = self._pv(42)
        assert val == 42

    def test_bool_roundtrip(self):
        val, pv = self._pv(True)
        assert val is True

    def test_string_roundtrip(self):
        val, pv = self._pv('hello')
        assert val == 'hello'

    def test_unsupported_type_returns_none(self):
        from rcl_interfaces.msg import ParameterValue
        # An unset ParameterValue has type 0 (PARAMETER_NOT_SET)
        pv = ParameterValue()
        assert _ros_param_value(pv) is None

    def test_make_unsupported_type_returns_unset(self):
        from rcl_interfaces.msg import ParameterType
        pv = _make_ros_param_value([1, 2, 3])  # list not supported
        assert pv.type == ParameterType.PARAMETER_NOT_SET


# =====================================================================
# _apply_segment_parameters / _restore_segment_parameters
# =====================================================================


class TestApplyRestoreSegmentParameters:
    """Parameter save/restore behaviour in isolation from Nav2."""

    def _make_segment(self, target='WP2', edge_props=None, action='navigate_to_pose'):
        """Build an ActionSegment stub."""
        props = edge_props if edge_props is not None else {}
        return ActionSegment(
            action_type=action,
            edge_ids=['e1'],
            source_nodes=['WP1'],
            target_nodes=[target],
            edge_data=[{'edge_id': 'e1', 'source': 'WP1',
                        'target': target, 'properties': props}],
        )

    def test_no_params_returns_empty(self):
        """Segment with no relevant properties -> no parameters set."""
        server = _make_server()
        # Use a dict-of-dicts with a nodes accessor (networkx-style)
        server._graph = _FakeGraph({'WP2': {'properties': {}, 'name': 'WP2'}})
        server._get_ros_params_sync = lambda names: {}
        server._set_ros_params_async = lambda _d: None

        seg = self._make_segment()
        prev = server._apply_segment_parameters(seg)
        assert prev == {}

    def test_node_tolerances_queried_and_set(self):
        """xy/yaw goal tolerances from node properties are queried and set."""
        server = _make_server()
        server._graph = _FakeGraph({
            'WP2': {
                'name': 'WP2',
                'properties': {
                    'xy_goal_tolerance': 0.1,
                    'yaw_goal_tolerance': 0.05,
                },
            },
        })
        queried_names = []
        set_calls = []

        def _fake_get(names):
            queried_names.extend(names)
            return {n: 0.5 for n in names}  # pretend current value is 0.5

        server._get_ros_params_sync = _fake_get
        server._set_ros_params_async = lambda d: set_calls.append(dict(d))

        seg = self._make_segment()
        prev = server._apply_segment_parameters(seg)

        # Both tolerance param names should have been queried
        assert 'goal_checker.xy_goal_tolerance' in queried_names
        assert 'goal_checker.yaw_goal_tolerance' in queried_names

        # The returned dict contains the old values (0.5 faked above)
        assert prev.get('goal_checker.xy_goal_tolerance') == 0.5
        assert prev.get('goal_checker.yaw_goal_tolerance') == 0.5

        # SetParameters was called with new values
        assert len(set_calls) == 1
        assert set_calls[0]['goal_checker.xy_goal_tolerance'] == pytest.approx(0.1)
        assert set_calls[0]['goal_checker.yaw_goal_tolerance'] == pytest.approx(0.05)

    def test_edge_ros_parameters_queried_and_set(self):
        """Edge properties mapped via ros_parameters are queried and applied."""
        server = _make_server()
        server._graph = _FakeGraph({})  # no node in graph -> no tolerance params
        server._action_clients = {
            'row_traversal': {
                'client': None,
                'action_class': None,
                'config': {
                    'ros_parameters': {
                        'max_speed': 'FollowPath.max_robot_speed',
                    },
                },
            },
        }

        queried_names = []
        set_calls = []

        def _fake_get(names):
            queried_names.extend(names)
            return {'FollowPath.max_robot_speed': 1.0}

        server._get_ros_params_sync = _fake_get
        server._set_ros_params_async = lambda d: set_calls.append(dict(d))

        seg = self._make_segment(
            target='WP2',
            edge_props={'max_speed': 0.3},
            action='row_traversal',
        )
        prev = server._apply_segment_parameters(seg)

        assert 'FollowPath.max_robot_speed' in queried_names
        assert prev.get('FollowPath.max_robot_speed') == pytest.approx(1.0)
        assert set_calls[0]['FollowPath.max_robot_speed'] == pytest.approx(0.3)

    def test_restore_calls_set_with_prev_values(self):
        """_restore_segment_parameters sends saved values via SetParameters."""
        server = _make_server()
        set_calls = []
        server._set_ros_params_async = lambda d: set_calls.append(dict(d))

        prev = {'goal_checker.xy_goal_tolerance': 0.5,
                'goal_checker.yaw_goal_tolerance': 0.2}
        server._restore_segment_parameters(prev)

        assert len(set_calls) == 1
        assert set_calls[0] == prev

    def test_restore_empty_is_noop(self):
        """_restore_segment_parameters({}) must not call SetParameters."""
        server = _make_server()
        set_calls = []
        server._set_ros_params_async = lambda d: set_calls.append(d)

        server._restore_segment_parameters({})
        assert set_calls == []

    def test_get_ros_params_sync_unavailable_service(self):
        """_get_ros_params_sync returns {} when service is not ready."""
        server = _make_server()

        class _FakeClient:
            def service_is_ready(self):
                return False

        server._get_params_client = _FakeClient()

        result = server._get_ros_params_sync(['some_param'])
        assert result == {}

    def test_get_ros_params_sync_empty_names(self):
        """_get_ros_params_sync with empty list returns {} immediately."""
        server = _make_server()
        # No client needed – should return before touching it
        result = server._get_ros_params_sync([])
        assert result == {}


import pytest
