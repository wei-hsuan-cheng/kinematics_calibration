import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import JointState

from calibration_tools.joint_state_recorder import JointStatesRecorder

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3']


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def make_msg(names, positions):
    msg = JointState()
    msg.name = names
    msg.position = positions
    return msg


def test_filters_and_orders_joints(tmp_path):
    node = JointStatesRecorder('/joint_states', str(tmp_path), JOINT_NAMES, input_mode=None)
    node.joint_states_callback(make_msg(['joint_3', 'gripper', 'joint_1', 'joint_2'],
                                        [3.0, 9.0, 1.0, 2.0]))
    node.on_press('a')
    assert node._data['hole_0'] == [[1.0, 2.0, 3.0]]
    node.destroy_node()


def test_incomplete_message_is_not_recorded(tmp_path):
    node = JointStatesRecorder('/joint_states', str(tmp_path), JOINT_NAMES, input_mode=None)
    node.joint_states_callback(make_msg(['joint_1', 'joint_2'], [1.0, 2.0]))
    node.on_press('a')
    assert node._data['hole_0'] == []
    node.destroy_node()


def test_save_and_reload(tmp_path):
    node = JointStatesRecorder('/joint_states', str(tmp_path), JOINT_NAMES, input_mode=None)
    for i in range(3):
        node.joint_states_callback(make_msg(JOINT_NAMES, [i + 0.0, i + 0.1, i + 0.2]))
        node.on_press('a')
    node.on_press('s')
    node.joint_states_callback(make_msg(JOINT_NAMES, [5.0, 6.0, 7.0]))
    node.on_press('a')
    node.on_press('q')
    node.destroy_node()

    with open(tmp_path / 'hole_0.csv') as f:
        assert f.readline().strip() == '#' + ','.join(JOINT_NAMES)
    # Same call as the ROS-free loaders in calibrate_fk
    hole_0 = np.loadtxt(tmp_path / 'hole_0.csv', delimiter=',')
    assert hole_0.shape == (3, 3)
    np.testing.assert_allclose(hole_0[2], [2.0, 2.1, 2.2])

    # Restarting in the same folder appends to the recorded data
    node = JointStatesRecorder('/joint_states', str(tmp_path), None, input_mode=None)
    assert node._joint_names == JOINT_NAMES
    assert len(node._data['hole_0']) == 3
    assert node._data['hole_1'] == [[5.0, 6.0, 7.0]]
    node.destroy_node()
