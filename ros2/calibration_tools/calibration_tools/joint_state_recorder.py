import os
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class TerminalKeys:
    """Reads single key presses from a terminal stream in a background thread.

    Works in a plain terminal (e.g. inside `docker exec`) without an X server.
    The stream defaults to stdin; if stdin is a pipe, characters are read as they arrive,
    which allows scripted runs. ros2 launch does not pass the terminal to its processes as
    stdin, so under ros2 launch the stream is the controlling terminal (/dev/tty) instead.
    """

    def __init__(self, on_press, stream=None):
        self._on_press = on_press
        self._stream = stream if stream is not None else sys.stdin
        self._old_settings = None
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        if self._stream.isatty():
            import termios
            import tty
            self._old_settings = termios.tcgetattr(self._stream)
            tty.setcbreak(self._stream.fileno())
        self._thread.start()

    def stop(self):
        if self._old_settings is not None:
            import termios
            termios.tcsetattr(self._stream, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

    def _loop(self):
        while True:
            char = self._stream.read(1)
            if char == '':
                return
            self._on_press(char)


class PynputKeys:
    """Global key listener (needs an X server, keys are captured in any window)."""

    def __init__(self, on_press):
        import pynput
        self._on_press = on_press
        self._listener = pynput.keyboard.Listener(on_press=self._on_pynput_press)

    def _on_pynput_press(self, key):
        # Special keys (like arrows, ctrl, etc.) don't have char attribute
        char = getattr(key, 'char', None)
        if char is not None:
            self._on_press(char)

    def start(self):
        self._listener.start()

    def stop(self):
        self._listener.stop()


class JointStatesRecorder(Node):
    def __init__(self, joint_state_topic_name: str, folder_name: str, joint_names: list = None,
                 input_mode: str = 'terminal'):
        super().__init__("joint_states_recorder")

        self._joint_states_sub = self.create_subscription(
            JointState,
            joint_state_topic_name,
            self.joint_states_callback,
            qos_profile_sensor_data,
        )

        self._data = {"hole_0": [], "hole_1": []}
        self._joint_names = joint_names  # None: use all joints of the first message
        self._positions = None  # Latest positions ordered as self._joint_names
        self._active_hole = 0
        self._folder_name = folder_name
        self._done = threading.Event()

        if os.path.isdir(folder_name) and len(os.listdir(folder_name)) != 0:
            self.log(f"Folder {self._folder_name} already exists, appending to the recorded data.")
            self.read_data()
        else:
            self.log(f"Creating folder {self._folder_name}")
            os.makedirs(self._folder_name, exist_ok=True)

        # input_mode None: no key listener, on_press() is driven by the caller (tests)
        self._keys = None
        if input_mode == 'pynput':
            self._keys = PynputKeys(self.on_press)
        elif input_mode == 'terminal':
            self._keys = TerminalKeys(self.on_press)
        elif input_mode == 'tty':
            try:
                tty_stream = open('/dev/tty', 'r')
            except OSError as error:
                raise RuntimeError(
                    "--input tty needs a controlling terminal (/dev/tty); run it from a terminal, "
                    "e.g. with `docker exec -it`") from error
            self._keys = TerminalKeys(self.on_press, stream=tty_stream)

        self.print_info()
        self.print_status()
        if self._keys is not None:
            self._keys.start()

    def print_info(self):
        self.log("Press 'a' to add a data point")
        self.log("Press 'd' to delete the last data point")
        self.log("Press 's' to switch between holes")
        self.log("Press 'i' to show this help")
        self.log("Press 'q' to save the data and quit.")

    def log(self, msg: str):
        self.get_logger().info(msg)

    def hole_name(self):
        return f"hole_{self._active_hole}"

    def on_press(self, char: str):
        if self._done.is_set():
            return
        if char == 'a':
            if self._positions is None:
                self.log("No complete joint state received yet. "
                         "Please ensure the robot is publishing joint states.")
                return
            self._data[self.hole_name()].append(self._positions.tolist())
            self.log(f"Added data point for {self.hole_name()} with value {self._positions}")
        elif char == 'd':
            if len(self._data[self.hole_name()]) > 0:
                self._data[self.hole_name()] = self._data[self.hole_name()][0:-1]
                self.log(f"Deleted data point for {self.hole_name()}")
        elif char == 'q':
            self.log("Saving data to file and exiting")
            self.save_data()
            self._done.set()
            return
        elif char == 's':
            self._active_hole = (self._active_hole + 1) % 2
            self.log(f"Switched to hole {self._active_hole}")
        elif char == 'i':
            self.print_info()
        else:
            return
        self.print_status()

    def print_status(self):
        self.log("Saved joint for hole 0:")
        self.log_joint_length(len(self._data['hole_0']))
        self.log("Saved joint for hole 1:")
        self.log_joint_length(len(self._data['hole_1']))

        if self._joint_names is not None:
            self.log(f"Joint names: {', '.join(self._joint_names)}")

        self.log("Active hole is green:")
        self.print_circle_with_number(self._active_hole)

    def log_joint_length(self, length):
        # Create an ASCII block representation of the length
        block = '█' * length  # Each '█' represents one unit in the length
        self.log(f"{block} ({length})")

    def print_circle_with_number(self, number):
        color1, color2 = (32, 0) if number == 0 else (0, 32)
        square = f"""
            \033[{color1}m████████████   \033[{color2}m████████████
            \033[{color1}m█          █   \033[{color2}m█          █
            \033[{color1}m█     {0}    █   \033[{color2}m█     {1}    █
            \033[{color1}m█          █   \033[{color2}m█          █
            \033[{color1}m████████████   \033[{color2}m████████████
            \033[0m
        """
        self.log(square)

    def save_data(self):
        for hole_name, data in self._data.items():
            if len(data) == 0:
                continue
            file_name = os.path.join(self._folder_name, f"{hole_name}.csv")
            # The '#' header is skipped by np.loadtxt and by calibrate_fk.utils
            header = ','.join(self._joint_names) if self._joint_names is not None else ''
            np.savetxt(file_name, np.array(data), delimiter=",", header=header, comments='#')
            self.log(f"Saved {len(data)} points for {hole_name} to {file_name}")

    def read_data(self):
        for file_name in sorted(os.listdir(self._folder_name)):
            if not file_name.endswith('.csv'):
                continue
            hole_name = file_name.split(".")[0]
            file_path = os.path.join(self._folder_name, file_name)

            with open(file_path, 'r') as f:
                first_line = f.readline().strip()
            if first_line.startswith('#') and len(first_line) > 1:
                file_joint_names = [name.strip() for name in first_line[1:].split(',')]
                if self._joint_names is None:
                    self._joint_names = file_joint_names
                elif file_joint_names != list(self._joint_names):
                    self.get_logger().warn(
                        f"Joint names in {file_path} ({file_joint_names}) differ from the "
                        f"requested ones ({self._joint_names})")

            data = np.loadtxt(file_path, delimiter=",", comments='#', ndmin=2)
            self._data[hole_name] = data.tolist()
            self.log(f"Loaded {len(data)} points for {hole_name}")

    def joint_states_callback(self, msg: JointState):
        if self._joint_names is None:
            self._joint_names = list(msg.name)
            self.log(f"Auto-detected {len(self._joint_names)} joints: {self._joint_names}")

        missing = [name for name in self._joint_names if name not in msg.name]
        if missing:
            # Joint state publishers may split joints over several messages, ignore those
            self.get_logger().warn(f"Joints {missing} not found in message on "
                                   f"{self._joint_states_sub.topic_name}",
                                   throttle_duration_sec=5.0)
            return
        self._positions = np.array([msg.position[msg.name.index(name)]
                                    for name in self._joint_names])

    def run(self):
        try:
            while rclpy.ok() and not self._done.is_set():
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            if self._keys is not None:
                self._keys.stop()
