import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

phases = ('closed', 'open', 'closed', 'open')

def to_sec(t):
    return t.nanoseconds * 1e-9

class FeatureMeanCounter(Node):

    def __init__(self):
        super().__init__('feature_mean_counter')
        self.log = self.get_logger()
        self.declare_parameter('threshold', 251.0)
        self.declare_parameter('window', 60.0)
        self.declare_parameter('countdown', 5)
        self.declare_parameter('plot_path', 'feature_mean_sessions.png')
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.window = self.get_parameter('window').get_parameter_value().double_value
        self.countdown = self.get_parameter('countdown').get_parameter_value().integer_value
        self.plot_path = self.get_parameter('plot_path').get_parameter_value().string_value
        self.closed = []
        self.open = []
        self.collecting = False
        self.phase_start = None
        self.phase_offset = 0.0
        self.bucket = None
        self.create_subscription(Float64, '/feature_mean', self.handle_feature_mean, 1)

    def handle_feature_mean(self, msg):
        if not self.collecting or self.phase_start is None:
            return
        now = to_sec(self.get_clock().now())
        t = self.phase_offset + (now - self.phase_start)
        self.bucket.append((t, msg.data))

    def collect(self, state, offset):
        self.bucket = self.closed if state == 'closed' else self.open
        before = len(self.bucket)
        self.phase_offset = offset
        self.phase_start = None
        self.collecting = True
        last_mark = -1
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = to_sec(self.get_clock().now())
            if now <= 0.0:
                continue
            if self.phase_start is None:
                self.phase_start = now
                print(f'sampling {state} for {self.window:.0f}s', flush=True)
            elapsed = now - self.phase_start
            mark = int(elapsed) // 10
            if mark != last_mark:
                last_mark = mark
                print(
                    f'  {state}: {min(elapsed, self.window):.0f}/{self.window:.0f}s'
                    f'  samples={len(self.bucket) - before}',
                    flush=True)
            if elapsed >= self.window:
                break
        self.collecting = False
        print(f'{state} block done, {len(self.bucket) - before} new samples', flush=True)

    def report(self):
        for state, samples in (('closed', self.closed), ('open', self.open)):
            total = len(samples)
            if total == 0:
                print(f'x={state}: no samples', flush=True)
                continue
            below = sum(1 for _, value in samples if value <= self.threshold)
            above = total - below
            print(
                f'x={state}: above={above} below={below} total={total}',
                flush=True)
            print(f'P(z=open | x={state}) = {below / total}', flush=True)
            print(f'P(z=closed | x={state}) = {above / total}', flush=True)

    def save_plot(self):
        fig, ax = plt.subplots(figsize=(10, 4))
        if self.closed:
            t, v = zip(*self.closed)
            ax.plot(t, v, color='tab:blue', linewidth=0.8, label='closed')
        if self.open:
            t, v = zip(*self.open)
            ax.plot(t, v, color='tab:orange', linewidth=0.8, label='open')
        ax.axhline(self.threshold, color='black', linestyle='--', linewidth=1,
                   label=f'threshold {self.threshold:g}')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('feature_mean')
        ax.legend()
        fig.tight_layout()
        fig.savefig(self.plot_path)
        plt.close(fig)
        print(f'wrote {self.plot_path}', flush=True)

    def run(self):
        print(
            'Open the door with:\n'
            '  ros2 topic pub --once /hinged_glass_door/torque std_msgs/msg/Float64 "{data: 5.0}"\n'
            'Close it with:\n'
            '  ros2 topic pub --once /hinged_glass_door/torque std_msgs/msg/Float64 "{data: -5.0}"',
            flush=True)
        for index, state in enumerate(phases):
            print(
                f'\nPhase {index + 1}/4: put the door {state.upper()}, then press Enter.',
                flush=True)
            input()
            for tick in range(self.countdown, 0, -1):
                print(tick, flush=True)
                time.sleep(1)
            self.collect(state, index * self.window)
        self.report()
        self.save_plot()


def main():
    rclpy.init()
    node = FeatureMeanCounter()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
