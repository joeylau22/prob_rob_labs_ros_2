import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

heartbeat_period = 0.1
door_torque = 5.0
travel_distance = 4.0
stop_time = 1.0
retry_period = 4.0
close_belief_threshold = 0.999

def to_sec(t):
    return t.nanoseconds * 1e-9

def clamp_p(p):
    return min(max(p, 1e-6), 1.0 - 1e-6)

def predict_open(bel_open, p_open_command):
    bel_open = bel_open + p_open_command * (1.0 - bel_open)
    bel_open = min(max(bel_open, 1e-12), 1.0 - 1e-12)
    return bel_open, math.log(bel_open / (1.0 - bel_open))

def bayes_update(log_odds, z_open, p_z_open_x_open, p_z_open_x_closed):
    p_z_open_x_open = clamp_p(p_z_open_x_open)
    p_z_open_x_closed = clamp_p(p_z_open_x_closed)
    if z_open:
        log_odds += math.log(p_z_open_x_open) - math.log(p_z_open_x_closed)
    else:
        log_odds += math.log(1.0 - p_z_open_x_open) - math.log(1.0 - p_z_open_x_closed)
    bel_open = 1.0 / (1.0 + math.exp(-log_odds))
    return log_odds, bel_open

class OpenAndPassDoor(Node):

    def __init__(self):
        super().__init__('open_and_pass_door')
        self.log = self.get_logger()
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('open_threshold', 321.0)
        self.declare_parameter('p_z_open_x_open', 0.9)
        self.declare_parameter('p_z_open_x_closed', 0.1)
        self.declare_parameter('p_open_command', 0.2)
        self.declare_parameter('move_belief', 0.99)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.open_threshold = self.get_parameter(
            'open_threshold').get_parameter_value().double_value
        self.p_z_open_x_open = self.get_parameter(
            'p_z_open_x_open').get_parameter_value().double_value
        self.p_z_open_x_closed = self.get_parameter(
            'p_z_open_x_closed').get_parameter_value().double_value
        self.p_open_command = self.get_parameter(
            'p_open_command').get_parameter_value().double_value
        self.move_belief = self.get_parameter(
            'move_belief').get_parameter_value().double_value
        if self.speed <= 0.0:
            self.speed = 0.2
        self.speed = min(self.speed, 0.5)
        self.feature_means = []
        self.is_open = False
        self.is_close = False
        self.log_odds_open = None
        self.log_odds_close = 0.0
        self.bel_open = 0.0
        self.bel_closed = 0.5
        self.command_time = None
        self.pub_door = self.create_publisher(Empty, '/door_open', 1)
        self.pub_torque = self.create_publisher(
            Float64, '/hinged_glass_door/torque', 1)
        self.pub_cmd = self.create_publisher(
            TwistStamped, '/cmd_vel', 1)
        self.sub_feature_mean = self.create_subscription(
            Float64, '/feature_mean', self.handle_feature_mean, 1)
        self.state = 'open'
        self.state_start = to_sec(self.get_clock().now())
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def handle_feature_mean(self, msg):
        self.feature_means.append(msg.data)
        z_open = msg.data <= self.open_threshold
        if self.state == 'open' and not self.is_open and self.log_odds_open is not None:
            self.log_odds_open, self.bel_open = bayes_update(
                self.log_odds_open, z_open,
                self.p_z_open_x_open, self.p_z_open_x_closed)
            self.log.info(f'bel(x=open)={self.bel_open}')
            if self.bel_open >= self.move_belief:
                self.is_open = True
                self.log.info(f'is_open=True bel(x=open)={self.bel_open}')
        elif self.state == 'close' and not self.is_close:
            self.log_odds_close, bel_open = bayes_update(
                self.log_odds_close, z_open,
                self.p_z_open_x_open, self.p_z_open_x_closed)
            self.bel_closed = 1.0 - bel_open
            self.log.info(f'bel(x=closed)={self.bel_closed}')
            if self.bel_closed > close_belief_threshold:
                self.is_close = True
                self.log.info(f'is_close=True bel(x=closed)={self.bel_closed}')

    def heartbeat(self):
        now = to_sec(self.get_clock().now())
        elapsed = now - self.state_start
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        cmd.twist.angular.z = 0.0
        torque = None
        cmd.twist.linear.x = 0.0
        if self.state == 'open':
            if now > 0.0 and (self.command_time is None or (
                    now - self.command_time >= retry_period and
                    self.bel_open < self.move_belief)):
                self.pub_door.publish(Empty())
                self.bel_open, self.log_odds_open = predict_open(
                    self.bel_open, self.p_open_command)
                self.command_time = now
                self.log.info(f'door_open bel_bar(x=open)={self.bel_open}')
            if self.is_open:
                self.state = 'drive'
                self.state_start = now
                self.log.info(f'state: {self.state}')
        elif self.state == 'drive':
            torque = 0.0
            cmd.twist.linear.x = self.speed
            if elapsed > travel_distance / self.speed:
                self.state = 'stop'
                self.state_start = now
                self.log.info(f'state: {self.state}')
        elif self.state == 'stop':
            torque = 0.0
            if elapsed > stop_time:
                self.state = 'close'
                self.state_start = now
                self.log.info(f'state: {self.state}')
        elif self.state == 'close':
            torque = -door_torque
            if self.is_close:
                self.state = 'done'
                self.state_start = now
                self.log.info(f'state: {self.state}')
        else:
            torque = 0.0
        if torque is not None:
            self.pub_torque.publish(Float64(data=torque))
        self.pub_cmd.publish(cmd)

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    open_and_pass_door = OpenAndPassDoor()
    open_and_pass_door.spin()
    open_and_pass_door.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
