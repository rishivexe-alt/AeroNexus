#!/usr/bin/env python3
import sys, time, threading, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

ALT, GROUND = 1.05, 0.06
KP, VXY, VUP, VDN = 0.8, 0.6, 0.5, 0.4
OFFSETS = {  # offsets from the leader (x, y), per follower id
    'LINE':  {1: (-1.5, 0), 2: (-3.0, 0), 3: (-4.5, 0), 4: (-6.0, 0)},
    'ARROW': {1: (-1.5, 1.5), 2: (-1.5, -1.5), 3: (-3.0, 3.0), 4: (-3.0, -3.0)},
}
KEYS = {'1': 'LINE', '2': 'ARROW', 'T': 'TAKEOFF', 'L': 'LAND',
        'LINE': 'LINE', 'ARROW': 'ARROW', 'TAKEOFF': 'TAKEOFF', 'LAND': 'LAND'}


def clip(v, lim): return max(-lim, min(lim, v))


class Base(Node):
    def __init__(self, idx, name):
        super().__init__(name)
        self.idx, self.pos = idx, None
        self.create_subscription(PoseStamped, f'/drone{idx}/ground_truth/pose',
                                 self._pose, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, f'/gz/drone{idx}/cmd_vel', 10)

    def _pose(self, m):
        p = m.pose.position; self.pos = (p.x, p.y, p.z)

    def drive(self, t):
        x, y, z = self.pos
        ex, ey, ez = t[0]-x, t[1]-y, t[2]-z
        m = Twist()
        m.linear.x = clip(KP*ex, VXY); m.linear.y = clip(KP*ey, VXY)
        m.linear.z = clip(KP*ez*2, VUP if ez > 0 else VDN)
        self.pub.publish(m)
        return max(abs(ex), abs(ey), abs(ez))


class Leader(Base):
    def __init__(self):
        super().__init__(0, 'leader_drone0')
        self.cmd_pub = self.create_publisher(String, '/formation/command', 10)
        self.lp_pub = self.create_publisher(PoseStamped, '/formation/leader_pose', 10)
        self.target = None
        self.create_timer(0.05, self.loop)

    def loop(self):
        if self.pos is None: return
        m = PoseStamped(); m.header.frame_id = 'earth'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.pos
        self.lp_pub.publish(m)
        if self.target: self.drive(self.target)

    def send(self, cmd):
        self.cmd_pub.publish(String(data=cmd))
        x, y, _ = self.target if self.target else self.pos
        if cmd == 'TAKEOFF': self.target = (self.pos[0], self.pos[1], ALT)
        if cmd == 'LAND': self.target = (x, y, GROUND)
        print(f'[LEADER] broadcast {cmd}')


class Follower(Base):
    def __init__(self, idx):
        super().__init__(idx, f'follower_drone{idx}')
        self.mode, self.fixed, self.leader, self.last = 'IDLE', None, None, 0
        self.create_subscription(String, '/formation/command', self.on_cmd, 10)
        self.create_subscription(PoseStamped, '/formation/leader_pose',
                                 self.on_leader, 10)
        self.create_timer(0.05, self.loop)

    def on_leader(self, m):
        p = m.pose.position; self.leader = (p.x, p.y, p.z)

    def on_cmd(self, m):
        cmd = m.data
        self.get_logger().info(f'received command from leader: {cmd}')
        if self.pos is None: return
        if cmd == 'TAKEOFF':
            self.mode, self.fixed = 'HOLD', (self.pos[0], self.pos[1], ALT)
        elif cmd == 'LAND':
            self.mode, self.fixed = 'HOLD', (self.pos[0], self.pos[1], GROUND)
        elif cmd in OFFSETS:
            self.mode = cmd

    def loop(self):
        if self.pos is None or self.mode == 'IDLE': return
        if self.mode == 'HOLD': t = self.fixed
        elif self.leader is None: return
        else:
            ox, oy = OFFSETS[self.mode][self.idx]
            t = (self.leader[0]+ox, self.leader[1]+oy, ALT)
        err = self.drive(t)
        if time.time() - self.last > 3:
            self.last = time.time()
            self.get_logger().info(f'mode={self.mode} error={err:.2f} m')


def main():
    rclpy.init()
    if sys.argv[1] == 'follower':
        rclpy.spin(Follower(int(sys.argv[2]))); return
    node = Leader()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    while node.pos is None: time.sleep(0.2)
    print('Leader commands: 1=LINE 2=ARROW T=TAKEOFF L=LAND g dx dy=move leader q=quit')
    while True:
        c = input('Leader command > ').strip()
        if c.lower() == 'q': break
        if c.upper() in KEYS: node.send(KEYS[c.upper()])
        elif c.startswith('g '):
            try:
                dx, dy = map(float, c.split()[1:3])
            except ValueError:
                print('usage: g <dx> <dy>, e.g. g 2 0'); continue
            x, y, z = node.target or node.pos
            node.target = (x+dx, y+dy, z); print(f'[LEADER] moving by ({dx}, {dy})')
    rclpy.shutdown()


main()
