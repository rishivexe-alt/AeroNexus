#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped

N = 5
LINE = [(0,0),(-1.5,0),(-3.0,0),(-4.5,0),(-6.0,0)]
V = [(0,0),(-1.5,1.5),(-1.5,-1.5),(-3.0,3.0),(-3.0,-3.0)]
ALT, GROUND = 1.05, 0.06
KP, VXY, VUP, VDN, TOL = 0.8, 0.6, 0.5, 0.4, 0.15


class Fleet(Node):
    def __init__(self):
        super().__init__('direct_formation')
        self.pos = [None]*N
        self.pub = []
        for i in range(N):
            self.create_subscription(
                PoseStamped, f'/drone{i}/ground_truth/pose',
                lambda m, i=i: self.pos.__setitem__(
                    i, (m.pose.position.x, m.pose.position.y, m.pose.position.z)),
                qos_profile_sensor_data)
            self.pub.append(self.create_publisher(Twist, f'/gz/drone{i}/cmd_vel', 10))
        while None in self.pos:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.origin = self.pos[0][:2]

    @staticmethod
    def clip(v, lim): return max(-lim, min(lim, v))

    def goto(self, targets, vup=VUP, vdn=VDN, timeout=240, hold=2.0):
        ok_since, t0 = None, time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            worst = 0.0
            for i, (tx, ty, tz) in enumerate(targets):
                x, y, z = self.pos[i]
                ex, ey, ez = tx-x, ty-y, tz-z
                worst = max(worst, abs(ex), abs(ey), abs(ez))
                m = Twist()
                m.linear.x = self.clip(KP*ex, VXY)
                m.linear.y = self.clip(KP*ey, VXY)
                m.linear.z = self.clip(KP*ez*2, vup if ez > 0 else vdn)
                self.pub[i].publish(m)
            if worst < TOL:
                ok_since = ok_since or time.time()
                if time.time() - ok_since > hold: break
            else:
                ok_since = None
        print('  max error %.2f m' % worst)

    def formation(self, offs):
        ox, oy = self.origin
        self.goto([(ox+dx, oy+dy, ALT) for dx, dy in offs])

    def stop(self):
        for p in self.pub: p.publish(Twist())


def main():
    rclpy.init()
    f = Fleet()
    print('Commands: TAKEOFF LINE V_FORMATION LAND EXIT')
    while True:
        c = input('Leader command > ').strip().upper()
        if c == 'TAKEOFF':
            f.goto([(f.pos[i][0], f.pos[i][1], ALT) for i in range(N)])
        elif c == 'LINE': f.formation(LINE)
        elif c == 'V_FORMATION': f.formation(V)
        elif c == 'LAND':
            f.goto([(f.pos[i][0], f.pos[i][1], GROUND) for i in range(N)])
        elif c == 'EXIT': break
        f.stop()
    rclpy.shutdown()


main()
