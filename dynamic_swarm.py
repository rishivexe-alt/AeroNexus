#!/usr/bin/env python3
"""Dynamic leader election swarm: no hard-coded leader.

Every drone runs this SAME program:   dynamic_swarm.py <id 0-4> [--criterion battery|id|centroid]

Roles are decided at run time by election.py from heartbeats on /swarm/heartbeat.
The leader holds position; followers keep formation slots relative to it.
Operator commands arrive on /swarm/cmd (see swarm_cli.py).
"""


import sys, json, time, argparse, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from election import Election, assign_slots

ALT, GROUND, ALT_STEP = 1.05, 0.06, 0.3
KP, VXY, VUP, VDN = 0.8, 0.6, 0.5, 0.4
OFFSETS = {   # (x, y) offset from the leader, by follower SLOT (1..4)
    'LINE': {1: (-1.5, 0), 2: (-3.0, 0), 3: (-4.5, 0), 4: (-6.0, 0)},
    'ARROW': {1: (-1.5, 1.5), 2: (-1.5, -1.5), 3: (-3.0, 3.0), 4: (-3.0, -3.0)},
}
# Simulated battery (%). Replace with the real battery topic on hardware.
INIT_BATT = [100.0, 95.0, 90.0, 85.0, 80.0]
DRAIN, LEADER_EXTRA = 0.3, 0.3          # %/s airborne; leader works harder


def clip(v, lim): return max(-lim, min(lim, v))
def alt_of(i): return ALT + ALT_STEP * i   # one flight level per drone


class SwarmNode(Node):
    def __init__(self, idx, criterion):
        super().__init__(f'swarm_drone{idx}')
        self.idx, self.pos = idx, None
        self.battery, self.failed = INIT_BATT[idx], False
        self.mode, self.hold = 'IDLE', None
        self.el = Election(idx, criterion, t0=time.monotonic())
        self.last_t, self.last_log = time.monotonic(), 0.0
        self.create_subscription(PoseStamped, f'/drone{idx}/ground_truth/pose',
                                 self._pose, qos_profile_sensor_data)
        self.vel = self.create_publisher(Twist, f'/gz/drone{idx}/cmd_vel', 10)
        self.hb_pub = self.create_publisher(String, '/swarm/heartbeat', 10)
        self.create_subscription(String, '/swarm/heartbeat', self.on_hb, 10)
        self.create_subscription(String, '/swarm/cmd', self.on_cmd, 10)
        self.create_timer(0.05, self.loop)
        self.create_timer(0.10, self.beat)
        self.get_logger().info(f'drone{idx} up, criterion={criterion}, '
                               f'battery={self.battery:.0f}%')

    # inputs
    def _pose(self, m):
        p = m.pose.position; self.pos = (p.x, p.y, p.z)

    def on_hb(self, m):
        d = json.loads(m.data)
        if d['id'] != self.idx and not self.failed:
            self.el.hear(d, time.monotonic())

    def on_cmd(self, m):
        a = m.data.split()
        if not a or self.pos is None: return
        k = a[0].upper(); x, y, _ = self.pos
        try:
            if k in ('T', 'TAKEOFF'):
                self.mode, self.hold = 'HOLD', (x, y, alt_of(self.idx))
            elif k in ('L', 'LAND'):
                self.mode, self.hold = 'LANDING', (x, y, GROUND)
            elif k in ('1', 'LINE') and self.mode in ('HOLD', 'LINE', 'ARROW'):
                self.mode = 'LINE'
            elif k in ('2', 'ARROW') and self.mode in ('HOLD', 'LINE', 'ARROW'):
                self.mode = 'ARROW'
            elif k == 'G' and self.el.leader == self.idx and self.hold:
                hx, hy, hz = self.hold
                self.hold = (hx + float(a[1]), hy + float(a[2]), hz)
            elif k == 'FAIL' and int(a[1]) == self.idx:
                self.failed = True
                self.get_logger().warn(f'drone{self.idx}: SIMULATED FAILURE (silent, hovering)')
            elif k == 'RECOVER' and int(a[1]) == self.idx:
                self.failed = False
                pin = self.el.pinned
                self.el = Election(self.idx, self.el.criterion, t0=time.monotonic())
                self.el.pinned = pin
                self.get_logger().info(f'drone{self.idx}: recovered, rejoining swarm')
            elif k == 'BATT' and int(a[1]) == self.idx:
                self.battery = float(a[2])
            elif k == 'LEADER':
                n = int(a[1]); self.el.pinned = True
                if self.el.leader == self.idx:
                    if n == self.idx:
                        self.get_logger().info(f'MANUAL: drone{n} stays leader (pinned)')
                    elif self.el.force(n, time.monotonic()):
                        self.get_logger().info(f'MANUAL HANDOVER: drone{self.idx} -> drone{n} '
                                               f'(term {self.el.term}), pinned')
                    else:
                        self.get_logger().warn(f'MANUAL: drone{n} is not alive, request refused')
            elif k == 'AUTO':
                self.el.pinned = False
                if self.el.leader == self.idx:
                    self.get_logger().info('AUTO: automatic leader selection re-enabled')
        except (IndexError, ValueError):
            self.get_logger().warn(f'bad command: {m.data}')

    # outputs
    def beat(self):
        if self.failed or self.pos is None: return
        self.hb_pub.publish(String(data=json.dumps({
            'id': self.idx, 'battery': round(self.battery, 2),
            'pos': list(self.pos), 'mode': self.mode,
            'leader': self.el.leader, 'term': self.el.term,
            'pinned': self.el.pinned})))

    def drive(self, t):
        x, y, z = self.pos
        ex, ey, ez = t[0] - x, t[1] - y, t[2] - z
        m = Twist()
        m.linear.x = clip(KP * ex, VXY); m.linear.y = clip(KP * ey, VXY)
        m.linear.z = clip(KP * ez * 2, VUP if ez > 0 else VDN)
        self.vel.publish(m)
        return max(abs(ex), abs(ey), abs(ez))

    # main loop
    def loop(self):
        if self.pos is None: return
        now = time.monotonic(); dt, self.last_t = now - self.last_t, now
        if self.failed:                        # dead drone: hover, say nothing
            self.vel.publish(Twist()); return
        if self.mode != 'IDLE':
            rate = DRAIN + (LEADER_EXTRA if self.el.leader == self.idx else 0)
            self.battery = max(0.0, self.battery - rate * dt)
        self.el.update_self(self.battery, self.pos, now)
        old = self.el.leader
        leader, changed, reason = self.el.elect(now)
        if changed:
            self.get_logger().info(
                f'LEADER CHANGE: {old} -> drone{leader} (term {self.el.term}) [{reason}]')
            if leader == self.idx and self.mode in ('LINE', 'ARROW', 'HOLD'):
                self.hold = (self.pos[0], self.pos[1], alt_of(self.idx))
                self.get_logger().info(f'drone{self.idx} is now LEADER, holding position')
        if self.mode == 'IDLE' or leader is None: return
        if self.mode in ('HOLD', 'LANDING'):
            self.drive(self.hold); return
        if leader == self.idx:                 # LINE / ARROW leader
            self.drive(self.hold); return
        lp = self.el.peers.get(leader)
        if lp is None: return
        slots = assign_slots(leader, self.el.alive_ids(now))
        ox, oy = OFFSETS[self.mode][slots[self.idx]]
        err = self.drive((lp['pos'][0] + ox, lp['pos'][1] + oy, alt_of(self.idx)))
        if now - self.last_log > 3:
            self.last_log = now
            self.get_logger().info(f'{self.mode} slot {slots[self.idx]} behind drone{leader}, '
                                    f'err={err:.2f} m, battery={self.battery:.1f}%')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('id', type=int, choices=range(5))
    ap.add_argument('--criterion', default='battery', choices=['battery', 'id', 'centroid'])
    a = ap.parse_args()
    rclpy.init()
    rclpy.spin(SwarmNode(a.id, a.criterion))


if __name__ == '__main__':
    main()
