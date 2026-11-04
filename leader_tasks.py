#!/usr/bin/env python3
"""Leader assigns tasks, followers execute and report.
Usage: leader_tasks.py leader | follower <1-4>"""
import sys, time, json, threading, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

ALT, GROUND = 1.05, 0.06
KP, VXY, VUP, VDN, TOL = 0.8, 0.6, 0.5, 0.4, 0.2
OFFSETS = {
    'LINE':  {1: (-1.5, 0), 2: (-3.0, 0), 3: (-4.5, 0), 4: (-6.0, 0)},
    'ARROW': {1: (-1.5, 1.5), 2: (-1.5, -1.5), 3: (-3.0, 3.0), 4: (-3.0, -3.0)},
}
AREAS = {
    'A': [(-2.5, -2.0), (-2.5, 2.0), (-1.0, 2.0), (-1.0, -2.0)],
    'B': [(-0.5, -2.0), (-0.5, 2.0), (0.5, 2.0), (0.5, -2.0)],
    'C': [(1.0, -2.0), (1.0, 2.0), (2.5, 2.0), (2.5, -2.0)],
}
HELP = ('Leader commands: T=takeoff L=land 1=LINE 2=ARROW | mission [delay] | '
        'assign N HOLD|LAND|LINE|ARROW|GOTO x y z | g dx dy | status | q')


def clip(v, lim): return max(-lim, min(lim, v))
def alt_of(d): return ALT + 0.3 * (d - 1)   # separate altitude per drone
def wps(area, z): return [[x, y, z] for x, y in AREAS[area]]


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
        self.lock = threading.Lock()
        self.lp_pub = self.create_publisher(PoseStamped, '/formation/leader_pose', 10)
        self.task_pub = {d: self.create_publisher(String, f'/formation/task/drone{d}', 10)
                         for d in (1, 2, 3, 4)}
        self.create_subscription(String, '/formation/status', self.on_status, 10)
        self.target, self.busy, self.done, self.owner = None, set(), set(), {}
        self.create_timer(0.05, self.loop)

    def loop(self):
        if self.pos is None: return
        m = PoseStamped(); m.header.frame_id = 'earth'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.pos
        self.lp_pub.publish(m)
        if self.target: self.drive(self.target)

    def send_task(self, d, task):
        self.task_pub[d].publish(String(data=json.dumps(task)))
        print(f'[LEADER] -> drone{d}: {task["type"]}' +
              (f' area {task["area"]} waypoints {[i+1 for i in task["todo"]]}'
               if task['type'] == 'SURVEY' else ''))

    def broadcast(self, what):
        x, y, z = self.target or self.pos
        if what == 'TAKEOFF': self.target = (self.pos[0], self.pos[1], ALT)
        if what == 'LAND': self.target = (x, y, GROUND)
        task = {'type': 'FORMATION', 'name': what} if what in OFFSETS else {'type': what}
        with self.lock: self.busy.clear()
        for d in (1, 2, 3, 4): self.send_task(d, task)

    def start_mission(self, delay=False):
        with self.lock:
            self.done.clear(); self.owner = {}
        for d, area in ((1, 'A'), (2, 'B'), (3, 'C')):
            t = {'type': 'SURVEY', 'area': area, 'wps': wps(area, alt_of(d)),
                 'todo': [0, 1, 2, 3]}
            if delay and d == 1: t['delay_at'] = 2   # simulated delay at A3
            with self.lock:
                self.busy.add(d); self.owner[area] = d
            self.send_task(d, t)

    def assign_cmd(self, a):
        d, what = int(a[0]), a[1].upper()
        if what in ('HOLD', 'LAND'): t = {'type': what}
        elif what in OFFSETS: t = {'type': 'FORMATION', 'name': what}
        elif what == 'GOTO': t = {'type': 'GOTO', 'target': [float(v) for v in a[2:5]]}
        else: print('unknown task'); return
        with self.lock: self.busy.discard(d)
        self.send_task(d, t)

    def on_status(self, m):
        s = json.loads(m.data); d, ev = s['drone'], s['event']
        if ev == 'WP_DONE':
            with self.lock: self.done.add((s['area'], s['index']))
            print(f'[LEADER] drone{d} finished {s["area"]}{s["index"]+1}')
        elif ev == 'DELAYED':
            print(f'[LEADER] drone{d} reports DELAY, remaining {s["area"]} '
                  f'waypoints {[i+1 for i in s["remaining"]]}')
            self.reassign(d, s['area'], s['remaining'])
        elif ev == 'TASK_DONE':
            with self.lock: self.busy.discard(d)
            print(f'[LEADER] drone{d} finished its task')
            self.verify()
        elif ev == 'GOTO_DONE':
            print(f'[LEADER] drone{d} reached its GOTO point')

    def reassign(self, src, area, remaining):
        with self.lock:
            self.busy.discard(src)
            helpers = [d for d in (1, 2, 3, 4) if d not in self.busy and d != src]
            if not helpers:
                print('[LEADER] ERROR: no idle helper available'); return
            h = helpers[-1]   # drone4 is the spare
            self.busy.add(h)
        print(f'[LEADER] TRANSFER: drone{src} -> drone{h}, area {area}')
        self.send_task(h, {'type': 'SURVEY', 'area': area, 'wps': wps(area, alt_of(h)),
                           'todo': list(remaining)})
        self.send_task(src, {'type': 'HOLD'})

    def verify(self):
        with self.lock:
            ok = all((a, i) in self.done for a in AREAS for i in range(4))
            busy = bool(self.busy)
        if ok and not busy:
            print('[LEADER] MISSION VERIFIED: areas A, B, C 4/4 each, assigned by leader')

    def print_status(self):
        with self.lock:
            for a in AREAS:
                n = sum(1 for (ar, _) in self.done if ar == a)
                print(f'  Area {a}: {n}/4')
            print(f'  busy drones: {sorted(self.busy)}')


class Follower(Base):
    def __init__(self, idx):
        super().__init__(idx, f'follower_drone{idx}')
        self.task, self.leader = None, None
        self.st_pub = self.create_publisher(String, '/formation/status', 10)
        self.create_subscription(String, f'/formation/task/drone{idx}', self.on_task, 10)
        self.create_subscription(PoseStamped, '/formation/leader_pose', self.on_leader, 10)
        self.create_timer(0.05, self.loop)

    def status(self, event, **kw):
        kw.update(drone=self.idx, event=event)
        self.st_pub.publish(String(data=json.dumps(kw)))

    def on_leader(self, m):
        p = m.pose.position; self.leader = (p.x, p.y, p.z)

    def on_task(self, m):
        if self.pos is None: return
        d = json.loads(m.data); k = d['type']
        if k in ('HOLD', 'LAND', 'TAKEOFF'):
            x, y, z = self.pos
            d['target'] = [x, y, {'HOLD': z, 'LAND': GROUND, 'TAKEOFF': ALT}[k]]
        self.task = d
        self.get_logger().info(f'task from leader: {k} ' +
                               str({a: d[a] for a in ('area', 'name', 'todo') if a in d}))

    def hold_here(self):
        self.task = {'type': 'HOLD', 'target': list(self.pos)}

    def loop(self):
        if self.pos is None or self.task is None: return
        t = self.task; k = t['type']
        if k in ('HOLD', 'LAND', 'TAKEOFF'):
            self.drive(t['target'])
        elif k == 'GOTO':
            if self.drive(t['target']) < TOL:
                self.status('GOTO_DONE'); self.hold_here()
        elif k == 'FORMATION':
            if self.leader is None: return
            ox, oy = OFFSETS[t['name']][self.idx]
            self.drive((self.leader[0]+ox, self.leader[1]+oy, ALT))
        elif k == 'SURVEY':
            if not t['todo']:
                self.status('TASK_DONE', area=t['area']); self.hold_here(); return
            i = t['todo'][0]
            if t.get('delay_at') == i and not t.get('delayed'):
                t['delayed'] = True
                self.status('DELAYED', area=t['area'], remaining=list(t['todo']))
                self.hold_here(); return
            if self.drive(t['wps'][i]) < TOL:
                self.status('WP_DONE', area=t['area'], index=i)
                t['todo'].pop(0)


def main():
    rclpy.init()
    if sys.argv[1] == 'follower':
        rclpy.spin(Follower(int(sys.argv[2]))); return
    L = Leader()
    threading.Thread(target=rclpy.spin, args=(L,), daemon=True).start()
    while L.pos is None: time.sleep(0.2)
    print(HELP)
    while True:
        try: c = input('Leader command > ').strip()
        except EOFError: break
        if not c: continue
        a = c.split(); k = a[0].upper()
        try:
            if k == 'Q': break
            elif k in ('T', 'L', '1', '2'):
                L.broadcast({'T': 'TAKEOFF', 'L': 'LAND', '1': 'LINE', '2': 'ARROW'}[k])
            elif k == 'MISSION': L.start_mission(len(a) > 1 and a[1].lower() == 'delay')
            elif k == 'ASSIGN': L.assign_cmd(a[1:])
            elif k == 'G':
                x, y, z = L.target or L.pos
                L.target = (x+float(a[1]), y+float(a[2]), z)
            elif k == 'STATUS': L.print_status()
            else: print(HELP)
        except Exception as e:
            print('error:', e, '\n' + HELP)
    rclpy.shutdown()


main()
