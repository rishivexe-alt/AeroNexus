#!/usr/bin/env python3
"""Operator console for the dynamic-leader swarm.

Commands:    T takeoff | L land | 1 LINE | 2 ARROW | g dx dy (moves current leader)
             leader N (manual leader) | auto (automatic again)
             fail N | recover N | batt N PERCENT | status | q
"""
import json, time, threading, rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Cli(Node):
    def __init__(self):
        super().__init__('swarm_cli')
        self.pub = self.create_publisher(String, '/swarm/cmd', 10)
        self.hb = {}
        self.create_subscription(String, '/swarm/heartbeat', self.on_hb, 10)

    def on_hb(self, m):
        d = json.loads(m.data); d['t'] = time.monotonic(); self.hb[d['id']] = d

    def status(self):
        now = time.monotonic()
        print(' drone state battery mode          sees leader term')
        for i in range(5):
            d = self.hb.get(i)
            if d is None or now - d['t'] > 1.5:
                print(f' {i}       DEAD/SILENT'); continue
            lead = 'LEADER ' if d['leader'] == i else 'follower'
            print(f' {i}       {lead} {d["battery"]:6.1f}% {d["mode"]:8s} '
                  f'drone{d["leader"]}       {d["term"]}')
        views = {d['leader'] for d in self.hb.values() if now - d['t'] <= 1.5}
        print(' consensus:', 'YES' if len(views) == 1 else f'NO {views}')
        pins = {d.get('pinned') for d in self.hb.values() if now - d['t'] <= 1.5}
        print(' leader selection:', 'MANUAL (pinned)' if True in pins else 'AUTOMATIC')


def main():
    rclpy.init(); n = Cli()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    print(__doc__)
    while True:
        try: c = input('swarm > ').strip()
        except EOFError: break
        if not c: continue
        if c.lower() == 'q': break
        if c.lower() == 'status': n.status()
        else: n.pub.publish(String(data=c))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
