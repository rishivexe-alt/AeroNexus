#!/usr/bin/env python3
"""Offline verification of the election logic: 5 virtual drones, simulated
10 Hz heartbeats with 20% random packet loss and a fake clock."""
import random
from election import Election, assign_slots

DT = 0.1


class Sim:
    def __init__(self, criterion='battery', batt=(100, 95, 90, 85, 80), loss=0.2,
                 drain=0.3, leader_extra=0.3, seed=1, positions=None):
        random.seed(seed)
        self.t = 0.0


        self.crit, self.loss = criterion, loss
        self.drain, self.extra = drain, leader_extra
        self.batt = {i: float(b) for i, b in enumerate(batt)}
        self.pos = positions or {i: (-1.5 * i, 0.0, 1.0) for i in range(5)}
        self.el = {i: Election(i, criterion) for i in range(5)}
        self.failed = set()
        self.changes = {i: [] for i in range(5)}   # (time, leader)
        self.airborne = False

    def step(self):
        self.t += DT
        for i, e in self.el.items():
            if i in self.failed:
                continue
            if self.airborne:
                rate = self.drain + (self.extra if e.leader == i else 0)
                self.batt[i] = max(0.0, self.batt[i] - rate * DT)
            e.update_self(self.batt[i], self.pos[i], self.t)
            ldr, ch, _ = e.elect(self.t)
            if ch:
                self.changes[i].append((round(self.t, 1), ldr))
        for i, e in self.el.items():           # broadcast
            if i in self.failed:
                continue
            hb = {'id': i, 'battery': self.batt[i], 'pos': self.pos[i],
                   'leader': e.leader, 'term': e.term}
            for j, o in self.el.items():
                if j != i and j not in self.failed and random.random() > self.loss:
                     o.hear(hb, self.t)

    def run(self, secs):
        for _ in range(int(secs / DT)):
            self.step()

    def views(self):
        return {i: e.leader for i, e in self.el.items() if i not in self.failed}

    def agreed(self):
        return len(set(self.views().values())) == 1


def test_initial_election():
    s = Sim(); s.run(6)
    assert s.agreed() and s.views()[0] == 0, s.views()
    print('PASS initial election -> drone0 (highest battery), all agree')


def test_leader_failure():
    s = Sim(); s.run(6); s.failed.add(0); s.run(5)
    assert s.agreed() and s.views()[1] == 1, s.views()
    t_fail = 6.0
    t_switch = max(t for i in (1, 2, 3, 4) for t, _ in s.changes[i][-1:])
    print(f'PASS leader failure -> drone1 elected by all, '
          f'recovery in {t_switch - t_fail:.1f} s')
    assert t_switch - t_fail < 3.0


def test_cascading_failure():
    s = Sim(); s.run(6); s.failed.add(0); s.run(5); s.failed.add(1); s.run(5)
    assert s.agreed() and s.views()[2] == 2, s.views()
    print('PASS cascading failure drone0 then drone1 -> drone2')


def test_rejoin_no_takeover():
    s = Sim(); s.run(6); s.failed.add(0); s.run(5)
    s.batt[0] = 92.0                                 # weaker than drone1 (95)
    s.failed.discard(0)
    s.el[0] = Election(0, 'battery', t0=s.t)         # node restarts
    s.run(8)
    assert s.agreed() and s.views()[0] == 1, s.views()
    print('PASS weaker recovered drone0 adopts incumbent drone1, no forced re-election')


def test_rejoin_much_stronger_takes_over():
    s = Sim(); s.run(6); s.failed.add(0); s.run(5)
    s.failed.discard(0)
    s.el[0] = Election(0, 'battery', t0=s.t)         # rejoins with 100 vs 95
    s.run(8)
    assert s.agreed() and s.views()[0] == 0, s.views()


    print('PASS much stronger rejoiner regains leadership via planned handover')


def test_battery_handover_no_flapping():
    s = Sim(); s.run(6); s.airborne = True; s.run(300)
    ch = [(t, l) for t, l in s.changes[2]]
    assert s.agreed(), s.views()
    times = [t for t, _ in ch]
    dwell = [b - a for a, b in zip(times, times[1:])]
    assert len(ch) >= 2 and min(dwell) > 10, (ch, dwell)
    print(f'PASS battery handover: {len(ch)-1} planned handovers in 300 s, '
          f'min dwell {min(dwell):.0f} s (no flapping), all agree')


def test_equal_battery_stable():
    s = Sim(batt=(90, 90, 90, 90, 90)); s.airborne = False; s.run(60)
    assert s.agreed() and all(len(c) == 1 for c in s.changes.values())
    print('PASS equal battery: tie-break lowest id, zero flapping')


def test_id_and_centroid():
    s = Sim('id'); s.run(6); assert s.agreed() and s.views()[3] == 0
    s.failed.add(0); s.run(5); assert s.agreed() and s.views()[3] == 1
    pos = {0: (0, 0, 1), 1: (-10, 0, 1), 2: (-2, 0, 1), 3: (2, 0, 1), 4: (-1, 0, 1)}
    c = Sim('centroid', positions=pos); c.run(6)
    assert c.agreed() and c.views()[0] == 2, c.views()   # centroid x=-2.2 -> drone2
    print('PASS id criterion and centroid criterion')


def test_manual_leader_and_pin():
    s = Sim(); s.run(6)
    for e in s.el.values(): e.pinned = True                  # 'leader N' pins everyone first
    assert s.el[0].force(3, s.t)
    s.run(3); assert s.agreed() and s.views()[1] == 3, s.views()
    s.airborne = True; s.run(200)
    assert s.agreed() and s.views()[1] == 3, s.views()      # pinned: no auto handover
    s.failed.add(3); s.run(5)
    assert s.agreed() and 3 not in s.views().values()        # safety: failover still works
    ldr = s.views()[0]
    assert not s.el[ldr].force(3, s.t)                       # dead target refused
    for e in s.el.values(): e.pinned = False
    print('PASS manual leader: pin holds 200 s, failover still works, dead target refused')


def test_slots():
    assert assign_slots(1, [0, 1, 2, 3, 4]) == {0: 1, 2: 2, 3: 3, 4: 4}
    assert assign_slots(0, [0, 2, 3]) == {2: 1, 3: 2}
    print('PASS slot assignment with dead / changed leader')


if __name__ == '__main__':
    for f in (test_initial_election, test_leader_failure, test_cascading_failure,
              test_rejoin_no_takeover, test_rejoin_much_stronger_takes_over,
              test_battery_handover_no_flapping,
              test_equal_battery_stable, test_id_and_centroid, test_slots,
              test_manual_leader_and_pin):
        f()
    print('\nALL TESTS PASSED')
