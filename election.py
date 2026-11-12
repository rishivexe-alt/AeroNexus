"""Dynamic leader election logic
Protocol (term-based, Raft-inspired, no voting rounds)
* A drone is ALIVE if its heartbeat was heard within `timeout` seconds.
* A claim is the pair (term, leader). Claims are totally ordered: higher
  term wins; for equal terms the LOWER leader id wins. Every drone adopts
  the highest claim it hears (whose leader it can see alive). This makes all
  views converge, even with packet loss.
* Initial election: after `startup` seconds with no claim heard, every drone
  independently computes the best-ranked alive drone and claims term 1.
* Failure: if my leader is not alive, I claim (term+1, best alive drone).
  Concurrent claims from different drones resolve via the ordering above.
* Planned handover: ONLY the current leader may start one. If a challenger
  beats it by the hysteresis margin, the leader claims (term+1, challenger).
  Followers never decide planned handovers on their own, so there is no
  threshold race and no flapping.
* Rejoin: a restarted drone has no claim, hears the swarm's claim and adopts
  it; it only becomes leader later through a planned handover.

Criteria
battery : highest battery wins (tie -> lowest id); challenger must lead by
            `margin` percentage points to trigger a planned handover.
id       : lowest id wins; never pre-empts a live leader (bully-style).
centroid : drone closest to the swarm centroid wins; challenger must be
            closer by `centroid_margin` metres.
"""
import math


def assign_slots(leader, alive_ids):
    """Followers sorted by id get formation slots 1..n."""
    followers = sorted(i for i in alive_ids if i != leader)
    return {i: k + 1 for k, i in enumerate(followers)}


def claim_key(term, leader):
    return (term, -leader)


class Election:
    def __init__(self, my_id, criterion='battery', timeout=1.5, margin=5.0,
                 startup=3.0, t0=0.0, centroid_margin=1.0):
        if criterion not in ('battery', 'id', 'centroid'):
            raise ValueError('criterion must be battery, id or centroid')
        self.my_id, self.criterion = my_id, criterion
        self.timeout, self.margin = timeout, margin
        self.startup, self.t0 = startup, t0
        self.centroid_margin = centroid_margin
        self.peers = {}      # id -> {'battery','pos','seen','leader','term'}
        self.leader = None
        self.term = 0
        self.pinned = False   # True = manual leader, no automatic handovers

    # data in
    def update_self(self, battery, pos, now):
        self.peers[self.my_id] = {'battery': battery, 'pos': tuple(pos),
                                  'seen': now, 'leader': self.leader,
                                  'term': self.term}

    def hear(self, hb, now):
        if hb['id'] == self.my_id:
            return
        self.peers[hb['id']] = {'battery': hb['battery'],
                                'pos': tuple(hb['pos']), 'seen': now,
                                'leader': hb.get('leader'),
                                'term': hb.get('term', 0)}

    def alive_ids(self, now):
        return sorted(i for i, p in self.peers.items()
                      if now - p['seen'] <= self.timeout)

    # ranking
    def _dist(self, i, alive):
        xs = [self.peers[j]['pos'][0] for j in alive]
        ys = [self.peers[j]['pos'][1] for j in alive]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        x, y = self.peers[i]['pos'][:2]
        return math.hypot(x - cx, y - cy)


    def _key(self, i, alive):
        if self.criterion == 'battery':
            return (self.peers[i]['battery'], -i)
        if self.criterion == 'id':
            return (-i,)
        return (-self._dist(i, alive), -i)

    def best(self, alive):
        return max(alive, key=lambda i: self._key(i, alive))

    def _better_enough(self, challenger, current, alive):
        if self.criterion == 'battery':
            return (self.peers[challenger]['battery'] -
                    self.peers[current]['battery']) >= self.margin
        if self.criterion == 'id':
            return False
        return (self._dist(current, alive) -
                self._dist(challenger, alive)) >= self.centroid_margin

    # decision
    def _set(self, term, leader):
        self.term, self.leader = term, leader

    def force(self, target, now):
        """Manual handover (called on the CURRENT leader only). Returns True if done."""
        if target == self.leader or target not in self.alive_ids(now):
            return False
        self._set(self.term + 1, target)
        return True

    def elect(self, now):
        """Returns (leader, changed, reason)."""
        alive = self.alive_ids(now)
        if not alive:
            return self.leader, False, ''
        old = self.leader

        # 1. adopt the highest claim heard from a live peer naming a live leader
        claims = [(p['term'], p['leader']) for i, p in self.peers.items()
                   if i != self.my_id and i in alive
                   and p['leader'] is not None and p['leader'] in alive]
        if claims:
            top = max(claims, key=lambda c: claim_key(*c))
            mine = (claim_key(self.term, self.leader)
                     if self.leader is not None else (-1, 0))
            if claim_key(*top) > mine:
                self._set(*top)
                return self.leader, self.leader != old, \
                     f'adopted claim term {top[0]}'

        reason = ''
        if self.leader is None:
            if now - self.t0 >= self.startup:
                self._set(self.term + 1, self.best(alive))
                reason = 'initial election'
        elif self.leader not in alive:
            self._set(self.term + 1, self.best(alive))
            reason = f'leader drone{old} lost (heartbeat timeout)'
        elif self.leader == self.my_id and not self.pinned:
            b = self.best(alive)
            if b != self.my_id and self._better_enough(b, self.my_id, alive):
                self._set(self.term + 1, b)
                reason = f'{self.criterion} handover'
        return self.leader, self.leader != old, reason
