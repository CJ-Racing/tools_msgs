#!/usr/bin/env python3
"""地图重复地标实时监控 (诊断用, 可随时删除)

订阅 map_Core 的输出, 每 3s 打印一次:
  - total_landmarks : 所有层地标总数 (含 learning+unknown)
  - stable          : map_stable 里稳定锥桶数
  - min_gap         : 稳定锥桶两两最近间距 (正常赛道 >3m)
  - NN<0.5/1.0/2.0m : 有邻居落在该半径内的锥桶数 (>0 即重复)
  - max_cluster     : 单点 0.5m 内堆叠的最大锥桶数 ("残影"量化)
"""
import rospy
import math
import time
from tools_msgs.msg import ConeArray
from std_msgs.msg import Int32, Float32, Bool


class DupMonitor:
    def __init__(self):
        rospy.init_node('map_dup_monitor', anonymous=True)
        self.stable = None
        self.total = None
        self.dist = None
        self.done = None
        rospy.Subscriber('/map/cones_map/map_stable', ConeArray, self._cb_stable)
        rospy.Subscriber('/map/cones_map/telemetry/cone_count', Int32,
                         lambda m: setattr(self, 'total', m.data))
        rospy.Subscriber('/map/cones_map/telemetry/distance', Float32,
                         lambda m: setattr(self, 'dist', m.data))
        rospy.Subscriber('/map/cones_map/telemetry/map_complete', Bool,
                         lambda m: setattr(self, 'done', m.data))
        rospy.Timer(rospy.Duration(3.0), self._report)
        print('=== dup_monitor started, waiting for data ===', flush=True)

    def _cb_stable(self, msg):
        self.stable = [(c.position.x, c.position.y, c.color) for c in msg.cones]

    def _report(self, _event):
        t = time.strftime('%H:%M:%S')
        tot = self.total if self.total is not None else '?'
        dist = f'{self.dist:.1f}' if self.dist is not None else '?'
        done = self.done
        pts = self.stable
        if not pts:
            print(f'[{t}] total_landmarks={tot} dist={dist} complete={done} stable=0 (waiting...)',
                  flush=True)
            return
        n = len(pts)
        dup05 = dup10 = dup20 = 0
        min_d = 1e9
        max_cluster = 1
        worst = None
        for i in range(n):
            xi, yi, _ = pts[i]
            nn = 1e9
            cluster = 1
            for j in range(n):
                if i == j:
                    continue
                d = math.hypot(xi - pts[j][0], yi - pts[j][1])
                if d < nn:
                    nn = d
                if d < 0.5:
                    cluster += 1
            if nn < min_d:
                min_d = nn
            if nn < 0.5:
                dup05 += 1
            if nn < 1.0:
                dup10 += 1
            if nn < 2.0:
                dup20 += 1
            if cluster > max_cluster:
                max_cluster = cluster
                worst = (xi, yi)
        worst_s = f' worst@({worst[0]:.1f},{worst[1]:.1f})' if worst else ''
        print(f'[{t}] total_landmarks={tot} stable={n} dist={dist}m complete={done} | '
              f'min_gap={min_d:.2f}m  NN<0.5m={dup05}  NN<1.0m={dup10}  NN<2.0m={dup20}  '
              f'max_cluster@0.5m={max_cluster}{worst_s}', flush=True)


if __name__ == '__main__':
    try:
        DupMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
