import unittest
from types import SimpleNamespace
from uav_coverage_memory.resources import ByteWindow, WorkBudget


class ResourceTests(unittest.TestCase):
    def test_cache_uses_bytes_and_count_without_slicing(self):
        cache=ByteWindow(3,10)
        a,b,c=[SimpleNamespace(data=b'x'*n) for n in (3,4,5)]
        for item in (a,b,c):self.assertTrue(cache.append(item))
        self.assertEqual(list(cache),[b,c]);self.assertEqual(cache.bytes,9)
        self.assertFalse(cache.append(SimpleNamespace(data=b'x'*11)))
        self.assertEqual(cache.bytes,0);self.assertEqual(len(cache),0)
        for i in range(5):cache.append(SimpleNamespace(data=b'x'))
        self.assertEqual(len(cache),3)

    def test_heavy_frame_rest_and_no_catch_up_burst(self):
        budget=WorkBudget(3,.1,150)
        report=budget.finish(10,10.2,.15)
        self.assertTrue(report['budget_overrun'])
        self.assertAlmostEqual(budget.next_ready,11.5)
        self.assertFalse(budget.ready(11.4));self.assertTrue(budget.ready(12))
        budget.finish(12,12.02,.01)
        self.assertAlmostEqual(budget.next_ready,12+1/3)

    def test_io_or_contention_does_not_require_extra_cpu_rest(self):
        budget=WorkBudget(3,.1)
        report=budget.finish(10,12,.05)
        self.assertEqual(report['cooldown_ms'],0)

    def test_bad_budget_limits_rejected(self):
        for kwargs in [dict(cpu_fraction=0),dict(cpu_fraction=2),dict(max_hz=float('nan')),dict(warn_wall_ms=-1)]:
            with self.assertRaises(ValueError):WorkBudget(**kwargs)
        with self.assertRaises(ValueError):ByteWindow(2,0)

    def test_light_work_keeps_nominal_ticks_at_float_boundaries(self):
        budget=WorkBudget(3,.15)
        for i in range(40):
            now=i/3.
            self.assertTrue(budget.ready(now))
            budget.finish(now,now+.025,.025)


if __name__=='__main__':unittest.main()
