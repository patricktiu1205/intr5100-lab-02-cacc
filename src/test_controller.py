import importlib.util
import math
import random
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from controller import PredictivePID, middle_command, load_geometry
from simulate import StraightRoute, StraightSection, run
from idm_controller import IDMController, IDMParams


class ControllerTests(unittest.TestCase):
    def test_middle_replica(self):
        rng=random.Random(5100)
        real=IDMController(IDMParams())
        last=0.
        for _ in range(1000):
            v,vf,s,dt=rng.uniform(0,100),rng.uniform(0,100),rng.uniform(20,1500),rng.uniform(.05,.35)
            active=rng.choice([True,True,False])
            if active:
                expected=real.next_velocity(v,vf,s,dt,v_max=70)
            else:
                expected=vf
                real.last_cmd_v=vf
            expected=round(0 if s<80 else min(100,max(0,expected)))
            actual,last=middle_command(v,vf,s,dt,last,active)
            self.assertEqual(actual,expected)
            self.assertAlmostEqual(last,real.last_cmd_v)

    def test_history_and_reentry(self):
        c=PredictivePID(route=StraightRoute(),sections=[StraightSection()])
        states=[SimpleNamespace(x=x,y=0,velocity=40) for x in (2000,1300,600)]
        h=[]
        for n in range(20):
            for role in c.samples:
                h.append(dict(role=role,t_from_start=n*.2,velocity=40+n))
            value=c.step(*states,h,now=n*.2)
            self.assertTrue(math.isfinite(value) and 0<=value<=100)
        self.assertLessEqual(len(c.samples['leader']),12)
        self.assertAlmostEqual(c.acceleration('leader'),5.)
        self.assertTrue(math.isfinite(c.step(*states,h,now=10)))
        new=[]
        c.step(*states,new,now=11)
        self.assertEqual(len(c.samples['leader']),0)
        states[2].velocity=float('nan')
        self.assertEqual(c.step(*states,new,now=11.2),0)

    def test_route_wrapping_and_sections(self):
        r,sections=load_geometry()
        self.assertEqual(len(sections),3)
        self.assertAlmostEqual(r.forward_gap(20,r.loop_length-30),50.)
        c=PredictivePID(route=r,sections=sections)
        self.assertTrue(c.same(sections[0].start_s+10,sections[0].start_s+20))
        self.assertFalse(c.same(sections[0].start_s+10,sections[1].start_s+20))

    def test_stale_telemetry_and_sensor(self):
        c=PredictivePID(route=StraightRoute(),sections=[StraightSection()])
        states=[SimpleNamespace(x=x,y=0,velocity=40,recv_time=time.time()-2) for x in (2000,1300,600)]
        self.assertEqual(c.step(*states,now=0),0)
        for s in states:
            s.recv_time=time.time()
        states[2].data={'center_distance':.1}
        self.assertEqual(c.step(*states,now=.2),0)

    def test_variants_are_standalone(self):
        root=Path(__file__).resolve().parents[1]
        for folder,scale in [('unscaled',1.),('scaled_2p8',2.8)]:
            p=root/'controllers'/folder/'student_controller2.py'
            spec=importlib.util.spec_from_file_location(folder,p)
            m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
            self.assertEqual(m.GAMMA,scale)
            self.assertEqual(m.BETA,scale)
            states=[SimpleNamespace(x=x,y=y,velocity=40) for x,y in [(3000,5400),(3600,5400),(4200,5400)]]
            self.assertTrue(math.isfinite(m.student_control(*states,history=[])))

    def test_closed_loop_nominal(self):
        for scale in (1.,2.8):
            m,_=run(scale,scale,duration=80)
            self.assertFalse(m['collision'])
            self.assertEqual(m['emergency_samples'],0)


if __name__=='__main__':
    unittest.main()
