import json
import unittest
from pathlib import Path
from measured_luggage_policy import candidate_slots
from online_packing_demo import Box
from online_packing_real import Hull

class MeasuredInventoryTests(unittest.TestCase):
    def setUp(self):
        self.hull=Hull(1.49,1.97,1.48,1.535,.37,.057,1.383,.067,1.479,.53,-.745,-.985)
    def test_equal_dimensions_are_two_independent_suitcases(self):
        data=json.loads((Path(__file__).parent/'demo_inputs/site_thirteen_measured.json').read_text(encoding='utf-8'))
        self.assertEqual(len(data['boxes']),13)
        i=data['box_labels'].index('blackwhite'); j=data['box_labels'].index('blackbronze')
        self.assertNotEqual(i,j)
        self.assertEqual(data['boxes'][i],data['boxes'][j])
        self.assertEqual(data['boxes'][i],[.75,.49,.29])
        expected={'silver':[.56,.36,.21], 'pink':[.65,.41,.24],
                  'magenta':[.66,.42,.31], 'bronzewhite':[.67,.43,.27],
                  'bronze':[.72,.46,.28], 'smallgrey':[.41,.40,.22],
                  'blackpink':[.56,.34,.21]}
        self.assertEqual({name:data['boxes'][data['box_labels'].index(name)] for name in expected}, expected)
    def test_four_layer_limit_and_floor_slope(self):
        placed=[Box(str(i),(.55,.35,.2),(.325,.355,.2*i)) for i in range(4)]
        candidates=list(candidate_slots([.55,.35,.2],placed,[1,2,3,4],self.hull,1))
        self.assertTrue(candidates)
        self.assertTrue(all(level<=4 for _,_,_,_,level in candidates))
        self.assertFalse(any(abs(box.pos[2]-.8)<1e-6 and abs(x-.6)<1e-6 and abs(y-.53)<1e-6 for box,x,y,_,_ in candidates))
        self.assertTrue(all(box.pos[2]>0 or box.pos[1]+box.dims[1]<=self.hull.flat_y+1e-7 for box,*_ in candidates))
    def test_floor_edge_and_support_centres_are_candidates(self):
        slots=list(candidate_slots([.67,.43,.27],[],[],self.hull,1))
        self.assertTrue(any(abs(x-.6)<1e-6 and abs(y-1.32)<1e-6 and level==1
                            for _,x,y,_,level in slots))
        support=Box('base',(.67,.43,.27),(.265,1.105,0))
        upper=list(candidate_slots([.55,.36,.23],[support],[1],self.hull,1))
        self.assertTrue(any(abs(x-.6)<1e-6 and abs(y-1.32)<1e-6 and level==2
                            for _,x,y,_,level in upper))
