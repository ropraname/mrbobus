import unittest
from mrbobus_console.routing import road_route
class RoutingTest(unittest.TestCase):
 def test_closed_edge_uses_other_branch(self):
  n={'nodes':{'a':[0,0],'b':[1,0],'c':[0,1],'d':[1,1]},'edges':[['a','b'],['a','c'],['c','d'],['d','b']]}
  s={'x':0,'y':0};g={'x':1,'y':0}
  self.assertEqual([k for k,_ in road_route(n,s,g)],["a","b"])
  self.assertEqual([k for k,_ in road_route(n,s,g,[("a","b")])],["a","c","d","b"])
 def test_no_route_does_not_invent_edge(self):
  n={'nodes':{'a':[0,0],'b':[1,0]},'edges':[]}
  with self.assertRaises(RuntimeError):road_route(n,{'x':0,'y':0},{'x':1,'y':0})
