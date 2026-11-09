from autorace_objavoid.node import *


def test_outside_35_degree_primary_immediately_yields():
    z=Sector(float('inf'),20);c=Config()
    prior=(.4,.25,.06,.06,0.)
    outside=(.3,.25,.06,.06,0.)
    following=(.9,-.15,.06,.06,0.)
    before=Sectors(z,z,z,1.,1.,boxes=(prior,following))
    assert nearest_objects(before,c,prior)[0]==prior
    after=replace(before,boxes=(outside,following))
    assert nearest_objects(after,c,prior)==[following]
    overlapping=replace(after,boxes=((.05,.12,.06,.06,0.),))
    assert not nearest_objects(overlapping,c,prior)
    assert path_clearance(overlapping,0.,c)==0.


def test_priority_cone_edges_both_sides():
    z=Sector(float('inf'),20);c=Config()
    for sign in (-1,1):
        for degrees,ranked in ((34.9,True),(35.,True),(35.1,False),(90.,False),(120.,False)):
            a=math.radians(degrees);box=(.5*math.cos(a),sign*.5*math.sin(a),.03,.03,0.)
            scan=Sectors(z,z,z,1.,1.,boxes=(box,))
            assert bool(nearest_objects(scan,c,box))==ranked
