# tests for boids.py

import boids

def test_Vec3D_eq():
    a = boids.Vec3D(1.1, 2.3, 4)
    b = boids.Vec3D(1.1, 2.3, 4)
    assert a == b
    assert (a is b) is False
    c = boids.Vec3D(0, 1, 2)
    assert (a == c) is False

def test_Vec3D_ne():
    a = boids.Vec3D(1.1, 2.3, 4)
    b = boids.Vec3D(1.1, 2.3, 4)
    assert (a != b) is False
    c = boids.Vec3D(0, 1, 2)
    assert a != c

def test_Vec3D_unit():
    a = boids.Vec3D(1, 0, 0)
    # Allow for float error
    assert a.unit().mag() > 0.999999 and a.unit().mag() < 1.000001
    b = boids.Vec3D(2.3, 1.2, 0.3)
    unit = boids.Vec3D(0.880715518228, 0.45950374864, 0.11487593716)
    #print b.unit()
    #print unit
    #print "(b.unit() - unit).mag()", (b.unit() - unit).mag()
    # Allow for float error
    assert (b.unit() - unit).mag() < 0.0000001

def test_Vec3D_dist():
    a = boids.Vec3D(1.1, 2.3, 4)
    b = boids.Vec3D(1.1, 2.3, 4)
    dist = a.dist(b)
    #print dist
    assert a.dist(b) == 0
    c = boids.Vec3D(0, 0, 0)
    d = boids.Vec3D(1, 1, 1)
    dist = c.dist(d)
    #print dist
    assert dist == 1.7320508075688772
    c = boids.Vec3D(0, 0, 0)
    d = boids.Vec3D(-1, -1, -1)
    dist = c.dist(d)
    #print dist
    assert dist == 1.7320508075688772

def vec3d_float_eq(a, b, factor=1.000001):
    """Are these two Vec3Ds approximately equal? (Allowing for float incinsistencies...)"""
    x = False
    y = False
    z = False
    if (abs(a.x) * (1/factor)) <= abs(b.x) <= (abs(a.x) * factor):
        #print "x matches"
        x = True
    if (abs(a.y) * (1/factor)) <= abs(b.y) <= (abs(a.y) * factor):
        #print "y matches."
        y = True
    if (abs(a.z) * (1/factor)) <= abs(b.z) <= (abs(a.z) * factor):
        #print "z matches"
        z = True
    return x and y and z
        
def test_ObstacleSphere_correction_out():

    # Boid is outside of obstacle
    boid = boids.Boid()
    boid.set_position(1,2,2)
    obstacle = boids.ObstacleSphere(0, 0, 0, 1, 1, 2)
    #print "correction", obstacle.correction(boid)
    assert obstacle.correction(boid) == boids.Vec3D(0, 0, 0)

def test_ObstacleSphere_correction_in():
    # Boid is inside radius_max of obstacle
    boid = boids.Boid()
    boid.set_position(0.7, -0.3, 0.23)
    obstacle = boids.ObstacleSphere(0, 0, 0, 1, 1, 2)
    correction = obstacle.correction(boid)
    target = boids.Vec3D(0.879894275382, -0.377097546592, 0.289108119054)
    #print "correction:", correction, "target:", target
    # Allow for float error
    assert vec3d_float_eq(correction, target)

def test_ObstacleSphere_correction_mid():
    # Boid is midway between the max and min radius
    boid = boids.Boid()
    boid.set_position(1.5, 0, 0)
    obstacle = boids.ObstacleSphere(0, 0, 0, 15, 1, 2)
    correction = obstacle.correction(boid)
    target = boids.Vec3D(7.5, 0, 0)
    print "correction:", correction
    print "target:", target
    assert vec3d_float_eq(correction, target)
    


tests = [test_Vec3D_eq,
         test_Vec3D_ne,
         test_Vec3D_unit,
         test_Vec3D_dist,
         test_ObstacleSphere_correction_out,
         test_ObstacleSphere_correction_in,
         test_ObstacleSphere_correction_mid,
         ]

for test in tests:
    print "Testing", test.__name__
    test()
    print "\tPass."


