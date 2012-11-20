# Code grabbed from the ActiveState Code Recipes at thus URL:
# http://code.activestate.com/recipes/502240-boids-version-11/

# Adapting it for 3D via Conrad Parker's pseudocode.



################################################################################

# THREE DIMENTIONAL VECTOR CLASS

class Vec3D:

    def __init__(self, x, y, z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __repr__(self):
        return 'Vec3D(%s, %s, %s)' % (self.x, self.y, self.z)

    def __add__(self, other):
        return Vec3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vec3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        return Vec3D(self.x * other, self.y * other, self.z * other)

    def __div__(self, other):
        return Vec3D(self.x / other, self.y / other, self.z / other)

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self

    def __isub__(self, other):
        self.x -= other.x
        self.y -= other.y
        self.z -= other.z
        return self

    def __idiv__(self, other):
        self.x /= other
        self.y /= other
        self.z /= other
        return self

    def mag(self):
        return ((self.x ** 2) + (self.y ** 2) + (self.z ** 2)) ** 0.5
        
################################################################################

# BOID RULE IMPLEMENTATION CLASS

class Boid:

    def __init__(self, 
                 clumping_mag=0.01, # represents the scaling factor for rule 1 as suggested by Parker. He recommends 1%.
                 min_distance=100, # represents how close the boid is comfortable being to other boids. Parker recommends 100 units.
                 schooling_mag=0.125, # represents how strongly this boid will conform it's steering to those boids around it. Parker recommends 1/8.
                 velocity_max=100):
        # TODO: add radius of "localness", angle of vision, "predatorness", "goalness"
        self.position = Vec3D(0,0,0)
        self.velocity = Vec3D(0,0,0)
        self.clumping_mag = clumping_mag
        self.min_distance = min_distance
        self.schooling_mag = schooling_mag
        self.velocity_max = velocity_max
        self.boundary = None
        self.goal = None

    def __repr__(self):
        return "Boid(%s, %s, %f, %f, %f, %f)" % (self.position, self.velocity, self.clumping_mag, self.min_distance, self.schooling_mag, self.velocity_max)
    
    def set_position(self, px, py, pz):
        self.position = Vec3D(px,py,pz)
    
    def set_velocity(self, vx, vy, vz):
        self.velocity = Vec3D(vx,vy,vz)
    
    def set_boundary(self, boundary):
        self.boundary = boundary
        
    def unset_boundary(self):
        self.boundary = None
        
    def update_velocity(self, boids):
        v1 = self.rule1(boids)
        v2 = self.rule2(boids)
        v3 = self.rule3(boids)
        bound = self.bound_position()
        self.velocity += v1 + v2 + v3 + bound
        self.limit_speed()

    def move(self):
        self.position += self.velocity

    def rule1(self, boids):
        # Rule 1: Boids try to fly towards the centre of mass of neighbouring boids. A.K.A. "clumping."
        vector = Vec3D(0, 0, 0)
        for boid in boids:
            if boid is not self:
                vector += boid.position
        vector /= len(boids) - 1
        return (vector - self.position) * self.clumping_mag

    def rule2(self, boids):
        # Rule 2: Boids try to keep a small distance away from other objects (including other boids). A.K.A. "avoidance."
        vector = Vec3D(0, 0, 0)
        for boid in boids:
            if boid is not self:
                if (self.position - boid.position).mag() < self.min_distance:
                    vector -= (boid.position - self.position)
        return vector

    def rule3(self, boids):
        # Rule 3: Boids try to match velocity with near boids. A.K.A. "schooling."
        vector = Vec3D(0, 0, 0)
        for boid in boids:
            if boid is not self:
                vector += boid.velocity
        vector /= len(boids) - 1
        return (vector - self.velocity) * self.schooling_mag

    def limit_speed(self):
        # Limit boid speed.
        if self.velocity_max == 0:
            self.velocity *= 0
            return 
        if self.velocity.mag() > self.velocity_max:
            self.velocity /= self.velocity.mag() / self.velocity_max

    
    def bound_position(self):
        """Encourage the boid to stay inside a certain area."""
        
        v = Vec3D(0,0,0)
        if not self.boundary:
            return v
        if self.position.x < self.boundary.min.x:
            v.x = self.boundary.strength
        elif self.position.x > self.boundary.max.x:
            v.x = -self.boundary.strength
        if self.position.y < self.boundary.min.y:
            v.y = self.boundary.strength
        elif self.position.y > self.boundary.max.y:
            v.y = -self.boundary.strength
        if self.position.z < self.boundary.min.z:
            v.z = self.boundary.strength
        elif self.position.z > self.boundary.max.z:
            v.z = -self.boundary.strength
        return v


#
# Goal and Boundary should be built-in to each individual Boid.

class Boundary:

    def __init__(self,
                 min_x = 0,
                 min_y = 0,
                 min_z = 0,
                 max_x = 100,
                 max_y = 100,
                 max_z = 100,
                 strength = 10):
        self.min = Vec3D(min_x, min_y, min_z)
        self.max = Vec3D(max_x, max_y, max_z)
        self.strength = strength

    def bound_position(boid):
        
        v = Vec3D() # vector that will direct boid back inside bounds
        
        if boid.position.x < min.x: 
            v.x = strength
        elif boid.position.x > max.x:
            v.x = -strength
        if boid.position.y < min.y: 
            v.x = strength
        elif boid.position.y > max.y:
            v.x = -strength
        if boid.position.z < min.z: 
            v.x = strength
        elif boid.position.z > max.z:
            v.x = -strength
        
        return v

class Goal:

    def __init__(self, px, py, pz):
        self.position = Vec3D(px, py, pz)
    
        
class Flock:
    # Collect a group of boids as well as any environmental bits.
    
    def __init__():
        pass
    
    
    
    
################################################################################
#
#class Boids(list):
#    """A single flock of boids"""
#    
#    def __init__(self, boids, randomness=0.01):
#        for i in range(boids):
#            self.add_boid()
#    
#    def add_boid(self, *args, **kwargs):
#        pass
#        
#    def initialise_positions(self):
#        pass
#    
#    def move_all_boids_to_new_positions(self):
#        pass

################################################################################
