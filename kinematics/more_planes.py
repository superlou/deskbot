from sympy import *
from sympy.plotting import *

init_printing()

alpha, beta = symbols('alpha beta')

rot_x = Matrix([
    [1, 0, 0],
    [0, cos(alpha), -sin(beta)],
    [0, sin(alpha), cos(alpha)]
])

rot_y = Matrix([
    [cos(beta), 0, sin(beta)],
    [0, 1, 0],
    [-sin(beta), 0, cos(beta)]
])

surface_v = rot_y * rot_x * Matrix([0, 0, 1])

# Find vector normal to head surface
print "Surface vector:"
pprint(surface_v)

# Find plane of head at specific height in form ax + by + cz = d
x, y, z, h = symbols('x y z h')
print "\nSurface plane:"
plane_lh = surface_v[0]*x + surface_v[1]*y + surface_v[2]*z
point = Matrix([0, 0, h])

plane_rh = plane_lh.subs([
    (x, point[0]),
    (y, point[1]),
    (z, point[2])
])

plane = Eq(plane_lh, plane_rh)

pprint(plane)

# Find end effector planes and tracks
t0, t1, t2 = symbols('t0 t1 t2')

ee_plane_0_normal = Matrix([1, 0, 0])
ee_track_0 = surface_v.cross(ee_plane_0_normal) * t0 + point
print "\nTrack 0:"
pprint(ee_track_0)

ee_plane_1_normal = Matrix([-sin(30.*pi/180.).n(), cos(30.*pi/180.), 0])
ee_track_1 = surface_v.cross(ee_plane_1_normal) * t1 + point
print "\nTrack 1:"
pprint(ee_track_1)

ee_plane_2_normal = Matrix([cos(30.*pi/180.).n(), -sin(30.*pi/180.), 0])
ee_track_2 = surface_v.cross(ee_plane_2_normal) * t2 + point
print "\nTrack 2:"
pprint(ee_track_2)

# Build system of equations and solve for constraints
print "\nSolving:"
s = symbols('s')  # leg span

leg_0 = ee_track_0 - ee_track_1
leg_1 = ee_track_1 - ee_track_2
leg_2 = ee_track_2 - ee_track_0

# do L2 norm manually to prevent weird abs values
# In from f(t0, t1, t2) - s = 0
dist_0 = sqrt(leg_0[0] ** 2 + leg_0[1] ** 2 + leg_0[2] ** 2) - s
dist_1 = sqrt(leg_1[0] ** 2 + leg_1[1] ** 2 + leg_1[2] ** 2) - s
dist_2 = sqrt(leg_2[0] ** 2 + leg_2[1] ** 2 + leg_2[2] ** 2) - s

substitutions = [(alpha, 0), (beta, 0), (h, 2), (s, 2)]
dist_0 = dist_0.subs(substitutions)
dist_1 = dist_1.subs(substitutions)
dist_2 = dist_2.subs(substitutions)

soln = nsolve([dist_0, dist_1, dist_2], [t0, t1, t2], [-1, -1, -1])
ee_track_0 = ee_track_0.subs(substitutions)
ee_track_1 = ee_track_1.subs(substitutions)
ee_track_2 = ee_track_2.subs(substitutions)
c0 = ee_track_0.subs(t0, soln[0])
c1 = ee_track_1.subs(t1, soln[0])
c2 = ee_track_2.subs(t2, soln[0])
pprint(c0.n().T)
pprint(c1.n().T)
pprint(c2.n().T)

pprint(ee_track_0.T)
pprint(ee_track_1.T)
pprint(ee_track_2.T)
plot3d_parametric_line(ee_track_0[0], ee_track_0[1], ee_track_0[2], (t0, 0, -2))
