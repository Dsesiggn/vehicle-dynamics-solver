# Geometric anti-effects: derivation and scope

This model reports an instantaneous **nominal load-transfer fraction**, not
predicted pitch, squat, suspension force, or equilibrium ride height.

## Source evidence reviewed before implementation

The supplied source pages were visually read in full. Extracts/images remain
outside the repository. Milliken Ch17 §17.3 printed pp.617–619 / PDF328–329,
Figs17.12–17.15 distinguishes contact-patch versus wheel-center force lines
according to where brake/drive torque is reacted. It explicitly separates
geometric anti from total longitudinal load transfer. Seward Ch3 §3.4.1–3.4.3
printed pp.71–73 / PDF82–84, Fig3.8 and Eqs3.1–3.2 gives the outboard-brake
front fraction `100 f L tan(alpha)/h`, including front braking fraction.
Gillespie Ch7 printed pp.248,250–255 / PDF264,266–271, Figs7.12–7.13 and
Eqs7.10–7.29 independently derives the torque-reaction and front/rear force-split
dependence. Its rear independent-drive result uses `(e−r)/d`, whereas the
torque-reacting solid-axle example uses `e/d`.

Gillespie's **zero body pitch** equations include front/rear spring rates and
are different from a single axle having 100% anti-squat. They are not used here.
Thus the apparently different criteria describe different quantities, not a
license to infer chassis attitude from a geometric anti percentage. The current
application supports independent wheel suspensions; no solid axle is created.

## Original spatial virtual-work derivation

Let b be upward wheel-center bump (mm), so `v_C,z = dCz/db = −1`. The exact
corner constraint tangent supplies `v_C = dC/db` and spatial
`omega = d(rotation)/db` (rad/mm). A unit longitudinal tire force acts along
vehicle `e_X = [1,0,0]`. Let `r = Q−C` run from wheel center to the agreed rigid
circle support point, and n be the unit wheel spindle. Its moment per unit
force at C is `m = r cross e_X` (mm).

If spin torque is reacted by the upright (e.g. an outboard brake), use all m.
If spin torque is reacted by the chassis (ideal inboard brake or independent
drive), remove only the spindle component: `m_perp = m−n(n dot m)`. The
remaining bearing overturning moment is retained. Replacing this spatial wrench
by a force at the wheel center would be wrong at arbitrary camber/steer; the
center-force simplification follows only when m is entirely along n.

Virtual work gives the generalized bump force per unit longitudinal force:

`lambda = e_X dot v_C + m_reacted dot omega`.

The dimensional check is `(mm/mm) + (mm)*(rad/mm)`, a dimensionless quantity.
For an axle force fraction f, entered positive nominal CG height h, and nominal
wheelbase L, the reported anti percentage is

`A_front = 100 f L lambda / h`, `A_rear = −100 f L lambda / h`.

Braking uses the entered front braking fraction (rear is its complement): front
is anti-dive, rear anti-lift. Drive uses the entered front tractive-force fraction:
front is anti-lift, rear anti-squat. These signs follow +X forward and +b upward.
For braking Fx is negative: positive front lambda opposes bump; negative rear
lambda opposes rebound. Acceleration reverses Fx and the corresponding load
transfer. Equal left/right force split is explicit; a corner curve uses the
axle fraction and compares against half of the nominal axle load transfer.

In a planar front example with side-view pivot d behind and H above contact,
`lambda = H/d` for upright torque reaction. With chassis spin reaction it is
`(H−radius)/d`. This recovers the supplied moment equations. The spatial method
uses the complete constraint tangent, including steering links; it does not
substitute a possibly different knuckle-point instant center for a force center.

## Required inputs and limits

CG height, front brake/drive fractions, and each axle's brake/drive torque
reaction are explicit nullable inputs. None is inferred from a race-car label.
The nominal L/h reference stays fixed through the chassis-fixed travel sweep;
this is a comparative geometric curve, not a body-on-road simulation.
Longitudinal force is parallel to vehicle X, not an inferred steering-dependent
tire-force vector. Unsprung/wheel inertia, gyroscopic effects, driveshaft plunge
friction, compliance, force-dependent tire deformation and transient load
transfer are excluded. Ideal torque cancellation assumes wheel-spin equilibrium.
Negative and over-100% values remain signed results, not clamped limits.

Software verification must separately check planar limiting cases, zero fraction,
signs, mirror symmetry, and an inclined-spindle wrench that cannot be reduced
to a center force. No measured vehicle validation is claimed.
