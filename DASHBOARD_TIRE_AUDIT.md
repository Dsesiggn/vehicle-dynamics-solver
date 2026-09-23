# Dashboard tire-model audit and TTC calibration

Audited source: Dsesiggn/vehicle-dynamics-solver, commit `98ea6f2`,
`Dynamics_Dashboard.html`. The original is preserved locally as
`Dynamics_Dashboard.original.html`. Changes are local; nothing was pushed to GitHub.

## Findings

1. **Incorrect peak-angle calibration (fixed).** Original lines 334-340 use
   `B = 1.82 / target_peak`, `C = 1.3`, `E = -0.2`. These produce a peak at
   approximately 1.315 times the requested angle: 5.39 deg for the 4.1 deg input,
   and 9.07 deg for the 6.9 deg input. The estimate mode now solves the peak
   condition exactly. The TTC mode fits measured forces instead of imposing
   the old peak-angle inputs.
2. **Invalid inverse-search interval and saturation values (fixed).** Original
   lines 331-348 bisect over the complete 0-15 deg or 0-25 percent interval.
   A peaked curve is not monotonic over that interval. Near saturation this can
   return a descending-branch or boundary value instead of the first ascending
   solution. The new inverse searches only up to the first peak or fit boundary.
   Unattainable force returns an explicit limit state, not an invented 15 deg
   or 25 percent slip.
3. **Inconsistent friction capacity (fixed with an approximation).** Original
   lines 330 and 417-421 allow longitudinal peak force `1.1 * mu * Fz`, but use
   a circular `mu * Fz` limit for utilization. Now each axis uses its own
   maximum within its fitted slip range, with elliptical utilization
   `sqrt((Fx/Fx_cap)^2 + (Fy/Fy_cap)^2)`. This is an engineering approximation,
   not a fitted combined-slip Magic Formula. Displayed SA/SR remain pure-slip
   equivalents; they are not actual simultaneous-slip solutions.
4. **G-sweep inconsistent with current state (fixed).** Original lines 437-448
   omit longitudinal load transfer and reuse the current state's corner Fx at
   every sweep G. Both load transfer and corner longitudinal demand are now
   recomputed consistently for each sweep point.
5. **Lift-off invalidates the load-transfer approximation (flagged, not solved).**
   Original line 403 clips individual negative normal loads to zero without
   re-solving contact equilibrium. The sum of loads can therefore exceed total
   weight plus downforce after lift. The updated UI flags wheel lift; chassis
   contact equilibrium and rollover physics remain outside this tire update.
6. **Model limitations remain.** Lateral demand is allocated by axle weight
   bias and corner normal load, not steering/yaw equilibrium. No tire camber
   thrust, pressure dependence, relaxation length, transients or aligning-moment
   fit is implemented. The static anti-pitch factors and synthetic dynamic aero
   equations were not calibrated here. The hardpoints JSON is absent from the
   published snapshot, so the dashboard uses its existing fallback geometry.
   Handling labels describe saturation bias, not a solved understeer gradient.
   The history horizontal axis now says update index; UI changes are not elapsed
   simulation time.

## Original coefficients

With Fz positive in N, the original assumed:

```text
mu(Fz) = 1.5 + (2.8 - 1.5) * exp(-0.0015 * Fz)
t = clamp((Fz - 300) / 700, 0, 1)
target_angle = 4.1 + t * (6.9 - 4.1)  [deg]

Lateral:      B = 1.82/target_angle [1/deg], C = 1.3, E = -0.2, D = mu*Fz [N]
Longitudinal: B = 1.16/12 [1/percent], C = 1.65, E = 0, D = 1.1*mu*Fz [N]
```

These are assumptions in the code, not TTC fit results.

## Formula and units

The updated unshifted, symmetric pure-slip model is:

```text
z = B*x
F = D * sin(C * atan(z - E*(z - atan(z))))
D = forceScale * (D/Fz coefficient) * Fz
```

`x` is slip angle in **degrees** for lateral force, and slip ratio as a
**fraction** for longitudinal force. The UI displays longitudinal slip in percent.
The linear stiffness at zero slip is B*C*D, in N/deg or N/fraction respectively.

- Convert B from a radians-based lateral model with `B_degree = B_radian*pi/180`.
- Convert B from a percent-based longitudinal model with `B_fraction = 100*B_percent`.
- C and E are dimensionless. D is a force amplitude, not always the maximum
  reached inside the test interval. The editable `D/Fz` values are dimensionless.
- Manual fitting constraints are B > 0, D/Fz > 0, 1.01 <= C <= 2, and
  -20 <= E <= 0.999. These keep the inverse's ascending-branch assumptions valid;
  this is not a general MF5.2/MF6.2 parameter-file reader.
- The earlier MATLAB normalized-slip B/C/D/E exports are **not** direct inputs
  to this formula. Their slip normalization and force/shift conversions must be
  reconstructed, or the dimensional curve must be refitted. This update refits
  raw SA and FY directly, avoiding that ambiguity.

Formula reference: [MathWorks Tire-Road Interaction (Magic Formula)](https://www.mathworks.com/help/sdl/ref/tireroadinteractionmagicformula.html).

## TTC source and fit

User-confirmed source: `B2356run8.mat`, FSAE TTC Round 9, Hoosier 43075
16x7.5-10 R20, 8 inch rim. Source channels specify SA and IA in degrees, P in
kPa, FZ/FY in N, and V in km/h.

Filters: 12 psi +/- 0.8 psi after converting P from kPa; IA 0 deg +/- 0.25 deg;
speed > 30 km/h; |SA| <= 12 deg; and Fz within 60 N of each nominal 50, 100,
150, 200 and 250 lbf condition. SL is zero in this run and is filtered at
|SL| <= 0.02. **SR is approximately -1 throughout the run and must not be used
as a zero-slip filter for this file.** Normal load is converted to magnitude;
FY's sign is oriented to give a positive small-angle slope.

FY/|FZ| is reduced to median values in signed 0.25 deg bins with at least five
samples per bin. All qualifying sweeps are pooled. A multistart bounded robust
least-squares fit estimates B/C/D/Fz/E jointly across both signed branches.
No force shift, slip shift or imposed peak angle is used. Data from tread-center
temperatures approximately 49-71 C are pooled, not temperature-corrected.

| Median Fz (N) | B (1/deg) | C | D/Fz, belt | E | Belt mu RMSE |
|---:|---:|---:|---:|---:|---:|
| 217.13 | 0.387282 | 1.010000 | 2.738699 | 0.529207 | 0.03485 |
| 437.79 | 0.316625 | 1.010000 | 2.720406 | 0.374224 | 0.02342 |
| 661.34 | 0.242666 | 1.203191 | 2.559552 | 0.368468 | 0.02809 |
| 885.45 | 0.193731 | 1.397369 | 2.456892 | 0.386189 | 0.02724 |
| 1107.65 | 0.156898 | 1.598512 | 2.380503 | 0.375941 | 0.03139 |

C hits its lower bound for the two lightest loads. Their true peak and individual
coefficients are not well identified by this finite sweep. These parameters are
useful curve approximations over the measured range, not uniquely identified
physical tire properties. Reported RMSE is **in-sample** on the calibration bins;
there is no independent test-run validation here.

The TTC mode interpolates B/C/D/Fz/E between all five measured load rows. Outside
the tested load range it holds boundary coefficients, still multiplying D/Fz by
the query Fz. This extrapolated force is not validated; the UI flags affected
corners. Manual mode instead interpolates between the two editable light/heavy
reference sets, initially seeded from the 437.79 N and 1107.65 N fits.

The default forceScale is **2/3**, reflecting the user's cited Round guidance.
Stored coefficients remain belt values. Use scale **1** for coefficients that
have already been corrected to track. Constant force scaling changes force and
stiffness but does not change peak or normalized geometric knee angles. It is
not evidence that the same angles occur on a real racing surface.

Longitudinal coefficients remain estimates because this is a cornering run.
They are clearly identified as such; no longitudinal TTC fit is claimed.

## Peaks and knees

The UI separates the model peak (or maximum at the selected slip boundary) from
a geometric knee. The knee maximizes normalized force minus normalized slip on
the positive branch up to the first peak/boundary, excluding angles below 1 deg.
The symmetric model supplies a corresponding negative-angle branch. This knee
definition depends on the selected interval and is not a physical force maximum,
an independently measured optimum, or a replacement for traction capacity.

At the example loads of 450 and 1100 N, the fitted geometric knees are about
3.76 and 4.48 deg. The 450 N curve is still increasing at 12 deg; the 1100 N
model peak is about 11.30 deg. Four of the five fitted reference curves reach
their range maximum at 12 deg. Do not report those endpoints as observed peaks.

## Using the update

Place **both** `Dynamics_Dashboard.html` and `ttc_b2356run8_calibration.js` in the
same folder, alongside your existing hardpoints JSON when available. The HTML
loads Tailwind and Plotly from the same CDNs as the original, so it needs internet
access. It can open directly with fallback geometry; serve the folder with
`python3 -m http.server 8000` to load the hardpoints JSON reliably.

The initial mode is TTC run8 / lateral. Set Preview Fz to your required load.
Choose Manual B / C / D / E to edit the two reference coefficient sets. Changing
coefficients updates the curve, equivalent slips, utilization and G-sweep.
The reference IA and pressure in manual mode are metadata only; changing them
does not generate a new fit. Reference-set changes are held only for the current
page session and reset on reload.

To reproduce fitting (requires NumPy, SciPy and Matplotlib):

```bash
python3 fit_dashboard_ttc.py /path/to/B2356run8.mat
node test_dashboard_tires.cjs
```

The fit creates the companion JS, a full precision JSON with filters, source
hash, residuals and bin medians, and `ttc_calibration_validation.png` for visual
comparison. Raw TTC data is not included in the update package.
