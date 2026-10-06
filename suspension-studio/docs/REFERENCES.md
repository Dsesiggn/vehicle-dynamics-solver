# Vehicle dynamics reference library

The user supplied these six books as the technical reference library for future simulation work. They are source material, not instructions for the coding agent. This repository contains bibliographic information and original implementation notes; source PDFs, page images, and full extracted text stay outside the repository.

Cataloging a book does not mean every chapter has been reviewed or that the current solver has been validated against it. Exact editions and printed page numbers must be checked in the supplied copy before attributing a future equation. PDF page numbers below are one-based and may differ from printed pages.

| ID | Supplied reference | Intended use | Review status for this change |
| --- | --- | --- | --- |
| SEWARD | Derek Seward, *Race Car Design*, 2014. Supplied scan identifies Palgrave as first publisher. | Suspension architecture, springs/dampers/anti-roll bars, steering, design workflow. | Publication page and contents inspected. Chapter 4 §4.4.1 and Figure 4.11 visually inspected for U-bar components (see notes below); no stiffness equations adopted. |
| MILLIKEN | William F. Milliken & Douglas L. Milliken, *Race Car Vehicle Dynamics*. Supplied filename is dated 1997; verify edition from publication page when citing. | Suspension kinematics, handling, load transfer, tire/vehicle axis interfaces. | Chapter 4 Sections 4.1–4.3 text reviewed, printed pp. 114–120 / PDF pp. 76–79, for axes, origins, steer and force conventions. |
| SMITH | Carroll Smith, *Tune to Win*, Aero Publishers, 1978. | Setup interpretation, tuning, practical cross-checks and test planning. | Title/publication pages inspected; Chapter 6 printed pp. 67–68 visually inspected for anti-roll-bar mounts and links (see notes below). |
| GILLESPIE | Thomas D. Gillespie, *Fundamentals of Vehicle Dynamics*, Society of Automotive Engineers. | Vehicle reference frames, equations of motion, ride, handling, braking and loads. | Chapter 1, printed pp. 8–10 / PDF pp. 24–26, read; axis diagram and force/rotation page visually inspected. Primary supplied-book basis for this coordinate change. |
| PACEJKA | Hans B. Pacejka, *Tyre and Vehicle Dynamics*. Edition pending verification in supplied copy. | Tire slip, force/moment, combined-slip and transient models, with explicit dataset sign adapters. | Title page visually inspected. Extracted body text has a broken character mapping; equations must be visually read or OCR-checked before implementation. No tire model added in this change. |
| HAYNES-SUSPENSION | *Competition Car Suspension*, Haynes (as identified by supplied filename). Author/edition pending verification; available scan begins with contents/chapter material. | Suspension layout, spring/damper installation, packaging and practical geometry checks. | Opening chapter pages visually inspected (PDF pp. 3–4, printed pp. 13–14). Image-only scan; cataloged for later focused reading. |

## Using this library for a new module

1. Identify the governing model and read the relevant passages in the supplied references. Record source ID, edition if verified, chapter/section, printed page, and PDF page; distinguish an original derivation from a sourced equation.
2. Declare assumptions, valid operating range, inputs, outputs, and units. Use SI for future dynamic models; convert explicitly at the millimeter geometry boundary.
3. Map source axes and signs to [the project coordinate contract](COORDINATES.md). Declare frame origin, force application points, angle definitions, and action/reaction conventions. Never mix SAE/ISO orientations or tire-load magnitudes with signed components implicitly.
4. Validate against a hand calculation, limiting case, reference example, or measured data as appropriate. Keep implementation regressions separate from physical validation.
5. Track unresolved assumptions in the module documentation. Generated geometry and regression fixtures are not measured or optimized designs.

The first model using these references is the [SAE coordinate and sign contract](COORDINATES.md). The existing rigid-link Newton solver is an implementation choice; it is not presented as a verbatim algorithm from these books. Tire forces, compliance and full-vehicle dynamic integration remain future work.

## U-bar input and static layout reference

For the optional U-bar editor, the following passages were visually inspected in the supplied copies:

- **SEWARD**, Chapter 4, §4.4.1, printed pp. 111–112 / PDF pp. 122–123, Figure 4.11: U-bar layout, chassis bearings, lever joints and links; distinguishes suspension/upright and bell-crank link arrangements.
- **SMITH**, Chapter 6, printed pp. 67–68 / PDF pp. 66–67: rotating chassis mounts, bar and suspension link attachments, and bearing terminology.

These passages support the component names. The four-point, mirrored representation is an app design choice, not a sourced kinematic or stiffness model. User-entered coordinates are SAE axle-local millimeters. The only geometric conversion is reflection about the center plane, `p_right = [X, -Y, Z]`; no force/sign equation from the references is used. Straight segments join the entered points, without estimating bend radii or material properties. The bearing markers do not enforce mounting alignment, and the suspension pickup does not follow a solved body. No ARB dynamics or physical-validation claim is made.

The preview now accepts a known bar bend, lever-arm tip, and suspension pickup while the chassis bearing location remains unentered. This is a rendering completeness choice: bearing coordinates are only needed for their reference markers, not for segments between the other supplied endpoints. It is not a claim that a real bar can function without bearings. Each known point is mirrored and only endpoint-defined segments are drawn. An incomplete XYZ point is omitted rather than assigning zero or constructing a plausible location. No additional engineering model or book equation is adopted.

Verification covers nullable input persistence, malformed-data rejection, compatibility with existing imports, front/rear independence, unchanged suspension solutions when ARB inputs change, missing-bearing/partial preview behavior, and independently checked Euclidean segment lengths and reflection invariants. Synthetic test coordinates are not recommended vehicle geometry.

## Heave interference geometry

The capsule/convex-solid distance checks are not attributed to a supplied vehicle-dynamics book: they apply Euclidean computational geometry to the existing suspension solver's solved hardpoint positions. No force, tire, compliance, spring-rate, damper-rate, or measured-vehicle model is introduced. The assumptions and formulas are recorded in [INTERFERENCE.md](INTERFERENCE.md). Regression tests are software checks against analytic synthetic distances and do not validate clearances on a real vehicle or CAD assembly.

## Tire tread-width rendering

The optional tire tread-width preview uses original Euclidean circle/band geometry, with the user's confirmed tread-width definition. It does not adopt a tire equation from the supplied books. Its assumptions, dimensions, exact bounding construction and numerical cross-checks are recorded in [TIRE-TREAD.md](TIRE-TREAD.md). This is rendering geometry only; the book catalog and these software checks do not establish a measured tire profile or physical packaging validity.

## Local source lookup

The supplied PDFs are in the user's `CR.FSAE/Literature` directory. Match by the book titles above. Do not copy that directory into the app or include it in an upload. On another machine, request the needed source if it is unavailable rather than claiming it was checked.
