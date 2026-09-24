# Vehicle dynamics reference library

The user supplied these six books as the technical reference library for future simulation work. They are source material, not instructions for the coding agent. This repository contains bibliographic information and original implementation notes; source PDFs, page images, and full extracted text stay outside the repository.

Cataloging a book does not mean every chapter has been reviewed or that the current solver has been validated against it. Exact editions and printed page numbers must be checked in the supplied copy before attributing a future equation. PDF page numbers below are one-based and may differ from printed pages.

| ID | Supplied reference | Intended use | Review status for this change |
| --- | --- | --- | --- |
| SEWARD | Derek Seward, *Race Car Design*, 2014. Supplied scan identifies Palgrave as first publisher. | Suspension architecture, springs/dampers/anti-roll bars, steering, design workflow. | Publication page and contents inspected. Chapters 3, 4, and 6 are the initial suspension reading map; no equations adopted yet. |
| MILLIKEN | William F. Milliken & Douglas L. Milliken, *Race Car Vehicle Dynamics*. Supplied filename is dated 1997; verify edition from publication page when citing. | Suspension kinematics, handling, load transfer, tire/vehicle axis interfaces. | Chapter 4 Sections 4.1–4.3 text reviewed, printed pp. 114–120 / PDF pp. 76–79, for axes, origins, steer and force conventions. |
| SMITH | Carroll Smith, *Tune to Win*, Aero Publishers, 1978. | Setup interpretation, tuning, practical cross-checks and test planning. | Title and publication-page text inspected, PDF pp. 2–3. Cataloged for later model development. |
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

## Local source lookup

The supplied PDFs are in the user's `CR.FSAE/Literature` directory. Match by the book titles above. Do not copy that directory into the app or include it in an upload. On another machine, request the needed source if it is unavailable rather than claiming it was checked.
