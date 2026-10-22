# Browser-only Suspension Studio

The configured service is live:

- [Open Suspension Studio](https://vehicle-dynamics-studio.onrender.com/suspension-studio/)
- [Open Steering Studio](https://vehicle-dynamics-studio.onrender.com/suspension-studio/steering.html)
- [Deployed version and health](https://vehicle-dynamics-studio.onrender.com/api/v1/health)

The owner's existing deployment is managed in the
[Render service dashboard](https://dashboard.render.com/web/srv-dbaoro61egvs73bi1pg0)
and [Suspension Studio Blueprint](https://dashboard.render.com/blueprint/exs-dbaorebbc2fs73c7d760).
Use those existing resources for updates; the setup section below is for creating
a separate installation, not a step required on each visiting computer.

Run the complete studio on a Python web host, then open its HTTPS address on any
computer with a compatible browser. The visiting computer needs no Python, terminal,
software installation, or administrator privileges. Its network must allow access to
the site's domain, and its browser must allow JavaScript and WebGL for the 3D view.
An organization's network/browser restrictions still need to be checked on that computer.

The hosted service serves both the existing frontend and Python API at the same origin.
Steering requests use the existing fixed-axis 3D solver, including the inclined-axis
geometry and 5-degree sweep. Hosting does not change the mathematical model. The
method and applicability limits remain in [STEERING_BACKEND.md](STEERING_BACKEND.md)
and [the steering method notes](../suspension-studio/docs/STEERING.md).

## Set up a separate Render installation

The repository includes [render.yaml](../render.yaml) for a single **Free Python web
service**. No database, disk, paid instance, or custom domain is required.

1. Create/sign in to a [Render account](https://dashboard.render.com/). Review the
   account terms yourself. Connect the GitHub account that owns this repository and
   grant access to `Dsesiggn/vehicle-dynamics-solver`.
2. Open [the deployment template](https://render.com/deploy?repo=https%3A%2F%2Fgithub.com%2FDsesiggn%2Fvehicle-dynamics-solver%2Ftree%2Fcodex%2Fhybrid-engineering-backend).
   Alternatively, create a Blueprint from the connected repository, select branch
   **`codex/hybrid-engineering-backend`**, and use **`render.yaml`** at its root.
3. Verify that the proposed resource is one web service on the **Free** plan. The
   template runs Python 3.14.5, installs `backend/requirements.txt`, starts
   `python run_hybrid.py --host 0.0.0.0 --port $PORT`, and checks `/api/v1/health`.
4. Deploy it and wait for **Live**. Render supplies the actual HTTPS hostname;
   the service name does not guarantee that any particular hostname is available.
5. From that hostname, open **`/suspension-studio/`** for suspension and
   **`/suspension-studio/steering.html`** for steering. Bookmark those addresses on
   the other computer. Keep using this hosted address; `127.0.0.1` refers to the
   computer on which the browser is running.

Use the **connected GitHub repository**, not Render's unauthenticated **Public Git
Repository** option: Render documents that the latter cannot automatically deploy
updates. The template selects **After CI Checks Pass**. The `Studio checks` GitHub
workflow runs Python/API/parity tests and the Studio JavaScript tests, then starts
the real service and checks its HTTP routes. Check that GitHub Actions is enabled
and the workflow completes successfully for the branch before relying on auto-deploy.

If the service was created from the public template before GitHub was connected,
its initial deployment can work while later commits do not deploy. In the existing
service's **Settings → Source → Edit**, use **Git Provider**, connect Render's GitHub
app with access limited to this repository, and select the repository there. Confirm
**`codex/hybrid-engineering-backend`** again: the source-change form can default to
`main`. Keep the Python build/start commands above. The configured service's GitHub
connection was completed with the repository owner's approval on 2026-10-18.

## Updates and version verification

The current full application is on `codex/hybrid-engineering-backend`; the repository's
default branch and GitHub Pages site must not be assumed to contain that code.
Publish future app changes to the linked branch. Once CI passes and Render reports
the new deployment **Live**, reload the studio in each browser. Already-open pages
keep their loaded code until reloaded; deployment does not silently reload an editing
session. A failed build/check leaves the prior deployment in place.

Open **`/api/v1/health`** on the hosted address. Its **`source_commit`** must match
the commit shown on the linked GitHub branch and Render deployment. Render supplies
that SHA through `RENDER_GIT_COMMIT`; another host can set `APP_COMMIT_SHA` to the
deployed commit. An unset or invalid revision is reported as `null`, not guessed.
Mutable application HTML/modules and API responses use `Cache-Control: no-store`.
This version check verifies which source is deployed; it is not proof of physical
vehicle-model accuracy.

If the application is later merged and deployment moves to another branch, update
the Render service, Blueprint branch, this link/documentation, and the workflow's
push branch together. A stable URL updates only from its configured branch, not
from arbitrary changes elsewhere in GitHub or uncommitted files on a computer.

## Saved designs and access

Design libraries remain in each browser's storage. The hosted site does not add
cloud synchronization or a shared design database. Export suspension and steering
designs as JSON on the old computer/address and import them on the new one. Save
exports separately if the managed browser clears its storage. The Python service
receives steering geometry for calculation but does not persist it as a library.

The service is publicly accessible to anyone with the URL and has no user-account
access restriction. It serves only the selected application assets and API; local
reference PDFs and project files are not deployed as downloadable assets. Hosting
your own saved vehicle files publicly is not part of this deployment.

## Free plan and verification limits

Render documents that Free web services sleep after 15 minutes without inbound
traffic and take about a minute to wake. They have monthly usage limits and are
intended for previews/personal use; this is not a guarantee of continuous availability.
Review the account's current usage/billing settings before changing any plan.

An already-open tab's first calculation after the server sleeps may exceed the
client's 15-second request timeout or receive Render's loading page. Wait about a
minute, then select **Retry calculation** in steering (or **Calculate reference**
in the workbench). Inputs remain in the editor; stale calculated results are cleared.
This does not require installing Python on the visiting computer.

A deployment is complete only after the actual HTTPS site passes health, suspension
startup, steering calculation, 3D viewer, and reload/version checks. A local test or
a successful GitHub commit alone does not prove a remote deployment is live. Finally,
open that same URL on the managed computer to check its network and graphics policy.

Provider references reviewed 2026-10-18:
[FastAPI deployment](https://render.com/docs/deploy-fastapi),
[Blueprint fields](https://render.com/docs/blueprint-spec),
[GitHub automatic deployment](https://render.com/docs/deploys),
[deployment button and branch selection](https://render.com/docs/deploy-to-render),
[Python version](https://render.com/docs/python-version),
[commit environment variables](https://render.com/docs/environment-variables), and
[Free service limitations](https://render.com/docs/free).
