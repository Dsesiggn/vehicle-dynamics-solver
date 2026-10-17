# Bundled browser libraries

Runtime files are served locally; no package manager or CDN is needed to launch the app.

- Three.js **0.180.0**: three.module.js, three.core.js and MIT LICENSE, downloaded from https://cdn.jsdelivr.net/npm/three@0.180.0/ (build files under build/).
- Plotly.js **3.1.0**: plotly.min.js from https://cdn.jsdelivr.net/npm/plotly.js-dist-min@3.1.0/plotly.min.js and MIT LICENSE from https://cdn.jsdelivr.net/npm/plotly.js@3.1.0/LICENSE.

manifest.json records SHA-256 fingerprints of these exact files. Update a library deliberately, preserve its license, and repeat browser/numerical verification.
