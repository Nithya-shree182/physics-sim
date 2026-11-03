# Dashboard plan (objective O7)

Status: **design only, nothing built.** The user confirms the design before any UI code is
written. Last updated 2026-11-03 (evening).

## Design canvases (claude.ai, private to Nithya's account)

| Canvas | URL | Status |
|---|---|---|
| **Current**: interactive, live-style | https://claude.ai/artifact/9qqX3MFwCc3tSpgwerz76Q | The design to build from. Forward kinematics and confidence in it match Pinocchio to 1e-12 |
| Old multi-tab mockup | https://claude.ai/artifact/CHZBdDn8u4vw44dBbbe1Du | Superseded, left untouched |

Open them from the same claude.ai account. The canvas source files (`Main.dc.html`,
`Chart.dc.html`) lived in a Claude session scratchpad and are not in this repo; if they are
needed, read them back from the canvas itself.

## Decisions (latest wins)

- **Isaac Sim is live in the real product.** No saved or synthetic data in the final UI. The
  canvas runs a local model built from our real equations as a stand-in. The real thing needs
  two links to dev's machine:
  1. a headless Isaac Sim run served as an **HTTP feed** (same pattern as the Physyk AI demos),
     with the simulation time on every frame;
  2. a **control channel back to dev**: mock load, per-joint speed, fault injection.
  **Neither is written into `docs/NOTES_FOR_DEV.md` yet.** Add them when work on the dashboard
  starts, and wait for dev's feed before building the live parts.
- **Permanent tabs:** O2 Dynamics & Residual, O6 Benchmark.
- **Toggle buttons** (not tabs): Live view, O1 Telemetry, O3 Confidence, O4 Payload estimator
  (its own button; O3 and O4 are no longer combined), Model & Tests.
- **O5 Detection is not a tab.** It is alert pop-ups (top right) saying what changed, the
  cause (payload, gravity, friction, speed floor, delay, stuck) and which joint.
- **Mock load** slider and buttons (0 to 3 kg) and **per-joint speed sliders** (5 to 100%),
  both feeding the residual, the payload estimator and the alerts live.
- **50% of the velocity limit is the trusted band** (dev's measurement: above it the simulator
  shows a residual artifact that grows with the physics rate). Above it, residual alarms and
  the load estimate are held and an over-speed notice is shown instead. Each run's
  `speed_fraction` (requested from dev in `NOTES_FOR_DEV.md` Round 4) lets the UI apply the
  band automatically.
- **Run picker and time slider removed** (the view is live). Time scale is fine.
- O6 benchmarks only the run or window the user selects, never everything in a folder.
- Validation-push tab: waits for dev's feed.
- **Theme: Physyk AI's UI theme only** (taken from github.com/Devv-262/Physyk_AI, used for
  look and feel and nothing else): background `#0a0c10`, panels `#111317`, hairline borders
  `rgba(255,255,255,0.12)`, 6 px corners, JetBrains Mono for the interface and numbers, Inter for
  explanatory text, accent `#60a5fa`.
- **Colours, one meaning each on every view** (checked with the colour-blind validator on
  `#111317`): measured / Isaac Sim = light grey `#e5e7eb`; Pinocchio `#3987e5`; gravity
  `#199e70`; inertia `#d95926`; coupling `#9085e9`; friction `#c98500`. Status colours (amber
  threshold, red alarm, green pass) always come with an icon and a word.
- Torque breakdown is a share bar plus one strip per term, **not** a stacked area (inertia and
  friction swing negative, which a stack would misdraw).

## Build order the user set

1. Real captured runs from dev (in progress; two slow-reference runs are in).
2. Finish the remaining analysis work on real data (fast and fault runs, detector re-tune).
3. Dashboard last, once the design is confirmed and dev's HTTP feed exists.

Do not change the canvas or build the UI until the user explicitly confirms the design.

## Honesty rules to carry into the build

- Label any synthetic or mock-driven numbers as such; synthetic results are optimistic because
  the plant is the same URDF as the twin.
- Say that the URDF parameters make twin and plant agree and are not the real Panda's.
- Detector thresholds were tuned on synthetic data; show "re-tune on Isaac Sim logs" until
  they are re-tuned.
