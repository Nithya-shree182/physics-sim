# Dashboard plan (objective O7)

> **2026-11-05:** this was the plan. The dashboard is built and was redesigned; what it does now is in `dashboard/README.md` and `DECISIONS.md` D9. The confidence panel and the over-speed state in this plan no longer exist.

Status: **first version built** (`dashboard/`, Plotly Dash; see `dashboard/README.md`), running on a
stand-in plant and on replays of dev's real logs. The live Isaac Sim parts wait for dev's feed. The
interactive design canvas below is the reference. Last updated 2026-11-03 (23:30 IST).

## Design canvases (claude.ai, private to Nithya's account)

| Canvas | URL | Status |
|---|---|---|
| **Final design** (version 3): interactive, live-style | https://claude.ai/artifact/9qqX3MFwCc3tSpgwerz76Q | The design to build from. Its forward kinematics and confidence match Pinocchio to 1e-12 |
| Old multi-tab mockup | https://claude.ai/artifact/CHZBdDn8u4vw44dBbbe1Du | Superseded, left untouched |

Open them from the same claude.ai account. The canvas source files (`Main.dc.html`,
`Chart.dc.html`) lived in a Claude session scratchpad and are not in this repo; read them back
from the canvas itself if needed.

Everything on the canvas runs on a local stand-in model built from our real equations, not on
Isaac Sim. It is labelled as such on the page.

## Layout of the final design

One page, one question per area:

1. **Header**: brand, live/source pill, simulation time, Pause, **Demo**, and a bell with the
   active-alert count.
2. **Two permanent tabs**: *O2 Arm & residual* and *O6 Benchmark*.
3. **Panels** (buttons beside the tabs) open a **right-hand drawer**, one at a time:
   O1 Telemetry, O3 Confidence, O4 Payload, About (model, tests, honest limits). The bell opens
   the same drawer with the alert list and recent events.
4. **Arm twin is the main screen** (O2 tab):
   - links tinted by each joint's share of its torque limit;
   - force arrow at the gripper for the estimated external force;
   - dashed ellipse at the hand showing how freely it can move (flat = near singular, amber
     when the detector is gated);
   - rings and labels on joints with an active alert;
   - a switch between the twin and the Isaac Sim stream placeholder;
   - three stat cards beside it: payload estimate with band, pose confidence, worst residual.
5. **One controls row**: mock load (slider, minus and plus, 0 to 3 kg) and an all-joint speed
   slider. "More controls" opens per-joint speeds (5 to 100% of the velocity limit), tare, and
   the test-only fault injector (friction x3, stuck joint, 50 ms command delay).
6. **One chart** with a channel switch (Residual, Torque, Confidence, Payload), below it a row of
   seven residual bars (% of torque limit) that doubles as the joint selector, and a collapsed
   torque breakdown for the selected joint.
7. **O6 tab**: the trajectory comparison (three styles, cycle time, peak torque, energy per
   pick), a placeholder until dev supplies the runs; plus an *Evaluation of this live session*
   block (payload error, detection delays, false alarms, the residual floor against speed, the
   real empty-arm log numbers).
8. **Alerts are pop-ups, not a tab.** They appear top right when raised, say which joint, the
   cause and what changed (residual before and after), and minimise to the bell after about 8 s.

## Guided demo (the O7 definition of done)

The Demo button steps through the proposal's five-step scenario by driving the load, speed and
fault controls and opening the matching drawer. Next, Back and Stop are always available.

| Step | What happens | Status |
|---|---|---|
| 1 Pick up an unknown object | 1.2 kg appears after 3 s; the estimate converges | works on the stand-in |
| 2 Near-singular pose | elbow stretches; confidence dips, gate closes, estimate is held | works on the stand-in |
| 3 Tampered waypoint | **stand-in:** 50 ms command delay. The residual stays quiet and the tracking error raises the alert | needs dev's attack runs for the real thing |
| 4 Friction fault | friction x3 on joint 3, identified by its torque signature | works on the stand-in |
| 5 Compare trajectory styles | opens the O6 tab | **waits for dev's three trajectory styles** |

## Decisions (latest wins)

- **Isaac Sim is live in the real product.** No saved or synthetic data in the final UI. The
  real thing needs two links to dev's machine:
  1. a headless Isaac Sim run served as an **HTTP feed** (same pattern as the Physyk AI demos),
     with the simulation time on every frame;
  2. a **control channel back to dev**: mock load, per-joint speed, fault injection.
  Round 4 of `docs/NOTES_FOR_DEV.md` mentions the control channel as a heads-up; the feed itself
  is still dev's to define. Wait for it before building the live parts.
- **50% of the velocity limit is the trusted band** (dev's measurement; confirmed for O2, O4 and
  O5 in Round 4). (Superseded by `DECISIONS.md` D1: the dashboard's speed bars now stop at the band,
  so the over-speed state no longer exists.) `speed_fraction` in the manifest still records the speed of every run.
- O3 and O4 are separate panels. O5 Detection is **not** a view: alerts only.
- Time scale is fine. No run picker or time slider (the view is live).
- **Theme: Physyk AI's UI theme only** (github.com/Devv-262/Physyk_AI, look and feel and nothing
  else): background `#0a0c10`, panels `#111317`, hairline borders `rgba(255,255,255,0.12)`,
  6 px corners, JetBrains Mono for the interface and numbers, Inter for explanatory text,
  accent `#60a5fa`.
- **Colours, one meaning each everywhere**: measured / Isaac Sim = light grey `#e5e7eb`;
  Pinocchio `#3987e5`; gravity `#199e70`; inertia `#d95926`; coupling `#9085e9`; friction
  `#c98500`; unmodelled load `#6ee7b7`. Status colours (amber threshold, red alarm, green pass)
  always come with a word.
- The torque breakdown is a signed bar per term, **not** a stacked area (inertia and friction
  swing negative, which a stack would misdraw).

## Not in the design yet (proposal items waiting on dev or on later work)

- Validation push (applied vs estimated force): waits for dev's feed and torque-mode run.
- Tamper alert: needs dev's attack injectors, plus our separate geometric check (a tampered goal
  gives no residual).
- In-simulator `omni.ui` panel and debug-draw overlays: dev's side, last.
- Link tint and force arrows are on the web twin only; the in-sim overlay is separate work.

## Build order the user set

1. Real captured runs from dev (two slow-reference runs are in; the rest requested in
   `NOTES_FOR_DEV.md` Round 4).
2. Finish the remaining analysis work on real data (fast and fault runs, detector re-tune).
3. Dashboard last, once dev's HTTP feed exists. Implementation target: Plotly Dash (per
   `WORK_SPLIT.md`) or a small web front end on the same feed; decide when the feed is defined.

## Honesty rules to carry into the build

- Label any synthetic or mock-driven numbers as such; synthetic results are optimistic because
  the plant is the same URDF as the twin.
- Say that the URDF parameters make twin and plant agree and are not the real Panda's.
- Detector thresholds were tuned on synthetic data; show "re-tune on Isaac Sim logs" until they
  are re-tuned.
- The stand-in's floor model is calibrated from dev's sweep and the first real log, not
  measured on this arm; do not quote its numbers as results.
