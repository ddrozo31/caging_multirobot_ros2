# Methodology: From a Paper to a Simulator, a Course and a Lecture

A reusable working method for research and teaching projects in robotics, distilled from this
project: the implementation in ROS 2 / Gazebo of Tejada et al. (2025)
([doi:10.1007/s10846-025-02263-y](https://doi.org/10.1007/s10846-025-02263-y)), its documentation
as a 13-step course ([steps/](steps/)), a Docker image for students ([../docker/](../docker/)),
and a lecture (in preparation).

**For whom.** A professor or researcher who leads a similar project, students who want to see
how a project of this kind is organized, and any AI assistant that works with them. Copy this
file into a new project and adapt Section 16.

**About this repository.** The project was developed in a private working repository; this
public repository is generated from it, with one commit and one tag per step. The working files
that the method describes (the rules file `CLAUDE.md`, the decision log `MEMORY.md`, the step
branches) are therefore **not** included here; Sections 4 and 17 show their
structure so you can create your own.

**Author:** David Rozo-Osorio, I.M., M.Sc., Universidad EIA, Envigado, Colombia.
Work done by the author with AI assistance. **Version:** 20261104.

---

## Contents

1. [Principles](#1-principles)
2. [Life cycle of a project](#2-life-cycle-of-a-project)
3. [Phase 0 — Kickoff](#3-phase-0--kickoff)
4. [The project memory: CLAUDE.md and MEMORY.md](#4-the-project-memory-claudemd-and-memorymd)
5. [Repository and ROS 2 structure](#5-repository-and-ros-2-structure)
6. [Phase 1 — Development step by step](#6-phase-1--development-step-by-step)
7. [Git conventions](#7-git-conventions)
8. [Verification](#8-verification)
9. [Documentation of each step](#9-documentation-of-each-step)
10. [Phase 2 — Closing the reproduction](#10-phase-2--closing-the-reproduction)
11. [Phase 3 — Freeze](#11-phase-3--freeze)
12. [Phase 4 — Lecture and workshop material](#12-phase-4--lecture-and-workshop-material)
13. [Phase 5 — Packaging and sharing](#13-phase-5--packaging-and-sharing)
14. [Working with an AI assistant](#14-working-with-an-ai-assistant)
15. [Working on several computers](#15-working-on-several-computers)
16. [Starting a new project with this method](#16-starting-a-new-project-with-this-method)
17. [Catalogue of pitfalls](#17-catalogue-of-pitfalls)
18. [Templates](#18-templates)

---

## 1. Principles

| # | Principle | In practice |
|---|---|---|
| 1 | **Step by step** | One small, reviewable step at a time. Never build ahead of the agreed step. |
| 2 | **The human approves** | Every step ends at a review gate: the professor runs it and explicitly approves before it is merged. |
| 3 | **Every decision is written** | Choices, simplifications and their reasons go to a decision log with an ID (D-XXX). A simulation without written decisions is not a reproduction. |
| 4 | **Teaching code, not production code** | Simple, linear, fully commented code; no clever abstractions. A student must follow it line by line. |
| 5 | **Link math to code** | Every implemented equation cites its source: `# Eq. (13) - approach pose of the leader`. |
| 6 | **Separate math from plumbing** | Algorithms in pure Python (`core/`, no ROS imports, unit-tested); ROS nodes only subscribe, call `core/` and publish. |
| 7 | **One source of truth** | A value written twice will diverge. Keep it in one place and let everything read it (robot dimensions in the URDF, timing in one agenda table, colors in one file). |
| 8 | **Measure, do not assume** | Every step states an expected result with numbers and is verified against it. Report deviations honestly. |
| 9 | **Nothing only on one computer** | Commit, push right away, keep a handoff note for the next session. |
| 10 | **Freeze when done** | Mark finished parts with a tag and stop changing them unless explicitly reopened. |

---

## 2. Life cycle of a project

```
Phase 0  Kickoff ───────── read the source, extract reference data, decide stack and conventions,
                           create CLAUDE.md + MEMORY.md, propose and approve a roadmap
Phase 1  Development ───── steps 0..N, one branch per step, review gate, merge + tag
Phase 2  Closing ───────── gap analysis vs. the source, scope decisions, repeatability,
                           quantitative claims, cleanup of duplications
Phase 3  Freeze ────────── tag v1.0 on the finished, verified simulator; no more changes
Phase 4  Teaching ──────── lecture slides + companion notes, built block by block
Phase 5  Sharing ───────── container image for students, public repository generated
                           from the private one
         (continuous) ──── decision log, session log, handoffs between computers
```

Typical size of this project: 13 development steps, about 70 decisions, 14 open questions (all
closed), 5 ROS 2 packages, 32 unit tests, one Docker image.

---

## 3. Phase 0 — Kickoff

### 3.1 Read the source and extract reference data

Create `docs/paper_reference.md` (here: [paper_reference.md](paper_reference.md)) with:

- every equation, numbered as in the source, in LaTeX;
- every parameter table (e.g. Table 1), initial conditions, trajectories;
- hardware data (robot dimensions from drawings, with the **derivation** of each value);
- **suspected typos or ambiguities**, copied *as printed* with a note on the interpretation used.

Render the source pages to images to read equations when text extraction garbles them.
Never copy the publisher's figures or third-party drawings into your material; redraw them
(e.g. [figures/rosmaster_x3_dimensions.tex](figures/rosmaster_x3_dimensions.tex)) and cite the
source.

### 3.2 Decide the stack and the conventions — by asking

Before writing code, ask the professor a short list of decisions, each with options and a
recommendation. In this project:

| Question | Answer |
|---|---|
| Language of docs, comments, commits | English |
| Main language / where C++ is allowed | Python; C++ only for the ros2_control controller |
| How actuation is simulated | ros2_control with our own controller |
| How sensing is simulated | Gazebo ground truth as "virtual Vicon" |
| Documentation layout | one Markdown guide per step + course index in the README |
| Date format | `YYYYMMDD` |
| Commit message format | `step-XX: <summary>` (later `step-XX.n`) |

Record every answer as a decision (D-001, D-002, …) and every unanswered item as an open
question (Q-001, …).

### 3.3 Propose a roadmap and get it approved

A table, one row per step, each step = one lecture/lab = one branch:

| # | Step | Source reference | Status |
|---|---|---|---|
| 0 | Workspace setup | – | Not started |
| 1 | Robot model in the simulator | §2.1.1 | Not started |
| … | … | … | … |

The roadmap can change (steps were added here: 11 repeatability, 12 single source), but every
change is recorded.

### 3.4 Create the project memory

`CLAUDE.md` (rules) and `MEMORY.md` (state and history) at the repository root before the first
commit (Section 4). Step 0 then sets up the workspace and the course index, and is the first
merged step.

---

## 4. The project memory: CLAUDE.md and MEMORY.md

Two files make the project resumable by any person or assistant, on any computer, at any time.
They live in the working repository (in this project: the private one); skeletons in Section 18.

### 4.1 CLAUDE.md — the rules

Read automatically by the assistant at the start of every session.

1. **Resume note** at the top: current state in 4–6 lines, where the latest handoff is.
2. **Project purpose**: source, goal, audience.
3. **Working rules** (numbered, "must follow").
4. **Git workflow** (Section 7).
5. **Technical stack** (fixed choices — "do not propose alternatives").
6. **Repository layout**.
7. **Known pitfalls** (Section 17), added each time a mistake is found.
8. **Documentation conventions**.

### 4.2 MEMORY.md — the traceable state

| Section | Content | Rules |
|---|---|---|
| 1. Decision log | `\| D-XXX \| date \| decision \| rationale / source \|` | Never delete: mark `Superseded by D-YYY`. The source says *who* decided ("user answer", "Step 3 design"). |
| 2. Open questions | `\| Q-XXX \| topic \| options \| status \|` | When answered: `Closed <date> → D-YYY`. |
| 3. Roadmap status | the roadmap table with status per step | `Not started` → `In review on branch step-XX` → `Done <date> (approved, merged, tag)`. |
| 4. Resume here (handoff) | state at pause, checklist for the next session, options for next work, environment notes | Rewritten at every pause. |
| 5. Reference data | pointer to `docs/paper_reference.md` | |
| 6. Session log | one row per work block: what was done, **what was verified (with numbers)**, what is pending | Append only. |

Rules: update MEMORY.md **before** committing (so the commit carries it); write dates as
`YYYYMMDD`; quote the user's words for approvals ("approved", "commit everything").

A separate line of work that will never be merged (here: the Docker packaging) keeps its **own**
memory file with its own ID prefix (e.g. `DK-001`), so the main log stays clean.

---

## 5. Repository and ROS 2 structure

### 5.1 Layout

```
<workspace>/
├── CLAUDE.md, MEMORY.md, README.md (course index + results)
├── docs/
│   ├── paper_reference.md, methodology.md, figures/
│   ├── steps/step_XX_<name>.md      one guide per step (+ companions for hard topics)
│   └── lecture/                     slides + notes (Phase 4)
├── docker/                          container image and student kit (Phase 5)
└── src/                             colcon workspace
```

### 5.2 Packages by layer: `<project>_<role>`

Prefix first (ROS convention, packages group together in listings and tab completion).
A package is created only when the step that needs it is reached.

| Package | Build type | Role |
|---|---|---|
| `<p>_description` | ament_python | xacro model (single source of dimensions), ros2_control tag, Gazebo plugins, RViz configs |
| `<p>_gazebo` | ament_python | worlds, **one** bridge YAML (`config/topic_bridge.yaml`), spawn launch files |
| `<p>_<x>_controller` | ament_cmake (C++) | custom ros2_control controller plugin (the only C++) |
| `<p>_control` | ament_python | `core/` pure math + unit tests, `nodes/` thin ROS wrappers, `tools/` offline analysis |
| `<p>_bringup` | ament_python | launch files per step/experiment, parameter YAMLs (source tables), batch scripts |

In this repository: [../src/](../src/) with the prefix `caging_`.

### 5.3 Conventions that paid off

- **`core/` vs `nodes/`**: math testable with `pytest` without ROS or the simulator.
- **One bridge YAML** listing every ROS ↔ Gazebo topic, absolute topic names (`/clock`).
- **Namespaces** for several robots (`/leader`, `/follower`); parameter YAMLs keyed with `/**/node:`.
- **Ground truth vs. measurement**: controllers may read a noisy "virtual sensor", the evaluation
  always reads the ground truth.
- **Features off by default**: new options (e.g. sensor noise) default to "off", so earlier steps
  behave exactly as before.
- **Launch arguments** declared at the top launch file are inherited by included files.
- **Teaching code**: comments explain *why*; Python equivalents in comments of C++ code.
- **Logging**: one CSV per experiment run + an offline plotting/table tool; a batch script that runs
  every configuration headless and prints a summary.

---

## 6. Phase 1 — Development step by step

### 6.1 The step loop

```
1. Propose   plan of the step + the decisions needed (options + recommendation) → user answers
2. Branch    git checkout main && git checkout -b step-XX
3. Build     implement the smallest thing that does the job (teaching code)
4. Verify    colcon build, unit tests, headless run with expected NUMBERS (Section 8)
5. Document  docs/steps/step_XX_<name>.md, README index, MEMORY.md (decisions + session log)
6. Review    the user runs the GUIs; changes → more sub-steps step-XX.n
7. Approve   explicit "approved" from the user
8. Merge     git merge --no-ff step-XX; annotated tag step-XX; push main + branch + tag
```

### 6.2 Review gate (a step is finished only when)

- `colcon build` succeeds and the automated checks pass;
- the user has run the launch files with the GUIs and approved the behavior;
- the step guide, README index and MEMORY.md are up to date;
- the user **explicitly** approves.

### 6.3 Companions for hard topics

When a step relies on knowledge the audience may lack, add a companion guide
([step_02_controller_walkthrough.md](steps/step_02_controller_walkthrough.md) for
C++/ros2_control, [step_04_lqr_math.md](steps/step_04_lqr_math.md) for the LQR math):
concepts with Python equivalents, line-by-line code reading, worked examples, self-check questions.

### 6.4 Changes to finished work

Fixes to a merged step go on the current branch as `step-XX.n: fix …`; earlier guides get short
"*Since Step N …*" notes instead of rewrites, so each tag still matches its documentation.

---

## 7. Git conventions

| Situation | Branch | Commit message |
|---|---|---|
| Sub-step of a step | `step-XX` | `step-XX.n: <summary>` + body (what and why) |
| Merge of an approved step | `main` | `step-XX: <summary> (approved)` |
| Lecture work | `lecture-XX` | `lecture-XX.n: …`, merge `lecture-XX: …` |
| A line of work never merged (e.g. Docker) | its own branch | `<branch>.n: …` |
| Maintenance, no code | `main` | `maint: <summary>` |
| Notes for the next session | `main` | `handoff: <summary>` |

Rules:

- One branch per step; `git log --first-parent main` shows one commit per step.
- Every commit includes **all** changed and new files (`git add -A`, respecting `.gitignore`).
- **Push right after every commit.** When a branch and a tag share a name, push with full refs:
  `git push origin main refs/heads/step-XX refs/tags/step-XX`.
- Annotated tags on merge commits (`step-XX`, `v1.0`, `lecture-XX`); **never move a pushed
  tag** (create a new one, e.g. `v1.1`).
- Merge messages from a file (`git merge --no-ff -F msg.txt`), chained with `&&` and checked before
  tagging.
- Build artifacts (`build/`, `install/`, `log/`, LaTeX `*.aux/*.bbl/…`, results CSVs) in `.gitignore`;
  keep final PDFs and small per-run summaries.
- A freeze tag (`v1.0`) marks finished work; a rule in CLAUDE.md forbids changes until reopened.

---

## 8. Verification

### 8.1 What to check in every step

| Level | How |
|---|---|
| Build | `colcon build` with no warnings |
| Math | unit tests of `core/` (`colcon test`), hand-computed examples |
| Behavior | headless simulation with exact commands and **expected numbers** (e.g. 0.2 m/s for 5 s → Δx = 1.000 m) |
| Integration | the whole task end to end; key metric compared with the previous result |
| Human | the professor with the GUIs (review gate) |

### 8.2 Headless test recipe (ROS 2 + Gazebo)

- Unique `ROS_DOMAIN_ID` and `GZ_PARTITION` per run, or two simulations share topics.
- Gazebo server only: pass `-s` in the world argument; no RViz.
- Start each launch with `setsid … &` and stop it with `kill -- -<PGID>`.
- Wait for readiness by polling (e.g. `ros2 control list_controllers` shows `active`), never fixed long sleeps.
- Drive with a small script that publishes for an exact **simulated** time.
- Put test scripts in a scratch folder, not in the repository; never kill processes you did not start.

### 8.3 Reporting

- Give the numbers ("Δx = 1.000 m, Δθ = 1.500 rad"), what was **not** verified ("RViz not opened"),
  and any anomaly with its explanation (e.g. a start-up transient vs. a real error).
- When a claim cannot be tested (no joystick on the machine), say so.

---

## 9. Documentation of each step

Every guide `docs/steps/step_XX_<name>.md` has the same sections:

1. **Objective** — what the student will have at the end.
2. **Paper reference** — sections and equations covered.
3. **Theory** — concepts, diagrams (ASCII or images), equations ↔ code.
4. **Files created / changed** — a tree with one comment per file.
5. **How to build and run** — exact commands.
6. **Expected result** — measured numbers, tables, figures.
7. **Exercises** — 5–8 questions, from "predict and measure" to "modify the code".

The README holds the course index (one row per step with a link), the results and the scope
(what is reproduced, what is left as exercise).

---

## 10. Phase 2 — Closing the reproduction

1. **Gap analysis.** Item-by-item table of what the source reports vs. what is reproduced and verified:
   reproduced + verified / implemented but untested / not done / deliberately simplified, with
   evidence (file, numbers).
2. **Scope decision by the professor.** Which gaps are closed and which become student exercises
   (record as a decision; make sure the exercises exist in the guides).
3. **Repeatability.** A deterministic simulation repeats exactly; add a realistic noise source with
   a seed, run each configuration n times, report mean ± std
   ([step_11_repeatability.md](steps/step_11_repeatability.md)).
4. **Turn qualitative claims into numbers** (e.g. "the object stays caged" → max distance from the
   cage center, pass/fail against a limit, k/n runs).
5. **Ambiguities of the source** found while implementing: list them explicitly (they are results).
6. **Remove duplications** (single source of truth) before freezing
   ([step_12_single_source.md](steps/step_12_single_source.md)).

---

## 11. Phase 3 — Freeze

`maint:` commit with a CLAUDE.md rule ("`src/` is frozen"), annotated tag `v1.0`.
Teaching material and the container image are built from the frozen tag, so they never go stale.
Changes after the freeze need the professor to reopen the frozen part explicitly.

---

## 12. Phase 4 — Lecture and workshop material

### 12.1 Decisions asked first

Language, format (lecture + live demos), depth of the companion notes, visual style — each with
options and a recommendation. Typography was chosen by **comparing rendered samples** (same page in
three fonts).

### 12.2 Structure

```
docs/lecture/
├── outline.md          storyline, learning goals, timing, demos, derivations to decide [D?]
├── common/             colors, notation macros, code style, boxes, agenda table, TikZ shapes
├── slides/             Beamer (metropolis, 16:9)
├── notes/              report class, one chapter per block, bibliography
├── figures/            own TikZ diagrams (to scale, in meters)
└── build.sh            builds both PDFs
```

### 12.3 Roadmap

`L-00` setup + outline (approved before content) → one item per block (slides **and** the matching
notes chapter together) → backup slides and appendices → rehearsal (timing, consistency, handout,
demo checklist) → tag `lecture-v1`.

### 12.4 Style rules

- **Slide titles: 1–3 words, a noun phrase** (*Motivation*, *Learning goals*, *Task stages*); the
  message goes in the body or a *Key idea* box.
- **Notes headings may be longer**, but each starts with a gray line naming the slide(s) it expands
  (`\slides{Task stages}`) — titles, not numbers.
- One color meaning everywhere (leader red, follower blue, object orange), shared by all figures.
- Shared notation macros and **one agenda table** used by slides and notes.
- Boxes with fixed meaning: *Key idea*, *Try it*, *Exercise*, *In the paper*.
- Own diagrams in TikZ (to scale), never copied figures from the source.
- Each live demo has a fallback (animation from logged data).

---

## 13. Phase 5 — Packaging and sharing

### 13.1 A container image for students

Students should run the frozen simulator without installing ROS 2
([../docker/README.md](../docker/README.md)).

- Build from the **frozen tag** (`v1.0`); the image never contains work in progress.
- One Compose file with one service per use: tests, headless experiments, simulation, GUI on
  Linux, GUI with an NVIDIA GPU, GUI on Windows (WSLg), development shell.
- Verify every service on every kind of machine the students have (Linux, Linux + NVIDIA,
  Windows), and publish the **minimum requirements** that came out of those tests (CPU
  architecture, cores, memory, OpenGL, disk).
- A short student guide and a kit (only the Compose file and a README) separate from the
  developer documentation.

### 13.2 A public repository generated from the private one

- The working repository stays private (decision log, work in progress, other lines of research).
- The public repository is **generated** by a script that copies an **allow-list** of files
  (never "everything except"), then scans every commit for excluded paths.
- **Clean history**: one commit and one annotated tag per step (`step-00` … `step-12`), so
  `git checkout step-05` shows the project after Step 5 and `git diff step-04 step-05 -- src`
  shows what Step 5 added; the guides in every commit are the cleaned final versions.
- Clean the texts for outside readers: internal decision IDs replaced by the reasons, private
  paths and machine names removed, third-party images redrawn.
- Separate licenses for code (e.g. Apache-2.0) and documents (e.g. CC BY 4.0), a `CITATION.cff`
  that also cites the reproduced paper, and credits that state the use of AI assistance.
- Once pushed, the public history is never rewritten; later material arrives as new commits.

---

## 14. Working with an AI assistant

### 14.1 Division of work

| The professor | The assistant |
|---|---|
| decides scope, stack, style, priorities | proposes options with a recommendation |
| runs the GUIs and approves steps | implements, verifies headless, documents |
| decides authorship and disclosure matters | records every decision and its source |
| says when to commit / merge / publish | commits, merges, tags and pushes as instructed |

### 14.2 Protocol

- The assistant asks **before** coding when a decision changes the result; uses defaults (and says
  so) when it does not.
- Ambiguous instructions are clarified with a short multiple-choice question (e.g. "Item 2 only"
  vs. "Both items").
- Long question lists can be answered in a text file in the repository, which the assistant reads
  and records as decisions.
- Reports end with what was verified, what was not, and the next decision needed.
- No destructive or outward action (delete, push, publish, move tags) without the user's
  instruction or a standing rule; deletions are recoverable from git and the user is told how.
- Findings from helpers (sub-agents) are checked before being reported as facts.

### 14.3 Standing authorizations used here

Push every commit right away; include all files in every commit; work only on the requested step.

---

## 15. Working on several computers

- Every computer pushes over SSH (`git@github.com:…`); check the key fingerprint against the
  GitHub account list when in doubt.
- At every pause: commit, push, write the **handoff** (MEMORY.md §4 + CLAUDE.md resume note) with
  a checklist for the next machine: sync (`git fetch --tags`, `merge --ff-only`), branches to check
  out, refs that exist only locally, rebuild/test commands, things that live only on one machine
  (large bags, raw results).
- If the handoff lives on a work-in-progress branch, leave a short `handoff:` pointer on `main`
  and merge `main` back into the branch to keep the later merge clean.
- Check tool availability on the other machine (ROS dependencies, LaTeX packages, Docker, GPU
  drivers) as part of the checklist; never assume `sudo`.
- Before pushing, fetch: another computer may have pushed to the same branch in the meantime.

---

## 16. Starting a new project with this method

1. Copy `methodology.md` into the new repository.
2. Create `CLAUDE.md` and `MEMORY.md` from the templates (Section 18); fill the purpose and the stack.
3. Extract the reference data of the source into `docs/<source>_reference.md`.
4. Ask the kickoff questions (Section 3.2); record D-001…
5. Propose the roadmap; get it approved; Step 0 = workspace setup + course index.
6. Adapt the package layout (Section 5.2): keep the prefix rule, the `core/`/`nodes/` split, the
   single bridge YAML and the bringup package.
7. Follow the step loop (Section 6) and the git conventions (Section 7) from the first commit.

To give an assistant the method in a new session: "Read methodology.md and follow it; start with
Phase 0."

---

## 17. Catalogue of pitfalls

| Area | Pitfall | Remedy |
|---|---|---|
| XML | `--` inside an XML comment makes `package.xml`/xacro/SDF invalid; colcon then treats the package as plain Python ("Package not found") | never write `--` in XML comments; validate XML files |
| URDF | `<surface><friction>` inside a URDF `<collision>` is silently dropped | Gazebo settings in `<gazebo reference="link">` blocks |
| Gazebo | DART ignores soft-contact `kp`/`kd` | model compliance with spring joints |
| ROS 2 params | a plain YAML key is ignored by a node inside a namespace | key `/**/node_name:` |
| ROS 2 | `rclpy.Node` has read-only properties (`publishers`, `services`, …) | do not reuse those attribute names |
| colcon | edited files under `install/` are not overwritten by a rebuild | edit `src/`, rebuild (or `--symlink-install`) |
| colcon | override warning when the workspace's own `install/` is sourced | harmless; use a fresh shell |
| Tests | two simulations on the same domain share topics | unique `ROS_DOMAIN_ID` + `GZ_PARTITION` |
| Shell | `pkill -f <pattern>` matches the calling shell's own command line | select by `pgrep -x` and the process environment |
| Shell | killing the launch PID leaves Gazebo children | `setsid` + `kill -- -PGID` |
| git | `git merge -F -` cannot read stdin | message from a file; check the merge before tagging |
| git | same name for branch and tag | push with full refs |
| git | build files committed from a branch whose `.gitignore` lacks them | update `.gitignore` before `git add -A` on every branch |
| Docker | a bind-mounted folder that does not exist is created by the Docker daemon as `root`; the container user cannot write to it | create the folder as the user before the first run |
| Docker | `osrf/ros:jazzy-desktop` is published for x86-64 only | state the CPU architecture in the requirements |
| Docker | software rendering under WSLg still uses the Gallium driver it names | `GALLIUM_DRIVER=llvmpipe` as the CPU fallback |
| LaTeX | metropolis warns "use XeLaTeX/LuaLaTeX"; title page overfull 15.6 pt | harmless with `FiraSans` under pdflatex |
| LaTeX | `newtxmath` + `amssymb` clash | load `amsmath`, then `newtxmath`, no `amssymb` |
| LaTeX | `\resizebox` on a TikZ figure enlarges its text | give the picture its natural width with `scale` |
| LaTeX | a TikZ `yshift` without a unit is in pt, not in the picture's x/y units | give the unit (`yshift=-102mm`) |
| Process | committing/tagging before the review finished | review gate + one branch per step |

---

## 18. Templates

### 18.1 CLAUDE.md skeleton

```markdown
# CLAUDE.md
Guidance for assistants. Decision history: MEMORY.md (read it first).

> **Resuming work?** Start with MEMORY.md §4 "Resume here". State <date>: …

## Project purpose
## Working rules (must follow)
1. Step by step only. 2. Teaching code. 3. Link math to code. 4. Document every step (guide + commit).
5. Update MEMORY.md. 6. Commit only when asked; push every commit; include all files.
## Git workflow: one branch per step
## Technical stack (fixed)
## Repository layout
## Known pitfalls
## Documentation conventions
```

### 18.2 MEMORY.md skeleton

```markdown
# MEMORY.md
Dates YYYYMMDD. Never delete a decision: mark it "Superseded by D-YYY".

## 1. Decision log
| ID | Date | Decision | Rationale / source |
|----|------|----------|--------------------|
## 2. Open questions / pending decisions
| ID | Topic | Options / notes | Status |
## 3. Roadmap status
| # | Step | Source reference | Status |
## 4. Resume here (handoff)
## 5. Reference data
## 6. Session log
| Date | Summary |
```

### 18.3 Step guide skeleton

```markdown
# Step XX — <Title>
## Objective
## Paper reference
## Theory
## Files created / changed
## How to build and run
## Expected result
## Exercises
```

### 18.4 Commit message

An example from the working repository (Step 11):

```
step-11.1: repeatability with a noisy virtual Vicon and object retention

- caging_control: core/noise.py (Gaussian pose noise, seeded) + 4 tests;
  vicon node (ground truth + noise -> /<body>/vicon/odom); parameter
  pose_source in lqr_controller, leader_follower, approach_manager; only the
  controllers read the noisy poses, the evaluation keeps the ground truth
- plot_results: groups repeated runs (mean ± std), max cage, retained
- caging_bringup: launch args vicon_noise_xy, vicon_noise_yaw, seed
  (default 0 = Steps 1-10 unchanged); run_experiments.sh REPEATS, NOISE_XY
- batch 4 box configurations x 5 seeds (2 mm, 0.005 rad): 20/20 DONE,
  20/20 retained, max cage 17.5 mm, e_dO std <= 2 mm
- docs/steps/step_11_repeatability.md, README, decision log

Co-Authored-By: <assistant attribution, if used>
```

First line: `<branch>.<n>: <summary>`. Body: one bullet per area (what and why), what was
verified with numbers, which documents and decisions were updated.

### 18.5 Handoff checklist

```markdown
**Paused <date> at <machine>. Next session: <date> at <machine>.**
State: what is merged/tagged, which branch is in progress, what is pushed.
1. Sync: git fetch --tags; git checkout main && git merge --ff-only origin/main
2. Branch to continue: git checkout -b <branch> origin/<branch>
3. Tools/dependencies to check on that machine (commands)
4. Rebuild / tests needed? (commands, expected counts)
5. Ask the user: <the next decision>
Only on one machine: <large or ignored data>
```

### 18.6 Decision question to the professor

```
<Question>?
  A. <option> (Recommended) — <why>
  B. <option> — <trade-off>
  C. <option> — <trade-off>
```
