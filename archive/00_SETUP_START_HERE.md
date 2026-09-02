# HACKSIMUL8 2026 — Setup Guide and Day-Of Checklist

**Event:** HACKSIMUL8 2026 · PES University, PESU 52
**Date:** Wednesday 2 September 2026 · report 8:15 AM · runs 8:30 AM – 4:30 PM
**Prepared:** 1 September 2026 (evening before)

---

## THE BLOCKER — read this first

Two problems were found on this machine tonight, and the second one is the one that will
sink you.

### 1. MATLAB is not installed

Verified: no `C:\Program Files\MATLAB`, nothing named MATLAB or MathWorks in
`Program Files`, `Program Files (x86)`, `AppData\Local` or `AppData\Roaming`, nothing on
`PATH`, and no installer sitting in `Downloads`.

### 2. Your MathWorks account has no licence — this is the real blocker

Checked at **License Center** (`mathworks.com/licensecenter/licenses`), signed in as
**vismayhs@gmail.com**:

> *"Your MathWorks Account is not currently linked to any licenses."*

And on the account overview page:

> *"You have no linked licenses or trials."*

**Without a linked licence you cannot download the installer at all.** This also explains
two other things observed tonight:

- The **Simulink Onramp** "Start course" button does nothing — the interactive lessons run
  in a hosted MATLAB Online session, which needs a licence.
- Every course in the **Control System Design** learning path is marked
  *"This course is locked — Available through the Online Training Suite."*

Fixing the licence therefore unblocks the install *and* both courses at once. Do this
tonight.

---

## Your machine versus R2026a requirements

| Requirement | R2026a needs | This machine | Verdict |
|-------------|--------------|--------------|---------|
| Windows | 11 version 23H2 or higher | Windows 11 Home SL, build 26200 | OK |
| RAM | 8 GB minimum, 16 GB recommended | 15.7 GB | OK |
| Processor | quad-core with AVX2 recommended | i5-12450H, 8 cores / 12 threads | OK |
| Disk | see below | 161.6 GB free of 397.8 GB | Plenty |

Hardware is fine. Only the licence stands in the way.

---

## How many GB is it?

Official R2026a figures from the MathWorks system requirements page:

| Install scope | Disk space |
|---------------|-----------|
| MATLAB only | **4.6 GB** |
| **Typical installation** | **5–8 GB** |
| All products | **25 GB** |

**What you actually need for HACKSIMUL8** — MATLAB, Simulink, Control System Toolbox,
Deep Learning Toolbox, Stateflow, and the core maths that ships inside MATLAB:

- **Expect roughly 8–12 GB on disk**, and a download of a similar order.
- The download is larger than you would guess because Deep Learning Toolbox and Simulink
  both carry a lot of content.
- **Budget 1–3 hours** end to end on a normal home connection. On a slow link, longer.
- The installer displays the **exact** figure once you tick your product selection, on the
  screen just before it starts downloading. Check it there.

**Do not select "all products" (25 GB) tonight.** It roughly triples your download for
toolboxes you will not open, and the clock matters.

---

## Fixing the licence — two paths

### Path A — PES University campus licence (best: free, full toolboxes, permanent)

Most Indian universities running MathWorks events hold a campus-wide (TAH) licence, and
PES is hosting this hackathon *with MathWorks*, which is a strong sign one exists.

The reason yours is not attached is that your MathWorks account is registered to
**vismayhs@gmail.com**. Campus licences associate by **university email domain**.

1. Sign in at `mathworks.com` → **My Account** → **Profile**.
2. **Add your PES email address** (your `@pesu.pes.edu` / `@pes.edu` address) to the
   account and verify it from your inbox.
3. Go back to **License Center**. The campus licence often associates automatically once a
   recognised academic domain is verified. If it does not, click **Link a License** and
   follow the academic route.
4. Alternatively, search "PES University MATLAB portal" — universities with a TAH licence
   get a dedicated download portal page that links the licence for you in one click.

> Only you can do this — it requires signing in and clicking a verification link in your
> email. I will not enter credentials on your behalf.

### Path B — 30-day free trial (fastest, guaranteed to work tonight)

If the campus route stalls, or you cannot reach anyone at PES this late:

1. Go to `mathworks.com/campaigns/products/trials.html`.
2. Choose a trial package that includes **Simulink** and **Control System Toolbox**
   (the "Control Systems" trial bundle is the right one, and it also carries Deep Learning
   Toolbox in most configurations).
3. It activates immediately against your existing account. No purchase, no card.
4. Thirty days covers the hackathon comfortably.

**My recommendation:** try Path A for fifteen minutes. If your PES email does not verify
cleanly or no licence appears, switch to Path B without hesitating. A trial that works
tomorrow morning beats a campus licence that arrives Thursday.

---

## Installing, once the licence is linked

1. `mathworks.com/downloads` → download the **R2026a** installer for Windows.
   (The event is on R2026a — the Onramp courses are labelled `simulinkR2026a`.)
2. Run the installer, sign in with your MathWorks account, accept the licence.
3. **Product selection — tick exactly these:**

   - [x] **MATLAB** *(includes the basic mathematical and computational functions the
         organisers listed)*
   - [x] **Simulink**
   - [x] **Control System Toolbox** *(named in the invitation)*
   - [x] **Deep Learning Toolbox** *(named in the invitation)*
   - [x] **Stateflow** *(the third recommended course is Stateflow Onramp — and the
         supervisory-logic problems at these events almost always want it)*
   - [x] **Simulink Control Design** *(gives you PID Tuner inside Simulink and the
         `linearize` function — you will want this and it is easy to forget)*

   Worth adding if the download size stays reasonable:

   - [ ] Signal Processing Toolbox
   - [ ] Symbolic Math Toolbox
   - [ ] Statistics and Machine Learning Toolbox
   - [ ] Optimization Toolbox

4. Install to the default location. Let it finish; do not sleep the laptop.

### Verify the install — run this in MATLAB before you sleep

```matlab
ver                                    % lists every installed product and version

% Confirm the four things the organisers named:
license('test','Control_Toolbox')      % Control System Toolbox
license('test','Neural_Network_Toolbox')  % Deep Learning Toolbox
license('test','Simulink')
license('test','Stateflow')

% Prove Simulink actually launches and simulates:
simulink                               % Library Browser must open
open_system(new_system('smoketest'))   % blank model must open

% Prove Control System Toolbox works end to end:
s = tf('s');
G = 1/(s^2 + 2*s + 1);
C = pidtune(G,'PIDF');
step(feedback(C*G,1))                  % a plot must appear
```

If `ver` lists the products and that step plot appears, you are ready.

**Fallback if the install fails:** with a linked licence you also get **MATLAB Online**
(`matlab.mathworks.com`) — full MATLAB and Simulink in the browser, nothing to install.
Slower, and it needs the venue Wi-Fi, but it will save your day. Confirm tonight that it
opens for you, so you know the fallback is real.

---

## Tonight's order of work

| When | Do |
|------|-----|
| Now | Fix the licence — Path A, then Path B if it stalls |
| Then | Start the R2026a download **immediately** — it is the long pole |
| While downloading | Read `01_Simulink_Onramp.md`, then `02_Control_System_Design.md` |
| After install | Run the verification block above |
| Before bed | Confirm MATLAB Online opens as a fallback; charge the laptop |
| If time remains | Skim `03_Stateflow_Onramp.md` |

**Reading priority if you are short on time:** file 02 (Control System Design) matters
most — the organisers named Control System Toolbox, and control design is the likeliest
problem statement. File 01 is the foundation. File 03 is marked optional by the organisers
but is what separates a good entry from an average one when the problem involves modes.

**Team split, as the organisers suggested:** one member takes Simulink Onramp, one takes
Control System Design, one takes Stateflow. Then each teaches the others for ten minutes.

---

## Pack list

- Laptop **plus charger** — an eight-hour event will flatten any battery
- Phone hotspot as Wi-Fi backup, with enough data to reach MATLAB Online
- Your MathWorks account credentials, and confirmation that they work
- These notes, in the PDFs alongside this file
- Notebook and pen for sketching block diagrams before you build them

---

## Day-of playbook

### First 30 minutes — do not touch a model yet

1. **Read the problem statement twice.** Write down, in one sentence each: what is the
   plant, what is the input, what is the output, what defines success.
2. **Sketch the block diagram on paper.** Every model you build should exist on paper
   first. This is the highest-value half hour of the day.
3. **Decide the scope you can finish by 3:30 PM**, leaving an hour for the demo. Then cut
   twenty percent from it.

### Time budget for an 8:30–4:30 event

| Time | Phase |
|------|-------|
| 8:30 – 9:00 | Inauguration; read the problem |
| 9:00 – 9:30 | Scope, sketch, split the work |
| 9:30 – 11:30 | Build the plant model; get it simulating and sane |
| 11:30 – 1:00 | Add the controller; first closed-loop result |
| 1:00 – 1:45 | Lunch — do not skip it |
| 1:45 – 3:00 | Tune, add limits and logic, handle edge cases |
| 3:00 – 3:30 | **Freeze the model.** No new features after this point |
| 3:30 – 4:15 | Produce plots, numbers and the explanation |
| 4:15 – 4:30 | Evaluation and certificates |

**The 3:00 PM freeze is the rule that wins hackathons.** A working simple model that you
can explain beats an ambitious broken one every time.

### Build order that avoids wasted work

1. Get **something running end to end** in the first hour, however crude — source, plant,
   scope. A running model you improve beats a perfect model you never finish.
2. **Open loop first.** Confirm the plant behaves sensibly before adding a controller. If
   the plant is wrong, no controller will save it.
3. **Then close the loop.** Start with proportional only, then add integral, then
   derivative.
4. **Add realism last:** saturation, noise, delay, quantisation. These turn a toy into
   something judges believe.

### Rules that save hours

- **Parameterise everything.** Every gain in a workspace variable, in one `params.m`.
  Then tuning is editing one file, not hunting through a diagram.
- **Save versions as you go:** `model_v1.slx`, `model_v2.slx`. When something breaks at
  2 PM you want a working file to fall back on.
- **Name your signals.** Unnamed lines make a diagram unreadable to judges and to you.
- **One thing at a time.** Change one parameter, run, observe. Changing three and running
  once teaches you nothing.
- **Use the Simulation Data Inspector** to overlay runs. Before-and-after on one axis is
  the most persuasive plot you can show.

### Presenting to judges

Have these ready:

1. **One sentence on the problem** and what success means.
2. **The block diagram**, which you can walk through in ninety seconds.
3. **A before-and-after plot** — untuned versus tuned, overlaid.
4. **Hard numbers:** settling time, percent overshoot, steady-state error, phase margin,
   gain margin in dB, bandwidth.
5. **An honest limitation.** "This is linearised about a 50 % operating point and would
   need gain scheduling across the full range" scores far better than pretending there is
   no limitation. Judges probe for exactly this.

**Questions to expect:**

- Why that solver / that sample time?
- What operating point did you linearise around?
- What happens when the actuator saturates? *(Have anti-windup, and say so.)*
- How would this behave on real hardware?
- Why PID rather than something else?

Every one of those is answered somewhere in files 01–03.

---

## The files in this folder

| File | What it covers |
|------|----------------|
| `00_SETUP_START_HERE` | This file — install, licence, checklist, day-of playbook |
| `01_Simulink_Onramp` | All 14 modules; blocks, signals, solvers, discrete and continuous systems, debugging |
| `02_Control_System_Design` | All 5 courses; `tf`/`ss`, linearisation, margins, `pidtune`, lead-lag design |
| `03_Stateflow_Onramp` | All 12 modules; states, transitions, temporal logic, actions, hierarchy, flow charts |

Each exists as both `.md` and `.pdf`. The PDFs are for reading on the day; the Markdown is
for searching.
