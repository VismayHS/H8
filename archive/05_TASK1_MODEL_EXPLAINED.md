# Task 1 — The 6-DOF Quadcopter Model, Explained

**HACKSIMUL8 2026 · PES University · 2 September 2026**
**Status: COMPLETE and NUMERICALLY VERIFIED**

> **What Task 1 asked:**
> *"Develop the dynamic mathematical model of the 6 DOF UAV Quadcopter in MATLAB and
> SIMULINK in terms of state space equations."*

Three deliverables are hidden in that sentence: a **dynamic model**, in **state-space**
form, and **evidence that it is correct**. This document covers all three.

---

## 1. What "6-DOF" actually means

A rigid body free in space has **six degrees of freedom**:

| | Degrees of freedom | Symbol |
|---|---|---|
| **Translation** | move along x, y, z | ξ = [X, Y, Z] |
| **Rotation** | rotate about x, y, z | η = [φ, θ, ψ] |

- **φ (roll)** — rotation about the body x-axis
- **θ (pitch)** — rotation about the body y-axis
- **ψ (yaw)** — rotation about the body z-axis

**The central difficulty:** the quadcopter has **6 degrees of freedom but only 4
actuators**. It is *underactuated*. It physically cannot slide sideways — it must **tilt**
to move horizontally. That single fact shapes the entire control problem, and it appears
in the model as the coupling `Ẍ = g·θ`.

---

## 2. Two coordinate frames

The problem statement's Fig. 1 shows both:

- **Inertial (ground) frame** `O–xyz` — fixed to the earth. Position and altitude are
  measured here.
- **Body frame** `O_B–x_B y_B z_B` — glued to the airframe, origin at the centre of
  gravity. **Thrust always acts along +z_B**, whatever attitude the vehicle is in.

Everything in the model comes from this one tension: *forces are generated in the body
frame, but motion is measured in the inertial frame.* The rotation matrix links them.

The three assumptions stated in the problem are what let us do this:

1. homogeneous and symmetric block → the inertia tensor is **diagonal**, `I = diag(Ixx, Iyy, Izz)`
2. centre of the quadcopter is the centre of gravity → no offset terms
3. transmission absolutely rigid, elasticity neglected → no flexible-body dynamics

---

## 3. From rotor speeds to forces and torques

Each rotor spins at ω_i and produces:

- **thrust** `f_i = k·ω_i²` along +z_B (k = 2.996×10⁻⁶ N·s²)
- **reaction torque** `τ_i = b·ω_i²` about z_B (b = 1.260×10⁻⁷ N·m·s²)

Both scale with **ω²**, which is what makes the quadcopter nonlinear at the input.

### Rotor positions (from Fig. 1)

The figure shows a **cross / "plus" configuration** — the arms lie along the body axes:

| Rotor | Position | Spin |
|---|---|---|
| 1 | (l, 0, 0) — on +x_B | one way |
| 2 | (0, −l, 0) — on −y_B | other way |
| 3 | (−l, 0, 0) — on −x_B | one way |
| 4 | (0, l, 0) — on +y_B | other way |

Rotors 1 and 3 spin opposite to 2 and 4, so in hover their reaction torques **cancel** and
the quadcopter does not spin about z.

### Deriving the moments with τ = r × F

Take rotor 4 at `r₄ = (0, l, 0)` with thrust `F₄ = (0, 0, f₄)`:

```
τ₄ = r₄ × F₄ = | i   j   k  |
               | 0   l   0  |  =  (l·f₄, 0, 0)
               | 0   0   f₄ |
```

So rotor 4 gives a **positive roll torque**. Rotor 2 at `(0, −l, 0)` gives `(−l·f₂, 0, 0)`.
Doing this for all four rotors gives the **four virtual control inputs**:

```
U1 = k(ω1² + ω2² + ω3² + ω4²)      total thrust          [N]
U2 = k(ω4² − ω2²)                  roll  input   → τφ = l·U2
U3 = k(ω3² − ω1²)                  pitch input   → τθ = l·U3
U4 = b(ω1² + ω3² − ω2² − ω4²)      yaw torque            [N·m]
```

**Why this matters:** we design controllers in terms of `U1…U4` (thrust and three
moments), which are physically intuitive, and only convert back to individual rotor speeds
at the very end. This decoupling is standard practice and makes the whole problem
tractable.

**Verified in code:** commanding all four rotors at ω_hover = 649.9 rad/s gives
`U1 = 5.0620 N` (exactly the weight) and all three moments zero — confirming the mapping
and the hover point together.

---

## 4. Translational dynamics

Thrust acts along +z_B. To find its effect in the inertial frame we rotate it using the
ZYX Euler rotation matrix `R = Rz(ψ)·Ry(θ)·Rx(φ)`:

```
       ⎡ cψcθ   cψsθsφ − sψcφ   cψsθcφ + sψsφ ⎤
  R =  ⎢ sψcθ   sψsθsφ + cψcφ   sψsθcφ − cψsφ ⎥
       ⎣ −sθ    cθsφ            cθcφ          ⎦
```

The body-frame thrust vector is `F_b = [0, 0, U1]ᵀ`. Rotating it and applying Newton's
second law with gravity gives:

```
Ẍ = (U1/m)(cos φ sin θ cos ψ + sin φ sin ψ)
Ÿ = (U1/m)(cos φ sin θ sin ψ − sin φ cos ψ)
Z̈ = −g + (U1/m)(cos φ cos θ)
```

**Read the third equation carefully — it is the heart of Task 2.** Altitude acceleration
depends on thrust *multiplied by* `cos φ cos θ`. Tilt the quadcopter and you lose vertical
thrust. At 30° of pitch you have lost about 13% of your lift. A naive altitude controller
sags every time the vehicle manoeuvres.

---

## 5. Rotational dynamics

Euler's equations for a rigid body with a diagonal inertia tensor, plus the **gyroscopic
term** from the spinning rotors:

```
φ̈ = ((Iyy − Izz)/Ixx)·θ̇ψ̇ − (I_M/Ixx)·θ̇·Ω_r + (l/Ixx)·U2
θ̈ = ((Izz − Ixx)/Iyy)·φ̇ψ̇ + (I_M/Iyy)·φ̇·Ω_r + (l/Iyy)·U3
ψ̈ = ((Ixx − Iyy)/Izz)·φ̇θ̇                    + (1/Izz)·U4
```

Three distinct effects, worth naming separately:

| Term | Name | Meaning |
|---|---|---|
| `((Iyy−Izz)/Ixx)θ̇ψ̇` | **inertial coupling** | rotating about two axes creates torque about the third — this is the "coupled dynamics" the problem statement mentions |
| `(I_M/Ixx)·θ̇·Ω_r` | **gyroscopic** | the four spinning rotors act as a gyroscope and resist tilting |
| `(l/Ixx)·U2` | **control** | the moment you actually command |

`Ω_r = ω1 − ω2 + ω3 − ω4` is the **residual rotor speed**. In perfect hover all four are
equal and Ω_r = 0, so the gyroscopic term vanishes. It only bites during aggressive
manoeuvres.

Note that `Ixx = Iyy = 4.984×10⁻³` but `Izz = 8.958×10⁻³` — nearly double. The quadcopter
is much harder to spin about the vertical axis, which is why **yaw is always the slowest
channel**.

### One honest assumption you must be ready to defend

The equations above use the **Euler angle rates** (φ̇, θ̇, ψ̇) where strictly they should
use **body angular rates** (p, q, r). The two are related by

```
⎡p⎤   ⎡1   0      −sin θ      ⎤ ⎡φ̇⎤
⎢q⎥ = ⎢0   cos φ   cos θ sin φ⎥ ⎢θ̇⎥
⎣r⎦   ⎣0  −sin φ   cos θ cos φ⎦ ⎣ψ̇⎦

```

For small angles this matrix → identity, so p ≈ φ̇, q ≈ θ̇, r ≈ ψ̇. This is the standard
simplification used in the quadcopter literature, including the Mien & Tu (2024) paper the
organisers cite. **It is valid near hover and for moderate angles, which is our entire
operating regime.** If a judge asks about the difference, this is the answer — do not
pretend the approximation is not there.

---

## 6. The state-space form

The task explicitly asks for state space. We choose **12 states** — two per degree of
freedom, since each is governed by a second-order equation:

```
x = [ X  Ẋ  Y  Ẏ  Z  Ż  φ  φ̇  θ  θ̇  ψ  ψ̇ ]ᵀ
```

Input vector `u = [δU1, U2, U3, U4]ᵀ`.

### Linearisation about hover

The equations above are nonlinear (products of states, trigonometric functions). Classical
control theory needs a **linear** model, so we linearise about the hover equilibrium:

```
all angles = 0,  all rates = 0,  U1 = W = mg = 5.0620 N
```

Applying small-angle approximations (sin φ ≈ φ, cos φ ≈ 1) and dropping products of small
terms:

| Nonlinear | Linearised |
|---|---|
| `Ẍ = (U1/m)(cφsθcψ + sφsψ)` | `Ẍ = g·θ` |
| `Ÿ = (U1/m)(cφsθsψ − sφcψ)` | `Ÿ = −g·φ` |
| `Z̈ = −g + (U1/m)cφcθ` | `Z̈ = δU1/m` |
| `φ̈ = … + (l/Ixx)U2` | `φ̈ = (l/Ixx)·U2` |
| `θ̈ = … + (l/Iyy)U3` | `θ̈ = (l/Iyy)·U3` |
| `ψ̈ = … + (1/Izz)U4` | `ψ̈ = (1/Izz)·U4` |

Giving `ẋ = Ax + Bu` with the non-zero entries:

```
A(1,2) = 1        A(2,9)  = +g       (Ẍ = g·θ)
A(3,4) = 1        A(4,7)  = −g       (Ÿ = −g·φ)
A(5,6) = 1        B(6,1)  = 1/m
A(7,8) = 1        B(8,2)  = l/Ixx
A(9,10)= 1        B(10,3) = l/Iyy
A(11,12)=1        B(12,4) = 1/Izz
```

`C = I₁₂` (all states measured), `D = 0`.

**The `A(2,9) = g` and `A(4,7) = −g` entries are the underactuation made visible.** They
say: the only way to accelerate horizontally is to tilt. There is no B entry that moves the
vehicle sideways directly.

### The system decouples into four independent channels

| Channel | Equation | Transfer function | Value |
|---|---|---|---|
| **Altitude** | `Z̈ = δU1/m` | `1/(m s²)` | `1/(0.516 s²)` |
| Roll | `φ̈ = (l/Ixx)U2` | `l/(Ixx s²)` | `45.1/s²` |
| Pitch | `θ̈ = (l/Iyy)U3` | `l/(Iyy s²)` | `45.1/s²` |
| Yaw | `ψ̈ = (1/Izz)U4` | `1/(Izz s²)` | `111.6/s²` |

**Every channel is a double integrator.** This is the single most useful structural fact in
the whole problem, and it directly drives Task 2.

---

## 7. Why "double integrator" matters so much

`G(s) = 1/(m s²)` has **two poles at the origin**. Consequences:

1. **It is marginally stable, not stable.** Apply a constant force and the output grows
   without bound (as t²). Verified: a 2% thrust error makes altitude diverge quadratically.

2. **Proportional control alone cannot stabilise it.** With P-only, the closed loop is
   `m·s² + Kp = 0`, giving poles at `±j√(Kp/m)` — pure imaginary, sustained oscillation for
   *every* Kp. **Derivative action is mandatory, not optional.**

3. **Ziegler–Nichols is undefined here.** The ZN ultimate-gain method needs a unique gain
   Ku at which oscillation begins. Since it oscillates at every gain, no such Ku exists.
   The open-loop ZN variant also fails, since a double integrator has no S-shaped step
   response. *Saying this to the judges proves you understood the plant rather than
   applying a recipe.*

---

## 8. Verification — the evidence the model is right

Anyone can write equations. The mark is in proving they are correct. Three independent
checks, all passing:

### Check 1 — hover is a true equilibrium

Feed the state `x = 0` and input `U1 = W` into the nonlinear model. Every derivative must
be zero.

```
Residual at hover  ||ẋ|| = 0.000e+00        ✓
```

### Check 2 — the linearisation matches the nonlinear model

Finite-difference the nonlinear dynamics about hover and compare against the hand-derived
A and B matrices, entry by entry:

```
max|A − A_numerical| = 1.636e-12            ✓
max|B − B_numerical| = 3.122e-10            ✓
```

**This is the number to put on your slide.** Agreement to twelve decimal places means the
hand derivation and the code are the same system. It converts "trust our algebra" into
"here is the proof".

### Check 3 — controllability

```
rank(ctrb(A,B)) = 12 of 12  →  fully controllable    ✓
```

All twelve states can be driven with only four inputs. Underactuated does **not** mean
uncontrollable — it means you reach the horizontal states *through* the attitude states.
The rank test proves the four inputs are enough.

### Supporting result

```
Open-loop poles: all 12 at the origin
```

Confirms the vehicle cannot fly without feedback — which is the whole justification for
Tasks 2–4.

---

## 9. The files that implement Task 1

| File | What it does |
|---|---|
| `quad_params.m` | All eight Table 1 parameters, plus derived W = 5.0620 N and ω_hover = 649.9 rad/s, actuator limits, sample time |
| `quad_dynamics.m` | The full nonlinear 6-DOF model — the six equations of §4 and §5, with optional disturbance input |
| `rotor2U.m` | Rotor speeds → U1…U4, with the τ = r × F geometry documented |
| `task1_statespace.m` | Builds A, B, C, D; runs all three verification checks; forms the four channel transfer functions; saves `task1_model.mat` |
| `smoke_test.m` | 10-point regression check of the whole package |

**Output:** `task1_model.mat` containing `P, A, B, C, D, sys_full, sys_alt, G_alt`.

---

## 10. How to present Task 1 (about 3 minutes)

1. **"6 DOF, 4 actuators — it's underactuated."** Show the state vector. Say the quadcopter
   must tilt to translate, and point at `Ẍ = g·θ` in the A matrix.
2. **"Thrust and torques come from ω²."** Show the U1–U4 mapping and mention τ = r × F.
3. **"Here are the six nonlinear equations."** Point out the `cos φ cos θ` in the altitude
   equation and say it is why Task 2 needs feedback linearisation.
4. **"Linearised about hover, it decouples into four double integrators."** Show the table.
5. **"And here is the proof it is right."** Show `max|A − A_num| = 1.6e-12`, rank 12, and
   the zero hover residual.
6. **"All twelve poles are at the origin, so it cannot fly open loop."** → hand over to
   Task 2.

### Questions you will be asked

**"Why 12 states?"**
Six degrees of freedom, each governed by a second-order differential equation, so two
states each — position and velocity.

**"Why linearise if you already have the nonlinear model?"**
Because Bode, root locus, margins and `pidtune` all assume linearity. We keep the nonlinear
model for *simulation* and use the linear one for *design* — then verify the design back on
the nonlinear model. Task 2's Method D does exactly that.

**"Is the linear model valid for big angles?"**
No — it assumes small angles. That is precisely why the altitude controller uses feedback
linearisation instead, which stays exact at any tilt. We verified it tracks to within
0.017 m at 0.30 rad (17°) of pitch.

**"You used Euler rates instead of body rates."**
Correct, and deliberate. The two coincide for small angles, and it is the standard
formulation used in the reference paper. It is an approximation, and here is the exact
transformation matrix that relates them.

**"What did you neglect?"**
Aerodynamic drag on the airframe (coded but set to zero to match Table 1), blade flapping,
motor dynamics (rotors assumed to reach commanded speed instantly), ground effect, and
airframe flexibility — the last by explicit instruction in the problem statement.
