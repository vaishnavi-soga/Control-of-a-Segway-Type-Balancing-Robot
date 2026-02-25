<div align="center">

![Segway Balancing Robot Banner](images/banner.svg)

**Course:** MAE 506 – Advanced System Modeling, Dynamics, and Control  
**Institution:** Arizona State University  
**Team:** Abhay Parwal · Vaishanavi Sogalad · Josia Vargheese Thomas

</div>

---

## 📖 What Is This Project? (Start Here)

### The Problem

Imagine balancing a broomstick upright on your palm. The moment you stop moving your hand to correct it, it falls. A **Segway-type robot** has exactly this problem — it's a rigid body (like the broomstick) balanced on two wheels, and it will fall over the instant it tilts even slightly, unless something actively corrects it.

This is the classic **inverted pendulum** problem, one of the most fundamental challenges in control engineering.

```
        [Body mass]          ← naturally wants to fall!
             |
             |   ← rigid rod (pendulum)
             |
    =====[axle]=====
      ( wheel )  ( wheel )
    ________________________  ← ground
```

### The Solution

This project builds a **mathematical model** of the Segway and designs a **state-feedback controller** that continuously reads the robot's tilt and wheel velocity, then commands the motor torque needed to keep it upright.

The complete process covered in this project:

1. **Model** — Derive the equations of motion using Lagrangian mechanics
2. **Linearize** — Simplify the nonlinear model around the upright position (θ = 0)
3. **Analyze** — Check controllability, observability, and stability
4. **Design** — Compute a feedback gain matrix K using pole placement
5. **Simulate** — Verify the controller works in MATLAB

---

## 🔑 Key Concepts Explained Simply

### What Is State-Space?

Instead of one big equation, we describe the system using a set of **state variables** — the minimum information needed to predict future behavior:

| State | Symbol | What it is |
|---|---|---|
| x₁ | θ | Body tilt angle (how far from upright) |
| x₂ | θ̇ | Body angular velocity (how fast tilting) |
| x₃ | φ̇ | Wheel angular velocity (wheel spinning speed) |

The system equations then take the compact form:

```
ẋ = Ax + Bu       (how states evolve over time)
y  = Cx            (what we can measure: just θ here)
```

where **A** describes the natural dynamics, **B** describes how the motor input affects states, and **C** picks what we measure.

### Why Is the Robot Unstable?

The open-loop eigenvalues of the A matrix tell us everything:

```
λ₁ ≈ -7.04              → stable (one decaying direction)
λ₂,₃ ≈ 1.13 ± 4.58j    → UNSTABLE (growing oscillations)
```

Because two eigenvalues have **positive real parts**, any small tilt grows exponentially — the robot tips over. This is called a **saddle-focus** instability.

### What Is Pole Placement?

With state feedback `u = -Kx`, the closed-loop dynamics become:

```
ẋ = (A - BK)x
```

The eigenvalues of `(A - BK)` are the **closed-loop poles** — and we can place them wherever we want (since the system is controllable). We choose locations that give us:
- **~5% overshoot** → damping ratio ζ ≈ 0.69
- **~2s settling time** → natural frequency ωn ≈ 2.90 rad/s

Desired poles: **s₁,₂ = -2 ± 2j** (dominant pair) and **s₃ = -5** (fast mode)

### Control Loop Architecture

![Control Loop Diagram](images/control_loop.svg)

The controller reads all three states, multiplies by gain K, and feeds back a corrective torque to the motor — 100s of times per second.

### What Is Controllability?

> *"Can I drive the system to any state I want using the motor?"*

Test: if `rank([B, AB, A²B]) = 3` (full rank), YES. Our system: **rank = 3 ✓**

### What Is Observability?

> *"Can I figure out all internal states just from measuring θ?"*

Test: if `rank([C; CA; CA²]) = 3` (full rank), YES. Our system: **rank = 3 ✓**

This is important because in a real robot you might only have an angle sensor — but you can still reconstruct θ̇ and φ̇ mathematically.

---

## 📐 Physical Parameters

| Parameter | Symbol | Value | Units |
|---|---|---|---|
| Wheel radius | r | 0.085 | m |
| Wheel mass | m_w | 0.45 | kg |
| Wheel inertia | I_w | 0.0012 | kg·m² |
| Body mass | m_b | 3.20 | kg |
| Body inertia | I_b | 0.045 | kg·m² |
| COM height | h | 0.24 | m |
| Gravity | g | 9.81 | m/s² |
| Viscous friction | b | 0.11 | N·m·s |

---

## 🧮 Key Equations

### Inertia Matrix (from Lagrangian mechanics)

$$\begin{bmatrix} M_\theta & m_b hr \\ -m_b hr & M_w \end{bmatrix} \begin{bmatrix} \ddot{\theta} \\ \ddot{\phi} \end{bmatrix} = \begin{bmatrix} -m_b g h \,\theta \\ u - b\dot{\phi} \end{bmatrix}$$

### State-Space Matrices (after inversion and linearization)

$$A = \begin{bmatrix} 0 & 1 & 0 \\ -6.3488 & 0 & 1.3594 \\ -93.109 & 0 & -4.7755 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ -12.358 \\ 43.413 \end{bmatrix}, \quad C = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$$

### Feedback Gain (from pole placement)

$$K = \begin{bmatrix} 9.0053 & 11.9781 & -7.0588 \end{bmatrix}$$

### Control Law

$$u = -Kx = -(9.0053\,\theta + 11.9781\,\dot{\theta} - 7.0588\,\dot{\phi})$$

---

## 📁 Repository Structure

```
MAE506-Segway-Control/
│
├── 🚀 QUICKSTART
│   └── segway_main.m          ← Run this ONE file to get ALL results
│
├── 🔧 INDIVIDUAL SCRIPTS (run separately for specific results)
│   ├── segway_model.m          ← Builds A, B, C, D matrices from physical params
│   ├── segway_analysis.m       ← Controllability, observability, stability
│   ├── segway_phase_portrait.m ← 3D phase portrait (open-loop instability)
│   └── segway_controller.m     ← Pole placement + closed-loop simulation
│
├── 🖼️ images/
│   ├── banner.svg              ← Repository header image
│   └── control_loop.svg        ← Closed-loop block diagram
│
└── README.md
```

**Recommended order for understanding the project:**
`segway_model.m` → `segway_analysis.m` → `segway_phase_portrait.m` → `segway_controller.m`

Or just run `segway_main.m` to see everything at once.

---

## ▶️ How to Run

### Requirements
- MATLAB R2021a or newer
- **Control System Toolbox** (for `ss`, `step`, `place`, `rlocus`, `initial`)

### Steps

**1. Download this repository**

Click the green **Code** button → **Download ZIP** → extract to a folder.

Or with Git:
```bash
git clone https://github.com/vaishnavi-soga/MAE506-Segway-Control.git
```

**2. Open MATLAB and navigate to the folder**
```matlab
cd 'C:\path\to\MAE506-Segway-Control'
```

**3. Run everything at once**
```matlab
segway_main
```

**4. Or run individual scripts**
```matlab
segway_model           % Just build the model and print matrices
segway_analysis        % Controllability, observability, stability
segway_phase_portrait  % 3D phase portrait visualization
segway_controller      % Design K, simulate closed-loop
```

### Expected Outputs

Running `segway_main` produces **6 figures**:
1. **3D Phase Portrait** — spiraling trajectories showing open-loop instability
2. **Open-loop step response** — θ diverges (robot falls without control)
3. **Root locus (open-loop)** — poles in RHP confirm instability
4. **Closed-loop step response** — θ settles in ~2s with ~5% overshoot
5. **Root locus (closed-loop)** — all poles in LHP after controller applied
6. **State trajectories** — θ, θ̇, φ̇ all return to zero from 1° initial tilt

---

## 📊 Results Summary

### Open-Loop Analysis

| Property | Result |
|---|---|
| Eigenvalues | λ₁ ≈ -7.04, λ₂,₃ ≈ 1.13 ± 4.58j |
| Lyapunov stable? | ❌ No (positive real parts) |
| BIBO stable? | ❌ No (poles in RHP) |
| Controllable? | ✅ Yes (rank = 3) |
| Observable? | ✅ Yes (rank = 3) |
| Phase portrait | Outward spiral (saddle-focus) |

### Closed-Loop Controller

| Property | Result |
|---|---|
| Design method | Pole placement (`place()`) |
| Desired poles | -2 ± 2j, -5 |
| Feedback gain K | [9.0053, 11.9781, -7.0588] |
| Overshoot | ~5% ✅ |
| Settling time | ~2 seconds ✅ |
| Closed-loop stable? | ✅ Yes (all poles in LHP) |

---

## 🔮 Future Work

The report identifies several extensions that would make this a practical physical robot:

- **State observer (Luenberger/Kalman filter)** — currently the controller requires all 3 states; a real robot might only measure θ directly, so an observer would estimate θ̇ and φ̇ from θ alone
- **Hardware-in-the-loop testing** — validate on actual robot hardware with motor drivers and IMU sensors
- **Disturbance rejection** — extend to handle external pushes, ground slope, and noise
- **Actuator limits** — motor torque is bounded in reality; anti-windup and saturation handling needed
- **LQR design** — optimal trade-off between state error and control effort (vs. manual pole placement)

---

## 👥 Team Contributions

| Member | Contribution |
|---|---|
| **Vaishanavi Sogalad** | Introduction, nonlinear modeling framework, physical parameter organization, mathematical foundation |
| **Abhay Parwal** | Linearization, state-space formulation, controllability/observability/stability analysis, pole placement controller design |
| **Josia Vargheese Thomas** | All MATLAB simulations — open-loop, closed-loop, state trajectories; result analysis and validation |

---

## 📚 Course Information

**MAE 506 — Advanced System Modeling, Dynamics, and Control**  
Arizona State University

---

## 📄 License

Developed for academic purposes. Free to reference or adapt with attribution.
