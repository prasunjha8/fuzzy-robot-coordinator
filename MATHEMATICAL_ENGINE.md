# Mathematical Engine and Optimization

This note explains the decision calculations used by the fuzzy robot coordinator and how their results drive the MuJoCo robots. It describes the implementation in `fuzzy_robot/main.py`, `simulator.py`, and `mujoco_sim.py`.

## 1. Robot-task inputs

For robot \(i\) and task \(j\), the decision engine uses:

| Symbol | Input | Meaning |
|---|---|---|
| \(B_i\) | Battery | Remaining battery percentage, from 0 to 100 |
| \(D_{ij}\) | Distance | Euclidean distance from robot \(i\) to task \(j\), in metres |
| \(W_i\) | Workload | Current workload indicator |
| \(L_{ij}\) | Load ratio | Task payload divided by robot payload capacity |
| \(U_j\) | Urgency | Task urgency, from 0 to 100 |

Distance and payload ratio are calculated as:

\[
D_{ij} = \sqrt{(x_i-x_j)^2 + (y_i-y_j)^2}
\qquad
L_{ij} = \frac{\text{task payload}_j}{\text{robot capacity}_i}.
\]

The pair is infeasible when \(L_{ij} > 1\). Infeasible robot-task pairs return `None`; the allocator never assigns them.

## 2. Fuzzification

Fuzzification maps a numerical input to a membership degree \(\mu(x)\in[0,1]\). The code uses triangular and shoulder functions.

### Triangular membership

For parameters \(a < b < c\), the triangular function is:

\[
\mu_{\triangle}(x;a,b,c)=
\begin{cases}
0 & x\leq a \text{ or } x\geq c,\\
\frac{x-a}{b-a} & a < x < b,\\
\frac{c-x}{c-b} & b\leq x<c.
\end{cases}
\]

It rises from zero at \(a\), reaches one at \(b\), then falls to zero at \(c\).

### Shoulder membership

The left shoulder is:

\[
\mu_{\mathrm{left}}(x;a,b)=
\begin{cases}
1 & x\leq a,\\
\frac{b-x}{b-a} & a<x<b,\\
0 & x\geq b.
\end{cases}
\]

The right shoulder is its increasing counterpart:

\[
\mu_{\mathrm{right}}(x;a,b)=
\begin{cases}
0 & x\leq a,\\
\frac{x-a}{b-a} & a<x<b,\\
1 & x\geq b.
\end{cases}
\]

The membership breakpoints currently configured in `fuzzy_robot/main.py` are:

| Variable | Linguistic set | Membership |
|---|---|---|
| Battery (%) | low | left shoulder \((30,50)\) |
| | medium | triangle \((30,50,70)\) |
| | high | right shoulder \((50,70)\) |
| Distance (m) | near | left shoulder \((2,5)\) |
| | medium | triangle \((3,6,9)\) |
| | far | right shoulder \((7,11)\) |
| Workload | light | left shoulder \((0.5,1.5)\) |
| | moderate | triangle \((0.5,1.5,2.5)\) |
| | heavy | right shoulder \((1.5,2.5)\) |
| Load ratio | low | left shoulder \((0.2,0.4)\) |
| | medium | triangle \((0.3,0.5,0.7)\) |
| | high | right shoulder \((0.6,0.8)\) |
| Urgency (0-100) | low | left shoulder \((20,40)\) |
| | medium | triangle \((25,50,75)\) |
| | high | triangle \((60,80,95)\) |
| | critical | right shoulder \((85,100)\) |

Adjacent sets overlap, so an input can partially belong to more than one linguistic category. For example, a battery can be partly `medium` and partly `high`.

## 3. Sugeno-style fuzzy inference

The rule base contains eight expert-authored rules. A rule's antecedent uses the minimum membership as its firing strength:

\[
\alpha_r = \min_{(v,\ell)\in r}\mu_{v,\ell}(x_v).
\]

For example, the rule

> IF battery is high AND distance is near AND load is low

fires with strength:

\[
\alpha_r = \min\left(
\mu_{\mathrm{battery,high}},
\mu_{\mathrm{distance,near}},
\mu_{\mathrm{load,low}}
\right).
\]

Each rule has a first-order Sugeno consequent. Before calculating it, inputs are normalized as:

\[
b = B_i/100,\quad
d = \min(D_{ij}/14,1),\quad
\ell = L_{ij},\quad
w = \min(W_i/3,1),\quad
u = U_j/100.
\]

The consequent for rule \(r\) is an affine score:

\[
z_r = \operatorname{clip}_{[0,1]}
\left(
c_r + \beta_{r,b}b+\beta_{r,d}d+
\beta_{r,\ell}\ell+\beta_{r,w}w+\beta_{r,u}u
\right).
\]

The current rule parameters \((c,\beta_b,\beta_d,\beta_\ell,\beta_w,\beta_u)\) are:

| Rule | Antecedent | \(c\) | \(\beta_b\) | \(\beta_d\) | \(\beta_\ell\) | \(\beta_w\) | \(\beta_u\) |
|---|---|---:|---:|---:|---:|---:|---:|
| 1 | battery high, distance near, load low | 0.70 | 0.20 | -0.10 | -0.10 | -0.05 | 0.10 |
| 2 | battery high, distance medium, load medium | 0.60 | 0.20 | -0.10 | -0.10 | -0.05 | 0.10 |
| 3 | battery medium, distance near, load medium | 0.55 | 0.15 | -0.05 | -0.10 | -0.05 | 0.15 |
| 4 | battery low, distance far | 0.20 | 0.10 | -0.20 | -0.10 | -0.10 | 0.10 |
| 5 | load high, battery low | 0.15 | 0.10 | -0.05 | -0.20 | -0.10 | 0.10 |
| 6 | urgency critical, battery high, distance near | 0.75 | 0.15 | -0.05 | -0.05 | -0.05 | 0.20 |
| 7 | workload heavy, distance far | 0.20 | 0.10 | -0.15 | -0.05 | -0.20 | 0.10 |
| 8 | urgency high, battery medium, distance medium | 0.55 | 0.15 | -0.10 | -0.05 | -0.05 | 0.20 |

The fuzzy suitability is the normalized weighted average of the fired rule consequents:

\[
S^{\mathrm{fuzzy}}_{ij} =
\begin{cases}
\frac{\sum_r \alpha_r z_r}{\sum_r \alpha_r} & \sum_r\alpha_r>0,\\
0 & \text{no rule fires}.
\end{cases}
\]

This is Sugeno-style inference: it averages numerical consequent functions rather than forming an output fuzzy set and taking its centroid. The rules and their parameters are currently hand-designed, not learned from a training dataset.

## 4. Mathematical baseline and combined score

A separate weighted utility baseline provides a simple reference model. First define positive-is-better features:

\[
\begin{aligned}
b_i &= B_i/100,\\
d^+_{ij} &= \max(0,1-D_{ij}/14),\\
\ell^+_{ij} &= \max(0,1-L_{ij}),\\
w^+_i &= 1-\min(W_i/3,1),\\
u^+_j &= U_j/100.
\end{aligned}
\]

Then:

\[
S^{\mathrm{math}}_{ij}
=0.35b_i+0.25d^+_{ij}+0.20u^+_j
+0.15\ell^+_{ij}+0.05w^+_i.
\]

The weights sum to one. Battery has the largest weight; urgency, distance, payload margin, and workload contribute the remaining weights. The baseline is clipped to \([0,1]\).

For feasible pairs, the combined suitability used by the simulation is:

\[
S_{ij}=0.65S^{\mathrm{fuzzy}}_{ij}+0.35S^{\mathrm{math}}_{ij}.
\]

The fuzzy score therefore remains the primary signal, while the mathematical baseline gives a non-rule-based contribution. These weights are design choices that can be tuned or evaluated; they are not learned parameters.

## 5. Task allocation as a constrained optimization

At each dispatch, the runtime in `simulator.py` considers robots that are idle, not charging, and above the low-battery threshold, along with all pending tasks. A robot-task pair with payload exceeding capacity is excluded.

Urgency is applied as a priority multiplier:

\[
P_{ij}=S_{ij}\left(0.5+\frac{U_j}{200}\right).
\]

For urgency \(0\leq U_j\leq100\), the multiplier ranges from 0.5 to 1.0. Let \(x_{ij}\in\{0,1\}\) indicate whether available robot \(i\) is assigned open task \(j\). The dispatch objective is:

\[
\max_x \sum_i\sum_j P_{ij}x_{ij}
\]

subject to:

\[
\sum_j x_{ij}\leq1 \quad\forall i
\qquad
\sum_i x_{ij}\leq1 \quad\forall j
\qquad
x_{ij}=0 \text{ for infeasible pairs.}
\]

The inequalities allow robots or tasks to remain unassigned. The current implementation searches the small assignment space directly and finds the maximum-total-priority matching for the available robots in that dispatch. Equal-utility solutions are resolved deterministically by robot and task insertion order. Since the demo has three robots, exhaustive matching is small; a larger fleet would benefit from a polynomial-time assignment solver such as the Hungarian algorithm or min-cost flow.

Assignments already in progress remain reserved. If a robot's battery reaches 25% or less, it suspends its task, drives to the charger, and resumes dispatch after charging to 85%. Completed tasks update the associated demo inventory count. Once all open tasks are finished, the simulation queues another named delivery task while its 24-marker capacity allows it.

## 6. From assignment to MuJoCo wheel motion

The MuJoCo model represents each robot as a differential-drive body with two actuated wheels. With wheel radius \(r\), wheel separation \(L\), and wheel angular velocities \(\omega_L,\omega_R\), ideal differential-drive kinematics are:

\[
v=\frac{r}{2}(\omega_R+\omega_L),
\qquad
\dot{\theta}=\frac{r}{L}(\omega_R-\omega_L).
\]

The controller calculates the target bearing \(\theta^*=\operatorname{atan2}(y^*-y,x^*-x)\), heading error \(e_\theta=\operatorname{wrap}(\theta^*-\theta)\), and target distance \(d\). It requests:

\[
v_c=\min(0.8,1.2d)\max(0,\cos(e_\theta)),
\qquad
\omega_c=\operatorname{clip}(2.5e_\theta,-1.8,1.8).
\]

These chassis velocities are converted to wheel speeds:

\[
\omega_L=\frac{v_c-\frac{L}{2}\omega_c}{r},
\qquad
\omega_R=\frac{v_c+\frac{L}{2}\omega_c}{r}.
\]

Wheel speeds are limited to \([-12,12]\) rad/s and sent to MuJoCo velocity actuators. MuJoCo advances the rigid-body and wheel dynamics; measured body positions feed back into the task and battery state. The current controller is intentionally simple: it follows direct target bearings and does not implement obstacle-aware path planning or ROS navigation.

## 7. Battery and charging model

Battery is a simulation state rather than an electrical battery model. In the MuJoCo path, the charge percentage is reduced by 1.5 percentage points per metre traveled. At or below 25%, the robot targets the dock at \((1,9)\), and while docked its charge increases by 12 percentage points per simulated second until it reaches 85%. The browser companion uses the same thresholds and dock, with its own simplified movement update.

## 8. Scope and next mathematical experiments

This prototype is intended to make the theory visible, not to claim an industrially validated controller. Useful next experiments:

1. Sweep the membership breakpoints and compare assignment outcomes.
2. Compare fuzzy-only, mathematical-only, and combined scores on repeated scenarios.
3. Compare the current objective against minimum travel distance, balanced workload, and battery reserve constraints.
4. Add uncertain travel time or energy as triangular fuzzy numbers and propagate their bounds.
5. Learn rule/consequent parameters from labelled examples, then compare against this transparent hand-authored baseline.
6. Model inventory demand and replenishment explicitly; inventory updates in the current demo are simple task-completion effects, not a learned demand forecast.

Run the implementation and tests from the repository root:

```bash
.venv/bin/mjpython mujoco_sim.py
.venv/bin/python -m unittest discover -s tests -v
```
