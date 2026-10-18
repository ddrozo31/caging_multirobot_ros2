# Step 4 (companion) — The Math of LQR, Step by Step

Companion to [step_04_lqr_controller.md](step_04_lqr_controller.md). That guide explains *what*
our controller does; this one explains the **matrix math** behind it, slowly, with every
multiplication written out. If matrices make you nervous, start at section 1 and do not skip
the worked examples: compute them yourself with pencil and paper, then check them with Python.

**Contents**

1. [Matrix toolkit (only what LQR needs)](#1-matrix-toolkit-only-what-lqr-needs)
2. [State-space models: describing a system with matrices](#2-state-space-models-describing-a-system-with-matrices)
3. [What LQR is: the idea in plain words](#3-what-lqr-is-the-idea-in-plain-words)
4. [Deriving the LQR: from scalar to matrices](#4-deriving-the-lqr-from-scalar-to-matrices)
5. [Example 1 — a scalar integrator, solved by hand](#5-example-1--a-scalar-integrator-solved-by-hand)
6. [Example 2 — a cart with mass (a real dynamic system)](#6-example-2--a-cart-with-mass-a-real-dynamic-system)
7. [Example 3 — a differential-drive robot](#7-example-3--a-differential-drive-robot)
8. [Example 4 — our mecanum robot, one full control step](#8-example-4--our-mecanum-robot-one-full-control-step)
9. [Choosing Q and R](#9-choosing-q-and-r)
10. [Checking your work with Python](#10-checking-your-work-with-python)
11. [Exercises (with answers)](#11-exercises-with-answers)

---

## 1. Matrix toolkit (only what LQR needs)

### 1.1 Vectors and matrices

A **vector** is a column of numbers. The pose of a robot is a 3×1 vector:

$$q = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix} = [x,\ y,\ \theta]^T$$

A **matrix** is a table with $m$ rows and $n$ columns ("$m \times n$"). The **size** is the first
thing to check in any matrix formula.

### 1.2 Matrix × vector

Each entry of the result is a **row of the matrix times the vector** (multiply pairwise, add):

$$
\begin{bmatrix} 2 & 0.5 \\ 0.5 & 1 \end{bmatrix}
\begin{bmatrix} 1 \\ 2 \end{bmatrix}
=
\begin{bmatrix} 2\cdot1 + 0.5\cdot2 \\ 0.5\cdot1 + 1\cdot2 \end{bmatrix}
=
\begin{bmatrix} 3 \\ 2.5 \end{bmatrix}
$$

**Size rule:** $(m \times n)(n \times p) = (m \times p)$. The inner sizes must match; the outer
ones give the result. A $3\times4$ matrix times a $4\times1$ vector gives a $3\times1$ vector.

Matrix × matrix is the same thing, column by column. **Order matters**: in general
$AB \ne BA$.

### 1.3 Transpose

$A^T$ swaps rows and columns: a column vector becomes a row vector.

$$\begin{bmatrix} 1 \\ 2 \end{bmatrix}^T = [1\ \ 2], \qquad
\begin{bmatrix} 1 & 2 \\ 3 & 4 \end{bmatrix}^T = \begin{bmatrix} 1 & 3 \\ 2 & 4 \end{bmatrix},
\qquad (AB)^T = B^T A^T$$

A matrix with $Q^T = Q$ is **symmetric**. $Q$, $R$ and $P$ in LQR are always symmetric.

### 1.4 The quadratic form $x^T Q x$ — the heart of LQR

$x^T Q x$ turns a vector into **one number** (a $1\times n$ times $n\times n$ times $n\times1$
product is $1\times1$). With the numbers above:

$$x^T Q x = [1\ \ 2]\begin{bmatrix} 2 & 0.5 \\ 0.5 & 1 \end{bmatrix}\begin{bmatrix} 1 \\ 2 \end{bmatrix}
= [1\ \ 2]\begin{bmatrix} 3 \\ 2.5 \end{bmatrix} = 3 + 5 = 8$$

Written out for 2×2: $x^T Q x = Q_{11}x_1^2 + 2Q_{12}x_1x_2 + Q_{22}x_2^2$. It is a **weighted
sum of squares**. If $Q$ is diagonal, there are no cross terms:

$$x^T \begin{bmatrix} q_1 & 0 \\ 0 & q_2 \end{bmatrix} x = q_1 x_1^2 + q_2 x_2^2$$

So $x^T Q x$ is a "size of $x$" where you choose how much each component counts. For our robot,
$q^T (0.1 I) q = 0.1(x^2 + y^2 + \theta^2)$.

**Positive definite** ($Q \succ 0$): $x^T Q x > 0$ for every $x \ne 0$. For a diagonal matrix:
all diagonal entries $> 0$. LQR needs $R \succ 0$ (every input costs something) and
$Q \succeq 0$ (no state is "rewarded").

### 1.5 Identity, diagonal matrices and the inverse

- $I$ (identity): ones on the diagonal, zeros elsewhere. $IA = AI = A$, $Ix = x$.
- $cI$ multiplies everything by $c$: $(0.1 I)x = 0.1x$.
- The **inverse** $A^{-1}$ undoes $A$: $A^{-1}A = I$. Only square matrices with
  $\det A \ne 0$ have one. For 2×2:

$$\begin{bmatrix} a & b \\ c & d \end{bmatrix}^{-1} = \frac{1}{ad - bc}\begin{bmatrix} d & -b \\ -c & a \end{bmatrix}$$

- For a diagonal matrix, invert each diagonal entry: $(cI)^{-1} = \frac1c I$.
- A **1×1 matrix is just a number**, and its inverse is $1/\text{number}$. This happens a lot in
  LQR when there is one input.

### 1.6 Pseudo-inverse (non-square matrices)

A $3\times4$ matrix (3 equations, 4 unknowns) has **no inverse**: there are infinitely many
solutions. The **right pseudo-inverse**

$$J^+ = J^T (J J^T)^{-1}$$

picks the solution with the smallest size, and satisfies $J J^+ = I_{3\times3}$. We use it for the
mecanum wheels in section 8.

> **Golden rule.** Before computing anything, write the size under every matrix and check that
> each product is allowed. Half of all matrix mistakes are size mistakes.

---

## 2. State-space models: describing a system with matrices

### 2.1 Continuous time

A system is described by its **state** $x$ (what you need to know to predict the future) and its
**input** $u$ (what you can command):

$$\dot x = A_c\,x + B_c\,u$$

### 2.2 Discrete time (what a computer uses)

A controller runs every $T_s$ seconds. With the **Euler** approximation
$\dot x \approx \frac{x_{(k)} - x_{(k-1)}}{T_s}$ (paper Eq. 5):

$$x_{(k)} = \underbrace{(I + T_s A_c)}_{A}\,x_{(k-1)} + \underbrace{T_s B_c}_{B}\,u_{(k-1)}$$

- $A$ ($n\times n$): how the state evolves alone in one step.
- $B$ ($n\times m$): how the input changes the state in one step.
- $n$ = number of states, $m$ = number of inputs.

### 2.3 Three systems we use in this guide

| System | State $x$ | Input $u$ | $A$ | $B$ | $n, m$ |
|---|---|---|---|---|---|
| Integrator (Ex. 1) | position $p$ | step $u$ | $1$ | $1$ | 1, 1 |
| Cart with mass (Ex. 2) | $[p,\ \dot p]^T$ | force $F$ | $\begin{bmatrix}1 & T_s\\0 & 1\end{bmatrix}$ | $\begin{bmatrix}0\\T_s/m\end{bmatrix}$ | 2, 1 |
| Our robot, linearized (Eq. 8) | $[x, y, \theta]^T$ | $v$ | $I_{3\times3}$ | $I_{3\times3}$ | 3, 3 |

Derivation of the cart: Newton $\ddot p = F/m$. With state $[p, \dot p]$:
$\dot p = \dot p$ and $\ddot p = F/m$, so
$A_c = \begin{bmatrix}0 & 1\\0 & 0\end{bmatrix}$, $B_c = \begin{bmatrix}0\\1/m\end{bmatrix}$, and
Euler gives the table row.

---

## 3. What LQR is: the idea in plain words

We want the state to go to zero (or to a goal), **but** big inputs are expensive (energy,
saturation, wheel slip, a spilled object). LQR finds the inputs that minimize the total cost

$$J = \sum_{k=0}^{\infty} \Big( \underbrace{x_{(k)}^T Q\, x_{(k)}}_{\text{"I am far from the goal"}} + \underbrace{u_{(k)}^T R\, u_{(k)}}_{\text{"I am pushing hard"}} \Big) \tag{Eq. 9}$$

- **L**inear: the model is $x_{(k)} = A x_{(k-1)} + B u_{(k-1)}$.
- **Q**uadratic: the cost uses $x^TQx$ and $u^TRu$ (section 1.4).
- **R**egulator: it drives the state to zero.

The surprising result: the best input is always a **constant matrix times the state**,

$$u = -K x$$

and $K$ is computed **once, offline**, from $A, B, Q, R$. At run time the controller is just a
matrix multiplication.

**Goal instead of zero.** To go to a goal $q^*$, regulate the *error* $x = q - q^*$:
$u = -K(q - q^*) = K(q^* - q)$. That is exactly the paper's Eq. 10, $v = K(q^* - q)$.
(For our model this works because $A_l = I$: a constant goal is an equilibrium.)

---

## 4. Deriving the LQR: from scalar to matrices

We derive it first with **plain numbers** (one state, one input), then translate to matrices.
Nothing new happens in the matrix version; only the notation changes.

### 4.1 Scalar case: $x_{(k)} = a\,x_{(k-1)} + b\,u_{(k-1)}$, cost $\sum (q x^2 + r u^2)$

**Idea (dynamic programming).** Call $V(x)$ the *minimum total cost* starting from state $x$.
For a linear system with a quadratic cost it turns out to be a parabola:

$$V(x) = P\,x^2$$

$P$ is the **"price" of being at state $x$**. Being at $x$ now costs $qx^2 + ru^2$ this step, plus
the price of wherever you land next:

$$P x^2 = \min_u \Big[\, q x^2 + r u^2 + P\,(a x + b u)^2 \,\Big] \tag{*}$$

**Step 1 — find the best $u$.** Set the derivative with respect to $u$ to zero:

$$2 r u + 2 P b\,(a x + b u) = 0 \;\Rightarrow\; u\,(r + b^2 P) = -abP\,x
\;\Rightarrow\; \boxed{u = -\underbrace{\frac{abP}{r + b^2 P}}_{K}\,x}$$

**Step 2 — find $P$.** Put $u = -Kx$ back into (*) and divide by $x^2$. After simplifying:

$$\boxed{P = q + a^2 P - \frac{a^2 b^2 P^2}{r + b^2 P}}$$

This is the **scalar Riccati equation**. $P$ appears on both sides: we solve it either with the
quadratic formula or by **iteration** (start with $P = q$, plug in, repeat).

**Our robot ($a = b = 1$):** $P = q + P - \frac{P^2}{r+P}$, so $\frac{P^2}{r+P} = q$, i.e.

$$P^2 - qP - qr = 0 \;\Rightarrow\; P = \frac{q + \sqrt{q^2 + 4qr}}{2}, \qquad K = \frac{P}{r + P}$$

(the minus root is negative, and a price cannot be negative).

### 4.2 Matrix case: just translate

| Scalar | Matrix | Why the order/transpose |
|---|---|---|
| $q x^2$ | $x^T Q x$ | section 1.4 |
| $a$, $b$ | $A$, $B$ | |
| $V = P x^2$ | $V = x^T P x$ | |
| $r + b^2P$ | $R + B^T P B$ | $(m\times n)(n\times n)(n\times m) = m \times m$ |
| $\dfrac{abP}{r + b^2P}$ | $(R + B^T P B)^{-1}\, B^T P A$ | "divide" = multiply by the inverse, **on the left** |
| $a^2 P$ | $A^T P A$ | |

So:

$$K = (R + B^T P B)^{-1} B^T P A \tag{Eq. 11}$$

$$P = A^T \Big( P - P B\,(B^T P B + R)^{-1} B^T P \Big) A + Q \tag{Eq. 12}$$

with $u = -Kx$. These are exactly the paper's Eq. 11 and 12 (the paper writes $A_l$ instead of
$A_l^T$; for $A_l = I$ they are equal).

**Size check (do this every time):** for $n$ states and $m$ inputs,

| Term | Size |
|---|---|
| $B^T P B$, $R$, $(R + B^TPB)^{-1}$ | $m \times m$ |
| $B^T P A$ | $(m\times n)(n\times n)(n\times n) = m \times n$ |
| $K$ | $(m\times m)(m\times n) = m \times n$ |
| $u = -Kx$ | $(m\times n)(n\times 1) = m\times 1$ ✓ |
| $P$, $Q$, $A^TPA$ | $n \times n$ |

### 4.3 Stability check

With $u = -Kx$ the closed loop is $x_{(k)} = (A - BK)\,x_{(k-1)}$. It converges if all
**eigenvalues** of $A - BK$ have magnitude $< 1$ (inside the unit circle). LQR guarantees this
(if the system is controllable and $Q, R$ are positive definite). Eigenvalues near 0: fast.
Near 1: slow. Complex: oscillation.

---

## 5. Example 1 — a scalar integrator, solved by hand

System: $x_{(k)} = x_{(k-1)} + u_{(k-1)}$ ($a = b = 1$), $q = 0.1$, $r = 0.8$ (Table 1).
This is **one axis of our robot** after feedback linearization.

### 5.1 Closed form

$$P = \frac{0.1 + \sqrt{0.01 + 0.32}}{2} = \frac{0.1 + 0.5745}{2} = 0.3372, \qquad
K = \frac{0.3372}{0.8 + 0.3372} = 0.2965$$

### 5.2 By iteration (what `solve_riccati` does)

Start with $P_0 = q = 0.1$ and apply $P_{new} = q + P - \frac{P^2}{r + P}$:

| iteration | $P$ | $P_{new}$ | $K = P/(r+P)$ |
|---|---|---|---|
| 0 | 0.1000 | 0.1889 | 0.1111 |
| 1 | 0.1889 | 0.2528 | 0.1910 |
| 2 | 0.2528 | 0.2921 | 0.2401 |
| 3 | 0.2921 | 0.3140 | 0.2675 |
| 5 | 0.3255 | 0.3314 | 0.2892 |
| 8 | 0.3358 | 0.3365 | 0.2956 |
| 11 | 0.3371 | 0.3371 | 0.2964 |
| ∞ | **0.3372** | | **0.2965** |

Do iteration 0 yourself: $0.1 + 0.1 - \frac{0.01}{0.9} = 0.2 - 0.0111 = 0.1889$ ✓.

### 5.3 What the controller does

$u = -0.2965\,x$: every step it covers **29.65 %** of the remaining distance. Closed loop:
$x_{(k)} = (1 - 0.2965)\,x_{(k-1)} = 0.7035\,x_{(k-1)}$, eigenvalue 0.7035 (inside the unit
circle ✓). Starting at $x = 1$ m: 0.70, 0.49, 0.35, 0.24, … After 14 steps it is below 1 cm
($0.7035^{14} = 0.007$).

---

## 6. Example 2 — a cart with mass (a real dynamic system)

Our robot is "easy" because $A = B = I$. Real dynamic systems have **inertia**: you cannot change
the position directly, only the force, which changes the velocity, which changes the position.
Here $Q$, $R$ and $P$ are true matrices with off-diagonal terms.

**System:** cart of mass $m = 2$ kg on a rail, state $x = [p,\ \dot p]^T$ (position, velocity),
input $u = F$ (force, N), $T_s = 0.1$ s:

$$A = \begin{bmatrix}1 & 0.1\\0 & 1\end{bmatrix}, \quad
B = \begin{bmatrix}0\\0.05\end{bmatrix}, \quad
Q = \begin{bmatrix}1 & 0\\0 & 0.1\end{bmatrix}, \quad R = [0.01]$$

$Q$: position errors matter 10× more than velocity. $R$ small: force is cheap.
Sizes: $n = 2$, $m = 1$, so $B^TPB$ and $R$ are 1×1 (numbers) and $K$ is 1×2.

### 6.1 One Riccati iteration, fully written out

Start with $P_0 = Q$. We compute Eq. 12 piece by piece.

**(a)** $PB$ (2×2 times 2×1 = 2×1):
$$P_0 B = \begin{bmatrix}1 & 0\\0 & 0.1\end{bmatrix}\begin{bmatrix}0\\0.05\end{bmatrix}
= \begin{bmatrix}1\cdot0 + 0\cdot0.05\\ 0\cdot0 + 0.1\cdot0.05\end{bmatrix} = \begin{bmatrix}0\\0.005\end{bmatrix}$$

**(b)** $B^T P B + R$ (1×2 times 2×1 = 1×1, a number):
$$B^T (P_0 B) + R = [0\ \ 0.05]\begin{bmatrix}0\\0.005\end{bmatrix} + 0.01 = 0.00025 + 0.01 = 0.01025$$
Its inverse is simply $1/0.01025$.

**(c)** $P B (B^TPB + R)^{-1} B^T P$ (2×1 times number times 1×2 = 2×2):
$$\frac{1}{0.01025}\begin{bmatrix}0\\0.005\end{bmatrix}[0\ \ 0.005]
= \frac{1}{0.01025}\begin{bmatrix}0 & 0\\0 & 0.000025\end{bmatrix} = \begin{bmatrix}0 & 0\\0 & 0.00244\end{bmatrix}$$

**(d)** Subtract from $P$:
$$P_0 - (c) = \begin{bmatrix}1 & 0\\0 & 0.09756\end{bmatrix}$$

**(e)** $A^T (\cdot) A$, first $(\cdot)A$, then $A^T$ times that:
$$\begin{bmatrix}1 & 0\\0 & 0.09756\end{bmatrix}\begin{bmatrix}1 & 0.1\\0 & 1\end{bmatrix} = \begin{bmatrix}1 & 0.1\\0 & 0.09756\end{bmatrix},\qquad
\begin{bmatrix}1 & 0\\0.1 & 1\end{bmatrix}\begin{bmatrix}1 & 0.1\\0 & 0.09756\end{bmatrix} = \begin{bmatrix}1 & 0.1\\0.1 & 0.10756\end{bmatrix}$$

**(f)** Add $Q$:
$$P_1 = \begin{bmatrix}1 & 0.1\\0.1 & 0.10756\end{bmatrix} + \begin{bmatrix}1 & 0\\0 & 0.1\end{bmatrix} = \begin{bmatrix}2 & 0.1\\0.1 & 0.2076\end{bmatrix}$$

Notice the **off-diagonal 0.1** that appeared: the matrix $A$ couples position and velocity, so
the "price" now depends on combinations of both. $P_1$ is still symmetric ✓.

Repeating (the computer does this) converges to

$$P = \begin{bmatrix}8.115 & 2.387\\2.387 & 1.699\end{bmatrix}$$

### 6.2 The gain

$$B^T P B + R = 0.05^2 \cdot 1.699 + 0.01 = 0.01425, \qquad
B^T P A = [0\ \ 0.05]\,P\,A = [0.1194\ \ 0.0969]$$

$$K = \frac{1}{0.01425}[0.1194\ \ 0.0969] = [\,8.38\ \ \ 6.80\,]$$

The control law $F = -Kx = -8.38\,p - 6.80\,\dot p$ is a **PD controller**: a spring
($-8.38\,p$) pulling the cart to 0 and a damper ($-6.80\,\dot p$) braking it. LQR chose the
spring and damper constants for us, from $Q$ and $R$.

### 6.3 Closed loop

$$A - BK = \begin{bmatrix}1 & 0.1\\0 & 1\end{bmatrix} - \begin{bmatrix}0\\0.05\end{bmatrix}[8.38\ \ 6.80]
= \begin{bmatrix}1 & 0.1\\-0.419 & 0.660\end{bmatrix}$$

Eigenvalues $0.83 \pm 0.11j$: magnitude 0.84 < 1 ✓ (stable), small imaginary part (slightly
oscillatory). Starting at $p = 1$ m at rest:

| k | $p$ [m] | $\dot p$ [m/s] | $F = -Kx$ [N] |
|---|---|---|---|
| 0 | 1.000 | 0.000 | −8.38 |
| 1 | 1.000 | −0.419 | −5.53 |
| 2 | 0.958 | −0.695 | −3.30 |
| 3 | 0.889 | −0.860 | −1.59 |
| 4 | 0.803 | −0.940 | −0.33 |
| 5 | 0.709 | −0.957 | +0.57 (braking) |

The cart is within 1 cm of the goal after 18 steps (1.8 s). Compare with Example 1: the position
does not jump, it must first build up velocity. That is what inertia means in matrix form.

---

## 7. Example 3 — a differential-drive robot

A differential-drive robot (e.g. TurtleBot) has inputs $v$ (forward speed) and $\omega$ (turn
rate) and the **unicycle model**:

$$\dot x = v\cos\theta, \qquad \dot y = v\sin\theta, \qquad \dot\theta = \omega$$

$$\begin{bmatrix}\dot x\\ \dot y\\ \dot\theta\end{bmatrix} =
\underbrace{\begin{bmatrix}\cos\theta & 0\\ \sin\theta & 0\\ 0 & 1\end{bmatrix}}_{3\times2}
\begin{bmatrix}v\\ \omega\end{bmatrix}$$

Three states, **two inputs**: the matrix is 3×2 and the robot **cannot move sideways**
(nonholonomic). There is no way to choose $(v, \omega)$ to get an arbitrary
$(\dot x, \dot y, \dot\theta)$, so the trick of Eq. 7 ($\mu = B^{-1}v$) fails for the full pose.

### 7.1 The fix: control a point in front of the robot

Control the point $p$ at distance $d$ in front of the wheel axle:

$$p = \begin{bmatrix}x + d\cos\theta\\ y + d\sin\theta\end{bmatrix}
\;\Rightarrow\;
\dot p = \underbrace{\begin{bmatrix}\cos\theta & -d\sin\theta\\ \sin\theta & d\cos\theta\end{bmatrix}}_{M(\theta),\ 2\times2}
\begin{bmatrix}v\\ \omega\end{bmatrix}$$

(Differentiate: $\frac{d}{dt}(d\cos\theta) = -d\sin\theta\,\omega$.) Now $M$ is **square**:
$\det M = d\cos^2\theta + d\sin^2\theta = d$. For any $d \ne 0$ it has an inverse (section 1.5):

$$M^{-1} = \frac{1}{d}\begin{bmatrix}d\cos\theta & d\sin\theta\\ -\sin\theta & \cos\theta\end{bmatrix}
= \begin{bmatrix}\cos\theta & \sin\theta\\ -\frac{\sin\theta}{d} & \frac{\cos\theta}{d}\end{bmatrix}$$

**Feedback linearization (same as paper Eq. 7):** choose $[v, \omega]^T = M^{-1} w$. Then
$\dot p = w$, and after Euler $p_{(k)} = p_{(k-1)} + T_s w$. With $v_{step} = T_s w$:
$A_l = B_l = I_{2\times2}$, **the same linear model as our robot**, one axis smaller. So the same
LQR gain works: $v_{step} = 0.2965\,(p^* - p)$.

### 7.2 Numbers

$d = 0.1$ m, $\theta = 30°$ ($\cos = 0.866$, $\sin = 0.5$). We want the point to move with
$\dot p = [0.2,\ 0.1]^T$ m/s:

$$M^{-1} = \begin{bmatrix}0.866 & 0.5\\ -5 & 8.66\end{bmatrix}, \qquad
\begin{bmatrix}v\\ \omega\end{bmatrix} = \begin{bmatrix}0.866\cdot0.2 + 0.5\cdot0.1\\ -5\cdot0.2 + 8.66\cdot0.1\end{bmatrix}
= \begin{bmatrix}0.223 \text{ m/s}\\ -0.134 \text{ rad/s}\end{bmatrix}$$

The robot drives forward and turns a little to the right to bring the point where we want it.

### 7.3 What we learn

- $\det M = d$: as $d \to 0$ the matrix becomes **singular** (no inverse). The wheel axle itself
  cannot be moved sideways: that is the nonholonomic constraint, seen in the matrix.
- We control the point, **not the heading**: $\theta$ goes wherever it must. To also control the
  heading you need other methods (linearization around a reference trajectory, nonlinear
  controllers).
- Our **mecanum** robot does not have this problem: its matrix $H(\theta)$ (3×4) always has rank 3,
  so it can control $x$, $y$ **and** $\theta$ independently. That is why the paper uses
  omnidirectional robots.

---

## 8. Example 4 — our mecanum robot, one full control step

Now we follow **one tick** of `lqr_controller.py` with real numbers: the robot is at the paper's
leader start pose and the goal is the one of the automatic test.

$$q = \begin{bmatrix}0.5\\ -2.5\\ \pi/2\end{bmatrix}, \qquad q^* = \begin{bmatrix}0\\ -1\\ \pi\end{bmatrix}, \qquad T_s = 0.05\text{ s}, \qquad K = 0.2965\,I$$

### 8.1 Error and LQR (Eq. 10)

$$e = q^* - q = \begin{bmatrix}-0.5\\ 1.5\\ 1.5708\end{bmatrix}, \qquad
v = K e = 0.2965\begin{bmatrix}-0.5\\ 1.5\\ 1.5708\end{bmatrix} = \begin{bmatrix}-0.148\\ 0.445\\ 0.466\end{bmatrix}$$

$v$ is the pose change wanted **in one step** (m, m, rad).

### 8.2 To a velocity in {G} (Eq. 7–8)

$$\frac{v}{T_s} = \begin{bmatrix}-2.97\\ 8.90\\ 9.31\end{bmatrix} \ \text{m/s, m/s, rad/s}$$

### 8.3 Saturation

Linear speed $\sqrt{2.97^2 + 8.90^2} = 9.38$ m/s > 0.3, so scale both by $0.3/9.38 = 0.032$.
Angular 9.31 > 1.0, so clip:

$$\dot q_G = \begin{bmatrix}-0.095\\ 0.285\\ 1.0\end{bmatrix}$$

### 8.4 To the robot frame {R} (inverse Eq. 3)

$R(\theta)^{-1} = R(\theta)^T$ (rotation matrices are orthogonal: the transpose is the inverse).
At $\theta = \pi/2$: $\cos = 0$, $\sin = 1$:

$$\dot q_R = R^T \dot q_G = \begin{bmatrix}0 & 1 & 0\\ -1 & 0 & 0\\ 0 & 0 & 1\end{bmatrix}
\begin{bmatrix}-0.095\\ 0.285\\ 1.0\end{bmatrix} = \begin{bmatrix}0.285\\ 0.095\\ 1.0\end{bmatrix}$$

The robot faces $+y$; the goal is up ($+y$) and a bit left ($-x$), which for the robot is
**forward and to its left**. ✓ This vector is what we publish on `/cmd_vel_nav`.

### 8.5 To wheel speeds (inverse Eq. 1, in the C++ controller)

$J_R$ (Eq. 1) is 3×4 with $r_w = 0.0325$, $L = l_x + l_y = 0.1645$:

$$J_R = \frac{r_w}{4}\begin{bmatrix}1 & 1 & 1 & 1\\ -1 & 1 & 1 & -1\\ -\frac1L & \frac1L & -\frac1L & \frac1L\end{bmatrix}$$

3 equations, 4 unknowns: no inverse, so use the pseudo-inverse $J_R^+ = J_R^T(J_RJ_R^T)^{-1}$
(section 1.6). For this matrix it has a simple form (you can check $J_R J_R^+ = I$):

$$J_R^+ = \frac{1}{r_w}\begin{bmatrix}1 & -1 & -L\\ 1 & 1 & L\\ 1 & 1 & -L\\ 1 & -1 & L\end{bmatrix}
= \begin{bmatrix}30.77 & -30.77 & -5.06\\ 30.77 & 30.77 & 5.06\\ 30.77 & 30.77 & -5.06\\ 30.77 & -30.77 & 5.06\end{bmatrix}$$

$$\mu = J_R^+ \dot q_R = \begin{bmatrix}30.77\cdot0.285 - 30.77\cdot0.095 - 5.06\cdot1.0\\ \vdots\end{bmatrix}
= \begin{bmatrix}0.78\\ 16.74\\ 6.61\\ 10.90\end{bmatrix} \text{ rad/s} \quad (\text{FL, FR, RL, RR})$$

The fastest wheel turns at 16.7 rad/s, well below the ≈ 35 rad/s of the motors ✓.
Check: $J_R\,\mu$ gives back $[0.285,\ 0.095,\ 1.0]^T$.

**Why the pseudo-inverse is the "good" solution.** $J_R\,[1, 1, -1, -1]^T = 0$: front wheels
forward and rear wheels backward makes the robot **not move** (the wheels fight each other).
You could add any amount of that to $\mu$ and the robot motion would be the same. The
pseudo-inverse gives the solution with **no fighting** (smallest wheel speeds).

### 8.6 The whole chain in one line

$$\mu = \underbrace{J_R^+}_{\text{C++}} \;\underbrace{R(\theta)^T}_{\text{Python}}\; \underbrace{\text{sat}}_{\text{Python}}\!\Big(\underbrace{\tfrac{1}{T_s}}_{\text{Python}}\, \underbrace{K\,(q^* - q)}_{\text{Eq. 10}}\Big)
\qquad\text{which is}\qquad \mu = B(\theta)^+ v,\quad B = T_s R(\theta) J_R \ \ (\text{Eq. 7})$$

(the saturation is the only part that is not in the paper's equations).

---

## 9. Choosing Q and R

### 9.1 Only the ratio matters

Multiplying $Q$ and $R$ by the same number does not change $K$ (try it with the closed form).
What matters is **how expensive errors are compared with inputs**. For our robot ($a = b = 1$):

| $q$ | $r$ | $P$ | $K$ | behavior |
|---|---|---|---|---|
| 0.1 | 8 | 0.946 | 0.106 | very gentle (10 % per step) |
| **0.1** | **0.8** | **0.337** | **0.2965** | **paper (Table 1)** |
| 1 | 1 | 1.618 | 0.618 | fast (the golden ratio appears!) |
| 1 | 0.8 | 1.525 | 0.656 | fast |
| 0.1 | 0.01 | 0.109 | 0.916 | almost dead-beat (92 % per step) |

- $R \to 0$ (inputs free): $K \to 1$, the controller tries to reach the goal in **one** step.
- $R \to \infty$ (inputs very expensive): $K \to 0$, it barely moves.

### 9.2 Bryson's rule (a good first guess)

$$Q_{ii} = \frac{1}{(\text{largest acceptable } x_i)^2}, \qquad R_{jj} = \frac{1}{(\text{largest acceptable } u_j)^2}$$

It makes every term of the cost "1" when the variable is at its limit, so different units
(meters, radians, newtons) become comparable.

### 9.3 Different weights per axis

$Q = \text{diag}(q_x, q_y, q_\theta)$ with different values gives different gains per axis. In
our robot each axis is independent ($A = B = I$), so each $k_i$ follows the closed form with
its own $q_i$, $r_i$. In the cart (Example 2) the axes are coupled and only the computer can tell
you $K$.

---

## 10. Checking your work with Python

```python
import numpy as np
from scipy.linalg import solve_discrete_are           # reference solver
from caging_control.core.lqr import lqr_gain           # our iterative solver

# Example 2: cart with mass
Ts, m = 0.1, 2.0
A = np.array([[1.0, Ts], [0.0, 1.0]])
B = np.array([[0.0], [Ts / m]])
Q = np.diag([1.0, 0.1])
R = np.array([[0.01]])

K, P = lqr_gain(A, B, Q, R)
print(K)                                  # [[8.378 6.799]]
print(P)                                  # [[8.115 2.387] [2.387 1.699]]
print(np.allclose(P, solve_discrete_are(A, B, Q, R)))   # True
print(np.abs(np.linalg.eigvals(A - B @ K)))            # [0.838 0.838] < 1: stable

# Example 4: wheel speeds
rw, L = 0.0325, 0.1645
J = rw / 4 * np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-1/L, 1/L, -1/L, 1/L]])
print(np.linalg.pinv(J) @ [0.285, 0.095, 1.0])         # [0.79 16.75 6.63 10.91] (inputs rounded)
print(J @ [1, 1, -1, -1])                               # [0 0 0]: the "fighting" mode
```

Run it with the workspace sourced (`source install/setup.bash`) so `caging_control` is found.

---

## 11. Exercises (with answers)

Do them by hand first. Answers are hidden: click to open.

**E1. Sizes.** A system has 4 states and 2 inputs. Give the sizes of $A$, $B$, $Q$, $R$, $P$, $K$,
$B^TPB$ and $A - BK$.

<details><summary>Answer</summary>

$A$: 4×4, $B$: 4×2, $Q$: 4×4, $R$: 2×2, $P$: 4×4, $K$: 2×4, $B^TPB$: 2×2, $A - BK$: 4×4.
</details>

**E2. Quadratic form.** Compute $x^TQx$ for $x = [2, -1]^T$ and $Q = \begin{bmatrix}3 & 1\\1 & 2\end{bmatrix}$.

<details><summary>Answer</summary>

$Qx = [3\cdot2 + 1\cdot(-1),\ 1\cdot2 + 2\cdot(-1)]^T = [5,\ 0]^T$; $x^T(Qx) = 2\cdot5 + (-1)\cdot0 = 10$.
Check with the formula: $3\cdot4 + 2\cdot1\cdot2\cdot(-1) + 2\cdot1 = 12 - 4 + 2 = 10$ ✓.
</details>

**E3. Scalar LQR.** For $a = b = 1$, $q = 1$, $r = 1$: compute $P$ and $K$ with the closed
form, then do two iterations starting from $P_0 = 1$.

<details><summary>Answer</summary>

$P = (1 + \sqrt5)/2 = 1.618$, $K = 1.618/2.618 = 0.618$.
Iterations: $P_1 = 1 + 1 - 1/2 = 1.5$; $P_2 = 1 + 1.5 - 2.25/2.5 = 1.6$ (→ 1.618).
</details>

**E4. Scale invariance.** Show with the closed form that $(q, r) = (0.1, 0.8)$ and $(1, 8)$
give the same $K$. What happens to $P$?

<details><summary>Answer</summary>

$(1, 8)$: $P = (1 + \sqrt{1 + 32})/2 = 3.372$ (10× larger), $K = 3.372/11.372 = 0.2965$ (same).
$P$ scales with the cost, $K$ does not.
</details>

**E5. Cart, by hand.** In Example 2 compute $P_2$ (the second iteration) starting from
$P_1 = \begin{bmatrix}2 & 0.1\\0.1 & 0.2076\end{bmatrix}$.

<details><summary>Answer</summary>

$P_1B = [0.005,\ 0.01038]^T$; $B^TP_1B + R = 0.000519 + 0.01 = 0.010519$;
$(c) = \frac{1}{0.010519}\begin{bmatrix}0.000025 & 0.0000519\\0.0000519 & 0.0001078\end{bmatrix}
= \begin{bmatrix}0.0024 & 0.0049\\0.0049 & 0.0102\end{bmatrix}$;
$P_1 - (c) = \begin{bmatrix}1.9976 & 0.0951\\0.0951 & 0.1973\end{bmatrix}$;
$A^T(\cdot)A + Q = \begin{bmatrix}2.998 & 0.295\\0.295 & 0.336\end{bmatrix}$.
</details>

**E6. Differential drive.** With $d = 0.2$ m and $\theta = 0$, which $(v, \omega)$ makes the
point move with $\dot p = [0, 0.1]^T$ (straight to the left)? Describe the motion.

<details><summary>Answer</summary>

$M^{-1} = \begin{bmatrix}1 & 0\\0 & 5\end{bmatrix}$, so $v = 0$, $\omega = 0.5$ rad/s: the robot
turns in place to the left, and the point in front of it moves left. The axle itself does not
move sideways.
</details>

**E7. Mecanum.** The robot is at $\theta = 0$. The node publishes $\dot q_R = [0, 0.3, 0]^T$
(pure sideways to the left). Compute the four wheel speeds. Which wheels turn backward?

<details><summary>Answer</summary>

$\mu = J_R^+[0, 0.3, 0]^T = 30.77\cdot0.3\,[-1, 1, 1, -1]^T = [-9.23,\ 9.23,\ 9.23,\ -9.23]$ rad/s.
Front-left and rear-right turn backward.
</details>

**E8. Stability.** For our robot $A - BK = (1 - K)I$. For which values of $K$ is the closed loop
stable? Which $K$ gives an oscillating (sign-changing) error? Can LQR ever produce that?

<details><summary>Answer</summary>

Stable if $|1 - K| < 1$, i.e. $0 < K < 2$. Oscillating if $1 < K < 2$ (the error changes sign
each step: overshoot). LQR never does it here: $K = P/(r + P) < 1$ for any $r > 0$.
</details>
