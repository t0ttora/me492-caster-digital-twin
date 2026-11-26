# V2 formula sources and oral defence

This guide accompanies `Arche_V2_Controller_Design_EN.pdf`. References identify methods and starting equations; the V2-specific algebra is shown in the calculation PDF. No source is claimed to prescribe this robot's numerical gains.

## 1. Tracking errors, tracking law and Lyapunov function

Y. Kanayama, Y. Kimura, F. Miyazaki and T. Noguchi, **A Stable Tracking Control Method for a Non-Holonomic Mobile Robot**, IROS 1991, pp. 1236–1241. [DOI](https://doi.org/10.1109/IROS.1991.174669); [primary institutional archive](https://hdl.handle.net/10945/40159).

- Eq. (7): body-frame tracking-error dynamics.
- Eq. (8): nonlinear velocity-feedback structure.
- Eq. (10): position-error quadratic plus heading term proportional to `1 − cos(heading error)`.

**Important adaptation:** the original angular command is `omega_r + v_r (K_y e_y + K_theta sin(e_theta))`. V2 uses `omega_r + k_y v_r e_y + k_theta sin(e_theta)`. V2's heading correction is independent of reference speed. Thus the original controller and theorem must not be cited as if they apply unchanged to V2. The calculation PDF differentiates the candidate again for the modified law. A nonpositive derivative alone does not establish lateral convergence at zero reference speed.

**What to say:** “I used Kanayama's body-frame error formulation and Lyapunov candidate. I modified the heading-feedback term and recomputed the derivative for the implemented law.”

Access note: equation numbering and expressions were checked against indexed text from the primary NPS archive. Direct archive PDF retrieval was blocked; no specific printed page is claimed for these equations. These locators refer to the **1991** paper, not the similarly titled 1990 ICRA paper.

## 2. Tracking gains and time constants

K. J. Åström and R. M. Murray, **Feedback Systems: An Introduction for Scientists and Engineers**, second edition, online version **v3.1.5, 24 July 2020**. Chapter PDFs include a cover page, so printed and PDF page numbers differ.

| Method | Exact locator | How V2 uses it |
| --- | --- | --- |
| Standard second-order target | [Chapter 7, §7.3, Eq. (7.22), printed p. 7-17 / PDF p. 18](https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-statefbk_24Jul2020.pdf#page=18) | Match `s² + k_theta s + k_y v0²` to `s² + 2 zeta omega_n s + omega_n²`. |
| First-order time constant | [Chapter 6, Exercise 6.7, printed p. 6-36 / PDF p. 37](https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-linsys_24Jul2020.pdf#page=37) | For `e_dot = −k e`, compare with `−e/tau` to obtain `k = 1/tau`. |

**Derived here:** linearization of V2's specific tracking equations and coefficient matching, giving `k_theta = 2 zeta omega_n`, `k_y = omega_n²/v0²` and `k_x = 1/tau_x`.

**What to say:** “I selected a critically damped nominal lateral response and matched the linearized error polynomial to the standard second-order form. The book supplies the form; my selected bandwidth supplies the numerical gains.”

**Why −1, −1?** Setting `zeta = 1` and `omega_n = 1 rad/s` produces `(s+1)²`. Critical damping is the intended nonoscillatory local lateral response. The value `1 rad/s` is a chosen nominal bandwidth, not a measured constant or a uniquely optimal result. Critical damping of this local scalar response does not guarantee monotonic behaviour of the whole robot.

## 3. Wheel PI and discrete poles

Same book and online edition:

| Method | Exact locator | Relationship to V2 |
| --- | --- | --- |
| PI structure | [Chapter 11, §11.1, Eq. (11.1), printed p. 11-2 / PDF p. 3](https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-pid_24Jul2020.pdf#page=3) | Set the derivative gain to zero. |
| Sampled integration | [§11.5, Eq. (11.17), printed p. 11-22 / PDF p. 23](https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-pid_24Jul2020.pdf#page=23) | Starting point for a discrete integral. The source includes back-calculation; V2 uses a different anti-windup rule and update order. |
| Discrete stability | [Chapter 6, Exercise 6.8(b), printed p. 6-37 / PDF p. 38](https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-linsys_24Jul2020.pdf#page=38) | State eigenvalues must lie strictly inside the unit circle. |
| Conditional integration concept | [Chapter 11, Exercise 11.11, printed p. 11-26 / PDF p. 27](https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-pid_24Jul2020.pdf#page=27) | Source exercise gates on error size. V2 instead gates on saturation and whether error would unwind it; this exact condition is an implementation variant. |

**Derived here:** the update-integral-first recurrence, its characteristic polynomial, and coefficient matching. With `a = h Kp/J`, `b = h² Ki/J`, the polynomial is `z² − (2−a−b)z + 1−a`. Consequently `a = 1−p1 p2` and `b = (1−p1)(1−p2)`. These formulas are not claimed as numbered equations from the book.

The scalar plant `J Omega_dot = torque − load` is a nominal wheel-axis moment balance used for gain design, not the complete coupled V1 dynamics. Its inertia is the aligned V1 wheel value, not a parameter taken from the control references. The wheel-command conversion follows directly from no-slip rolling at axle offsets: `r Omega_L = v − b omega/2` and `r Omega_R = v + b omega/2`. Solving these two velocity constraints gives the PDF's wheel setpoints; this is a geometric derivation, not an additional tuning formula.

**Why 0.8 and 0.5?** They are selected positive real discrete poles: both decay without alternating signs in their individual modal responses. At `h = 0.01 s`, the equivalent modal time constants are `−h/ln(0.8) = 0.0448 s` and `−h/ln(0.5) = 0.0144 s`, faster than the selected outer-loop dynamics. This is a nominal cascade design choice. It has not been established as optimal or as a measured motor bandwidth; actuator saturation and the coupled mechanics limit that interpretation.

**What to say:** “I chose stable real discrete poles for a nominal inner loop faster than the pose loop, then matched the polynomial derived from the exact PI update order. The gains follow algebraically from the selected poles, wheel inertia and sample interval.”

## 4. Terminal position and heading

R. Siegwart and I. Nourbakhsh, **Autonomous Mobile Robots**, Chapter 3 teaching slides, **§3.6.2 Kinematic Position Control, PDF pages 35–37**. [Primary author-attributed slides](https://www.ladispe.polito.it/corsi/meccatronica/01CFI/2004-05/Slides/Slides_Autonomous_Mobile_Robots_3.pdf#page=35).

The source supplies Cartesian axle kinematics, the polar transformation and polar error dynamics, including the singularity at zero position error. **Notation differs:** their `beta` is an orientation error; V2's `beta` is the world bearing to the goal. The controller on slide 38 is not V2's controller.

**Derived here:** choose the forward/reverse effective heading, substitute `v = s k_rho rho cos(alpha)`, then cancel the polar coupling with `omega = k_alpha alpha + k_rho sin(alpha)cos(alpha)`. This gives `alpha_dot = −k_alpha alpha` and `rho_dot = −k_rho rho cos²(alpha)`. The PDF derives convergence for this ideal stage. Sequential final-heading regulation and switching thresholds are implementation choices, not copied source formulas.

For the final-heading Lyapunov calculation and interpretation, use Åström & Murray, [Chapter 5, Theorems 5.2 and 5.4](https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-dynamics_24Jul2020.pdf): Lyapunov stability and invariant-set convergence. V2's specific `V_g = 1−cos(e_g)` substitution and derivative are calculated in the PDF. The position-to-heading switch, torque limits and slip are outside that ideal stage proof.

**What to say:** “I started from standard polar kinematics and designed a cancellation term so that the bearing error follows a first-order decay. I then regulate final heading as a separate stage.”

## Concise opening explanation

“V1 provides the mechanical dynamics. V2 adds trajectory feedback, wheel-speed PI control and terminal pose recovery. The design uses established tracking and stability methods; I derive the modified controller equations and calculate gains from explicitly selected response targets.”

Numerical targets are design specifications. Literature references support the mathematics, not the claim that these targets are experimentally validated or uniquely correct for the physical robot.
