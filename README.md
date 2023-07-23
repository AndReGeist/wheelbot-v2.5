# wheelbot-v2.5

This repository contains all files required to build a Wheelbot v2.5.

Project page:  [https://sites.google.com/view/wheelbot/start](https://sites.google.com/view/wheelbot/start)

## [/CAD files](https://github.com/AndReGeist/wheelbot-v2.5/tree/main/CAD%20files)
Contains the CAD files of the Wheelbot v2.5 as stl-files.

3D printer used: Markforged Onyx One <br/>
Material: Onyx <br/>
Layer height: 0.2 mm <br/>
Use default settings for all else. <br/>
See the following links for tips on printing: <br/>
https://markforged.com/resources/learn/design-for-additive-manufacturing-plastics-composites/3d-printing-strategies-for-composites/composites-3d-printing-design-tips <br/>
https://static.markforged.com/downloads/CompositesDesignGuide.pdf

The folder also contains the technical drawing for the cut copper rings forming the reaction wheel and the pdf *"wheelbot v2.5 assembly view.pdf"* that lets you interact with the complete Wheelbot's assembly (requires Adobe Acrobate reader).

## [/motherboard circuitry](https://github.com/AndReGeist/wheelbot-v2.5/tree/main/motherboard%20circuitry)
Contains the circuit layouts of the motherboard that connects to the Maevarm M2 and which also supplies the uDriver-v2 with power.

## [/simulation/matlab-symbolic-derivation-EOM](https://github.com/AndReGeist/wheelbot-v2.5/tree/main/simulation/matlab-symbolic-derivation-EOM)
Contains the matlab files used to symbolically derive the EOM of the Wheelbot.

**FlywUni_symbolic_derive.m:** Derives the Euler-Lagrange equations of a unicycle reaction wheel robot using the parameters set in "config_FlyUni" and the "Langrange.m" defined in external libraries.

**FlywUni_symbolic_linearize.m:** Reshapes the symbolic ODE obtained from "FlywUni_symbolic_derive", brings it into the form for the standard matlab ODE solvers and saves it as 
```matlab
save('EQS_matrices_nonlin.mat', 'M_nonlin', 'RHS_nonlin')
```
Also linearizes the set of equations and saves the linearization as 
```matlab
save('EQS_matrices.mat', 'M', 'RHS')
```

**FlywUni_symbolic_analyze.m:** Loads the pre-computed dynamic equations, computes a trajectory using a standard ODE solver (RK45), and plots trajectory and total energy of the system.

## [/simulation/simulink-model](https://github.com/AndReGeist/wheelbot-v2.5/tree/main/simulation/simulink-model)
Contains the simulink model used to tune the estimator and LQR controller. Recommended matlab version is R2020a.

The file *"s00_config"* contains the simulation settings including the exact estimates for the mass and inertia of the Wheelbot v2.5 which we obtained from CAD.

Before running *"s01_unicycle.slx"*, you need to run *"s00_designLQR"*.

## [/firmware](https://github.com/AndReGeist/wheelbot-v2.5/tree/main/firmware)
Contains the firmware required to run Wheelbot v2.5

### /firmware/M2-on-wheelbot
Contains the firmware that runs on a Maevarm M2 that is attached to the motherboard of the Wheelbot.

### /firmware/M2-wifi-dongle
Contains the firmware that runs on a Maevarm M2 being connected to a PC and that handles the wifi communication with the Wheelbot.

### /firmware/computer-python-interface
Python program running on Ubuntu 18.04 LTS collecting incoming data/outgoing commands from/to the wifi dongle.

# Erratum
In the initial publication, in Equation (3), the transform from averaged body rates ${}^{\text{B}}\omega_i$ to Euler rates was incorrectly denoted as

```math
\begin{bmatrix}
\dot{q}_{1, \mathrm{G}} \\
\dot{q}_{2, \mathrm{G}} \\
\dot{q}_{3, \mathrm{G}}
\end{bmatrix} = \begin{bmatrix}
e_1^{\mathrm{T}} R_2 \\
e_2^{\mathrm{T}} \\
e_3^{\mathrm{T}} R_1 R_2
\end{bmatrix} \sum_{k=1}^4 \frac{{ }^B \omega_i(k)}{4},
```

corresponding to

```math
\left[\begin{array}{c}
\dot{q}_{1, \mathrm{G}} \\
\dot{q}_{2, \mathrm{G}} \\
\dot{q}_{3, \mathrm{G}}
\end{array}\right]=\left[\begin{array}{c}
R_2^T e_1 &
e_2 &
R_2^T R_1^T e_3
\end{array}\right]^{\top} \sum_{i=1}^4 \frac{{ }^B \omega_i(k)}{4}.
```

However, the correct transform from averaged body rates ${}^{\text{B}}\omega_i$ to Euler rates is

```math
\left[\begin{array}{c}
\dot{q}_{1, \mathrm{G}} \\
\dot{q}_{2, \mathrm{G}} \\
\dot{q}_{3, \mathrm{G}}
\end{array}\right]=\left[\begin{array}{c}
R_2^T e_1 &
e_2 &
R_2^T R_1^T e_3
\end{array}\right]^{-1} \sum_{i=1}^4 \frac{{ }^B \omega_i(k)}{4}.
```

While we implemented the faulty transform in the Wheelbot's state estimation routine (see [main.c](https://github.com/AndReGeist/wheelbot-v2.5/blob/main/firmware/M2-on-wheelbot/src/main.c), Line 340),  the error had not been noticed during experimentation as the robot remained close to its upright equilibrium. In turn, $q_1$ and $q_2$ remained near zero such that both transforms became almost identical, as

```math
\left[\begin{array}{c}
R_2^T e_1 &
e_2 &
R_2^T R_1^T e_3
\end{array}\right]^{\top} =  \left[\begin{array}{c}
\cos(q_2) & 0 & \sin(q_2) \\
0 & 1 & 0 \\
-\cos(q_1) \sin(q_2) & \sin(q_1) & \cos(q_1) \cos(q_2)
\end{array}\right],
```

```math
\left[\begin{array}{c}
R_2^T e_1 &
e_2 &
R_2^T R_1^T e_3
\end{array}\right]^{-1} = \left[\begin{array}{c}
 \cos(q_2) & 0 & \sin(q_2) \\
 \tan(q_1) \sin(q_2) & 1 & -\tan(q_1) \cos(q_2) \\ 
 -\sin(q_2) / \cos(q_1) & 0 & \cos(q_2) / \cos(q_1)
\end{array}\right].
```

Importantly, **our results on tilt estimation using accelerometers** as depicted in Figure 10 **are not affected by this error**. In the first experiment (Figure 10, left) the robot's tilt angles were kept at zero. In the second experiment (Figure 10, right), $q_1 \approx 0$ such that $\tan(q_1) \approx 0$. In turn, both transforms deviated only marginally from each other in both experiments.

We added a [Jupyter notebook](https://github.com/AndReGeist/wheelbot-v2.5/blob/main/erratum_bodyrate_transform.ipynb) to the projects Github repository detailing the calculation of the transform from body rates to Euler rates. 
