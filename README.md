# wheelbot-v2.5

This repository contains all files required to build a Wheelbot v2.5.

Project page:  [https://sites.google.com/view/wheelbot/start](https://sites.google.com/view/wheelbot/start)

## /CAD files
Contains the CAD files of the Wheelbot v2.5 as stl-files.

3D printer used: Markforged Onyx One
Material: Onyx
Layer height: 0.2 mm
Use default settings for all else.
See the following links for tips on printing:
https://markforged.com/resources/learn/design-for-additive-manufacturing-plastics-composites/3d-printing-strategies-for-composites/composites-3d-printing-design-tips
https://static.markforged.com/downloads/CompositesDesignGuide.pdf

The folder also contains the technical drawing for the cut copper rings forming the reaction wheel and the pdf *"wheelbot v2.5 assembly view.pdf"* that lets you interact with the complete Wheelbot's assembly (requires Adobe Acrobate reader).

## /motherboard circuitry
Contains the circuit layouts of the motherboard that connects to the Maevarm M2 and which also supplies the uDriver-v2 with power.

## /simulation/matlab-symbolic-derivation-EOM
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

## /simulation/simulink-model
Contains the simulink model used to tune the estimator and LQR controller. Recommended matlab version is R2020a.

The file *"s00_designLQR"* contains all settings of the simulation includng the exact estimates for the mass and inertia of the Wheelbot v2.5 which we obtained from CAD.

Before running *"s01_unicycle.slx"*, you need to run *"s00_designLQR"*.

## /firmware
Contains the firmware required to run Wheelbot v2.5

### /firmware/M2-on-wheelbot
Contains the firmware that runs on the Maevarm M2 that is attached to the motherboard of the Wheelbot.

### /firmware/M2-wifi-dongle
Contains the firmware that runs on the Maevarm M2 that is connected to the PC and that handles the wifi communication with the Wheelbot.

### /firmware/computer-python-interface
Python program running on Ubuntu 18.04 LTS collecting incoming data/outgoing commands from/to the wifi dongle.

