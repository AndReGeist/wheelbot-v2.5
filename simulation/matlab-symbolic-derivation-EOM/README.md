# matlab_nonlinear_ODE

**FlywUni_symbolic_derive.m:** Derives the Euler-Lagrange equations of a uniwheel bot using the parameters set in "config_FlyUni" annd the "Langrange.m" defined in external libraries.

**FlywUni_symbolic_linearize.m:** Reshapes the symbolic ODE obtained from "FlywUni_symbolic_derive", brings it into the form for the standard matlab ODE solvers and saves it as 
save('EQS_matrices_nonlin.mat', 'M_nonlin', 'RHS_nonlin')
Also linearizes the set of equations and saves the linearization as 
save('EQS_matrices.mat', 'M', 'RHS')

**FlywUni_symbolic_analyze.m:** Loads the pre-computed dynamic equations, computes a trajectory using a standard ODE solver (RK45), and plots results.