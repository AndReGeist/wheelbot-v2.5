clear all
close all

syms Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp
% M: Masses, I: Moment of Inertias, Rw: Wheel Radius, T: Actuation Torques
% Lc: Distance COG wheel to COG chasis, Lcp distance COG chasis to COG
% joint, Lp: distance COG joint to COG pendulum
syms t x y
syms q1 q2 q3 q4 q5
syms q1_d q2_d q3_d q4_d q5_d x_d y_d 
syms q1_dd q2_dd q3_dd q4_dd q5_dd x_dd y_dd
q_v = [q1; q2; q3; q4; q5];
dq_v = [q1_d; q2_d; q3_d; q4_d; q5_d];
ddq_v = [q1_dd; q2_dd; q3_dd; q4_dd; q5_dd];

%q_all = [q1; q2; q5; q1_d; q2_d; q5_d]; % All terms that will be neglected if they appear as a product
q_all = [q_v; dq_v];
%% Nonlinear System
load('EQS.mat', 'EQS') % Save full lagrange equation of motion
EQS = subs(EQS, [Lp], [0]); %negative sign because we want to take RHS to the right eventually

%%%% We need to write EQS = [M][alpha]-RHS as [M][alpha] = RHS 
%%%% were M is a 2x2 matrix alpha = [ud1 ud2] 
%%%% and RHS is a 2x1 matrix. 
RHS1 = -subs(EQS(1),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M11 =  simplify(subs(EQS(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS1,'Steps',50);
M12 =  subs(EQS(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS1;
M13 =  subs(EQS(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS1;
M14 =  subs(EQS(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS1;
M15 =  subs(EQS(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS1;

RHS2 = -subs(EQS(2),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M21 =  simplify(subs(EQS(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS2,'Steps',50);
M22 =  subs(EQS(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS2;
M23 =  subs(EQS(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS2;
M24 =  subs(EQS(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS2;
M25 =  subs(EQS(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS2;

RHS3 = -subs(EQS(3),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M31 =  simplify(subs(EQS(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS3,'Steps',50);
M32 =  subs(EQS(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS3;
M33 =  subs(EQS(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS3;
M34 =  subs(EQS(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS3;
M35 =  subs(EQS(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS3;

RHS4 = -subs(EQS(4),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M41 =  simplify(subs(EQS(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS4,'Steps',50);
M42 =  subs(EQS(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS4;
M43 =  subs(EQS(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS4;
M44 =  subs(EQS(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS4;
M45 =  subs(EQS(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS4;

RHS5 = -subs(EQS(5),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M51 =  simplify(subs(EQS(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS5,'Steps',50);
M52 =  subs(EQS(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS5;
M53 =  subs(EQS(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS5;
M54 =  subs(EQS(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS5;
M55 =  subs(EQS(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS5;

%%%%%%% Final system [M] [alpha] = [RHS] %%%%%%%%%%%%%%%
M_nonlin  = [M11 M12 M13 M14 M15; ...
      M21 M22 M23 M24 M25; ...
      M31 M32 M33 M34 M35; ...
      M41 M42 M43 M44 M45; ...
      M51 M52 M53 M54 M55];
RHS_nonlin = [RHS1; RHS2; RHS3; RHS4; RHS5];

save('EQS_matrices_nonlin.mat', 'M_nonlin', 'RHS_nonlin')

%% Linearization
EQS_tmp1  = subs(EQS, [sin(q_v) sin(2*q_v) cos(q_v) cos(2*q_v)], [q_v, 2*q_v, ones(length(q_v),1), ones(length(q_v),1)]);

% Set terms of higher order (>2) to zero
tmp = nchoosek(q_all, 2);
tmp2 = [];
for i=1:length(tmp)
    tmp2 = [tmp2, tmp(i, 1)*tmp(i, 2)];
end
EQS_tmp2 = subs(EQS_tmp1, [tmp2], [zeros(1, length(tmp2))]);

EQS_linearized = simplify(collect(simplify(EQS_tmp2,'Steps',30),[ddq_v]),300);

save('EQS_linearized.mat', 'EQS_linearized')

% Check linearized equation of motion by comparing it with equations from paper
eps_1 = Mc*Lc + Mp*(Lc+Lcp+Lp);
eps_2 =Mp*Lp;
eps_3 = (Mw+Mc+Mp)*Rw;
rho_1 = Mc*Lc^2 + Mp*(Lc+Lcp+Lp)^2;
rho_3 = Mp*(Lc+Lcp+Lp)*Lp;
sigma_1 = Iwy - Iwz;
sigma_2 = Icy-Icz;
sigma_3 = Ipy-Ipz;
sigma_4 = Icx-Icz+Ipx-Ipz;

test_eqn1 = eps_1*Rw*q2_dd + (Iwy + eps_3*Rw + eps_1*Rw)*q1*q3_dd + ...
    eps_2*Rw*q5*q3_dd + (Iwy + eps_3*Rw)*q4_dd - eps_1*Rw*q2*q3_d^2 + ...
    (Iwy+2*eps_3*Rw+2*eps_1*Rw)*q1_d*q3_d + 2*eps_2*Rw*q5_d*q3_d - Tw;

a11 = subs(EQS_linearized(4), [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
a12 = subs(test_eqn1, [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
isequaln(a11, a12)

test_eqn2 = (Icy + Ipy + rho_1)*q2_dd + (Icy+Ipy+rho_1+eps_1*Rw)*q1*q3_dd +...
    (sigma_3+rho_3)*q5*q3_dd + eps_1*Rw*q4_dd - (sigma_4+rho_1)*q2*q3_d^2+...
    (Icx+sigma_2+Ipx+sigma_3+2*rho_1+2*eps_1*Rw)*q1_d*q3_d + ...
    (Ipx+sigma_3+2*rho_3)*q5_d*q3_d - eps_1*g*q2 + Tw;
a21 = subs(EQS_linearized(2), [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
a22 = subs(test_eqn2, [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
isequaln(a21, a22)

test_eqn3 = (Iwx+Icx+Ipx+eps_3*Rw+rho_1+2*eps_1*Rw)*q1_dd + ...
    (Ipx+eps_2*Rw+rho_3)*q5_dd -(sigma_4+eps_1*Rw+rho_1)*q2*q3_dd - ...
    (sigma_1+sigma_2+sigma_3+eps_3*Rw+rho_1+2*eps_1*Rw)*q1*q3_d^2 - ...
    (sigma_3+rho_3+eps_2*Rw)*q5*q3_d^2 -(Icx+sigma_2+Ipx+sigma_3+2*rho_1+2*eps_1*Rw)*q2_d*q3_d - ...
    (Iwy+eps_3*Rw+eps_1*Rw)*q3_d*q4_d - (eps_3+eps_1)*g*q1 - eps_2*g*q5;
a31 = subs(EQS_linearized(1), [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
a32 = subs(test_eqn3, [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
isequaln(a31, a32)

test_eqn4 = (Ipx+eps_2*Rw+rho_3)*q1_dd + (Ipx+eps_2*Lp)*q5_dd -...
    (Ipx+rho_3)*q2*q3_dd - (sigma_3+eps_2*Rw+rho_3)*q1*q3_d^2 - ...
    (sigma_3+eps_2*Lp)*q5*q3_d^2 - (Ipx+sigma_3+2*rho_3)*q2_d*q3_d - ...
    eps_2*Rw*q3_d*q4_d - eps_2*g*q1 - eps_2*g*q5 - Tp;
a41 = subs(EQS_linearized(5), [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
a42 = subs(test_eqn4, [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
isequaln(a41, a42)

test_eqn5 = -(sigma_4+rho_1+eps_1*Rw)*q2*q1_dd + (Icy+Ipy+rho_1+eps_1*Rw)*q1*q2_dd + ...
    (sigma_3+rho_3)*q5*q2_dd - (Ipx+rho_3)*q2*q5_dd + (Iwz+Icz+Ipz)*q3_dd + ...
    (Iwy + eps_3*Rw + eps_1*Rw)*q1*q4_dd + eps_2*Rw*q5*q4_dd + Iwy*q1_d*q4_d + ...
    eps_1*Rw*q2*q3_d*q4_d;
a51 = subs(EQS_linearized(3), [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
a52 = subs(test_eqn5, [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp], [1:1:19]);
isequaln(a51, a52)

% Linearized system matrices
% The system dynamics are represented by three subsystems/modes:
% - Longitudinal/ Pitch mode: 
%       Wheel dynamics -> EQS(4) and Longit. chassis dynamics -> EQS(2)
% - Lateral/ Roll mode:
%       Lateral chassis dynamics -> EQS(1) and Pendulum dynamics -> EQS(5)        
% - Turning/ Yaw mode
%       Turning dynamics -> EQS(3)

%%%% We need to write these as [M][alpha] = RHS 
%%%% were M is a 2x2 matrix alpha = [ud1 ud2] 
%%%% and RHS is a 2x1 matrix. 
RHS1 = -subs(EQS_linearized(1),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M11 =  simplify(subs(EQS_linearized(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS1,'Steps',50);
M12 =  subs(EQS_linearized(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS1;
M13 =  subs(EQS_linearized(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS1;
M14 =  subs(EQS_linearized(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS1;
M15 =  subs(EQS_linearized(1),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS1;

RHS2 = -subs(EQS_linearized(2),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M21 =  simplify(subs(EQS_linearized(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS2,'Steps',50);
M22 =  subs(EQS_linearized(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS2;
M23 =  subs(EQS_linearized(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS2;
M24 =  subs(EQS_linearized(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS2;
M25 =  subs(EQS_linearized(2),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS2;

RHS3 = -subs(EQS_linearized(3),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M31 =  simplify(subs(EQS_linearized(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS3,'Steps',50);
M32 =  subs(EQS_linearized(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS3;
M33 =  subs(EQS_linearized(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS3;
M34 =  subs(EQS_linearized(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS3;
M35 =  subs(EQS_linearized(3),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS3;

RHS4 = -subs(EQS_linearized(4),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M41 =  simplify(subs(EQS_linearized(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS4,'Steps',50);
M42 =  subs(EQS_linearized(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS4;
M43 =  subs(EQS_linearized(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS4;
M44 =  subs(EQS_linearized(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS4;
M45 =  subs(EQS_linearized(4),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS4;

RHS5 = -subs(EQS_linearized(5),[ddq_v],[zeros(5,1)]); %negative sign because we want to take RHS to the right eventually
M51 =  simplify(subs(EQS_linearized(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[1 0 0 0 0]) + RHS5,'Steps',50);
M52 =  subs(EQS_linearized(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 1 0 0 0]) + RHS5;
M53 =  subs(EQS_linearized(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 1 0 0]) + RHS5;
M54 =  subs(EQS_linearized(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 1 0]) + RHS5;
M55 =  subs(EQS_linearized(5),[q1_dd q2_dd q3_dd q4_dd q5_dd],[0 0 0 0 1]) + RHS5;

%%%%%%% Final system [M] [alpha] = [RHS] %%%%%%%%%%%%%%%
M  = [M11 M12 M13 M14 M15; ...
      M21 M22 M23 M24 M25; ...
      M31 M32 M33 M34 M35; ...
      M41 M42 M43 M44 M45; ...
      M51 M52 M53 M54 M55];
RHS = [RHS1; RHS2; RHS3; RHS4; RHS5];

save('EQS_matrices.mat', 'M', 'RHS')
