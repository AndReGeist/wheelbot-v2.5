clear all
close all

%% Add symbolic math toolbox
%addpath('./external_libraries')

%% Define system parameters
syms Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp
% M: Masses, I: Moment of Inertias, Rw: Wheel Radius, T: Actuation Torques
% Lc: Distance COG wheel to COG chasis, Lcp distance COG chasis to COG
% joint, Lp: distance COG joint to COG pendulum
syms t x y
syms q1 q2 q3 q4 q5
syms q1_d q2_d q3_d q4_d q5_d x_d y_d 
syms q1_dd q2_dd q3_dd q4_dd q5_dd x_dd y_dd
q_v = [q1; q2; q3; q4; q5; x; y];
dq_v = [q1_d; q2_d; q3_d; q4_d; q5_d; x_d; y_d];

Iw_w2 = diag([Iwx Iwy Iwz 0]);
Ic_c = diag([Icx Icy Icz 0]);
Ip_p = diag([Ipx Ipy Ipz 0]);

set_simplify = 10;

%% Define rotation and translation matrices
% w: wheel, c: chasis, p: pendulum
% q1: DOF roll, q2: DOF pitch, q3: DOF yaw, q4: DOF wheel angle, 
% q5: DOF pendulum angle

% IMPORTANT TO UNDERSTAND THE CODE: The code block below just states standard rotation and translation
% transformations which are used with the subs function

% Defining generic rotation matrices, use: test =  subs(Rotx,[q1],[DESIRED]);
Rotx = [1 0 0 0; 0 cos(q1) -sin(q1) 0; 0 sin(q1) cos(q1) 0; 0 0 0 1];
Roty = [cos(q1) 0 sin(q1) 0; 0 1 0 0; -sin(q1) 0 cos(q1) 0; 0 0 0 1];
Rotz = [cos(q1) -sin(q1) 0 0; sin(q1) cos(q1) 0 0; 0 0 1 0; 0 0 0 1];

transx = [1 0 0 Rw; 0 1 0 0; 0 0 1 0; 0 0 0 1];
transy = [1 0 0 0; 0 1 0 Rw; 0 0 1 0; 0 0 0 1];
transz = [1 0 0 0; 0 1 0 0; 0 0 1 Rw; 0 0 0 1];

% Transformation matrix from w2 (wheel CS) to cp (contact point CS):
T_cp_w2 = Rotx*transz;

T_g_cp = subs(Rotz,[q1],[q3]) + subs(transx,[Rw,1],[x,0]) + subs(transy,[Rw,1],[y,0]);

T_g_w2 = T_g_cp*T_cp_w2;

Pw_w2 = [0; 0; 0; 1];
Pw_g = T_g_w2*Pw_w2; % Wheel COG position

% Chasis CS Transformations to CS w2
T_w2_c = subs(Roty,[q1],[q2]) * subs(transz,[Rw],[Lc]);

T_g_c = T_g_w2*T_w2_c;

Pc_c = [0; 0; 0; 1]; % Position of chassis COG in c-CS
Pc_g = T_g_c*Pc_c;

Pturn_c = [0; 0; Lcp; 1]; % Position of chassis COG in c-CS
Pturn_g = T_g_c*Pturn_c;

% Transformation matrix from pendulum CS p to g-CS
T_c_p = subs(transz,[Rw],[Lcp]) *  subs(Rotx,[q1],[q5]) * subs(transz,[Rw],[Lp]);

T_g_p = T_g_c*T_c_p; 

Pp_p = [0; 0; 0; 1]; % Pendulum COG position
Pp_g = T_g_p*Pp_p;

% Compare position vector
Pp_g_comp1 = x + Rw*sin(q1)*sin(q3) + (Lc+Lcp)*cos(q2)*sin(q1)*sin(q3) + ...
    (Lc+Lcp)*sin(q2)*cos(q3) + Lp*cos(q5)*cos(q2)*sin(q1)*sin(q3) + ...
    Lp*cos(q5)*sin(q2)*cos(q3) + Lp*sin(q5)*cos(q1)*sin(q3);
a11 = simplify(subs(Pp_g(1), [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp q1 q2 q3 q4 q5 x y], [1:1:26]),50);
a12 = simplify(subs(Pp_g_comp1, [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp q1 q2 q3 q4 q5 x y], [1:1:26]),50);
isequaln(double(a11), double(a12))

Pp_g_comp2 = y-Rw*sin(q1)*cos(q5)-(Lc+Lcp)*cos(q2)*sin(q1)*cos(q3)+(Lc+Lcp)*sin(q2)*sin(q3) ...
    - Lp*cos(q5)*cos(q2)*sin(q1)*cos(q3) + Lp*cos(q5)*sin(q2)*sin(q3)...
    - Lp*sin(q5)*cos(q1)*cos(q3);
a21 = simplify(subs(Pp_g(2),    [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp q1 q2 q3 q4 q5 x y], [1:1:26]),50);
a22 = simplify(subs(Pp_g_comp2, [Mw Mc Mp Iwx Iwy Iwz Icx Icy Icz Ipx Ipy Ipz Rw Lc Lcp Lp g Tw Tp q1 q2 q3 q4 q5 x y], [1:1:26]),50);
disp('Test' + num2str(isequaln(double(a21), double(a22))))

save('position_vectors.mat', 'Pp_g', 'Pc_g', 'Pw_g', 'Pturn_g')
% Velocities: Compute time derivatives of body positions and transfos of
% angular rates
Vw_g = get_vel(Pw_g ,q_v,dq_v);
Vc_g = get_vel(Pc_g ,q_v,dq_v);
Vp_g = get_vel(Pp_g ,q_v,dq_v);

INV_tmp = T_g_w2\[0; 0; q3_d; 0];
OMEGAw_w2 = [0; q4_d; 0; 0] + [q1_d; 0; 0; 0] + simplify(INV_tmp, set_simplify);

INV_tmp = T_w2_c\[q1_d; 0; 0; 0];
INV_tmp2 = T_g_c\[0; 0; q3_d; 0];
OMEGAc_c = [0; q2_d; 0; 0] + simplify(INV_tmp, set_simplify) + simplify(INV_tmp2, set_simplify);

T_w2_p = T_w2_c*T_c_p;
INV_tmp = T_c_p\[0; q2_d; 0; 0];
INV_tmp2 = T_w2_p\[q1_d; 0; 0; 0];
INV_tmp3 = T_g_p\[0; 0; q3_d; 0];
OMEGAp_p = [q5_d; 0; 0; 0] + simplify(INV_tmp, set_simplify) + simplify(INV_tmp2, set_simplify) + simplify(INV_tmp3, set_simplify);

% Compute ENERGY TERMS in g-CS
TEw = simplify(0.5*Mw*transpose(Vw_g)*Vw_g + 0.5*transpose(OMEGAw_w2)*Iw_w2*OMEGAw_w2, set_simplify);
PEw = simplify(Mw*g*[0 0 1 0]*Pw_g, set_simplify);

TEc = simplify(0.5*Mc*transpose(Vc_g)*Vc_g + 0.5*transpose(OMEGAc_c)*Ic_c*OMEGAc_c, set_simplify);
PEc = simplify(Mc*g*[0 0 1 0]*Pc_g, set_simplify);

TEp = simplify(0.5*Mp*transpose(Vp_g)*Vp_g + 0.5*transpose(OMEGAp_p)*Ip_p*OMEGAp_p, set_simplify);
PEp = simplify(Mp*g*[0 0 1 0]*Pp_g, set_simplify);

TE = TEw + TEc + TEp;
PE = PEw + PEc + PEp;

% Compute dynamic ODEs through Lagrange
L = simplify(TE - PE, set_simplify);

V = [q1 q1_d q1_dd q2 q2_d q2_dd q3 q3_d q3_dd q4 q4_d q4_dd q5 q5_d q5_dd ...
    x x_d x_dd y y_d y_dd];
M = Lagrange(L,V);  % The function Lagrange outputs the EOM as m*ddx-g*m
%Qi = [0 -Tw 0 Tw Tp]
syms lambda1 lambda2
EQS_tmp = [M(1) M(2)+Tw M(3) M(4)-Tw+Rw*cos(q3)*lambda1+Rw*sin(q3)*lambda2 M(5)-Tp];
EQS_tmp2 = subs(EQS_tmp, [lambda1 lambda2], [M(6) M(7)]); % Equation of motion: 0 = EQS (5 DOF)

% Use dynamic constraints to get rid of x and y 
dx_subst = Rw*q4_d*cos(q3);
ddx_subst = Rw*q4_dd*cos(q3) - Rw*q4_d*q3_d*sin(q3);
dy_subst = Rw*q4_d*sin(q3);
ddy_subst = Rw*q4_dd*sin(q3) + Rw*q4_d*q3_d*cos(q3);
EQS = simplify(subs(EQS_tmp2, [x_d x_dd y_d y_dd], [dx_subst ddx_subst dy_subst ddy_subst]),100); % Equation of motion: 0 = EQS (5 DOF)
TE = simplify(subs(TE, [x_d x_dd y_d y_dd], [dx_subst ddx_subst dy_subst ddy_subst]),100); 
PE = simplify(subs(PE, [x_d x_dd y_d y_dd], [dx_subst ddx_subst dy_subst ddy_subst]),100); 

save('EQS.mat', 'EQS') % Save full lagrange equation of motion
save('EnergyTerms.mat', 'TE', 'PE') % Save full lagrange equation of motion


