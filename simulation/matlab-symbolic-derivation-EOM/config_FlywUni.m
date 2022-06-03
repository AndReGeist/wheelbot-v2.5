% Intial Positions
q1_0 = 0; % initial roll angle of unicycle to ground before stand up in degree
q2_0 = 1; % initial pitch angle of unicycle to ground before stand up in degree
q3_0 = 0; % initial yaw angle of unicycle to ground before stand up in degree
q4_0 = 0; % initial wheel angle of unicycle to ground before stand up in degree
q5_0 = 0; % initial pendulum angle of unicycle to ground before stand up in degree

g = 9.81;
%% Body Dimensions, NORMALSIZE
Rw = 0.075; % Outer wheel radius of disc
r = Rw - 0.01; % Inner radius of disc
b_disc = 0.01; % Width of disc

Lc = 0.0885; %  Distance between wheel joint and chasis COG
Lcp = 0.0885; %  Distance between 'pendulum' joint and chasis COG 
Lp = 0.00; % Distance between 'pendulum' joint and 'pendulum COG'

%% Masses and inertias, NORMALSIZE and ALU
Mw = 0.9; % Wheel mass in kg
Mc = 3.5; % Chasis mass in kg
Mp = 0.9; % Flywheel mass in kg

% Wheel inertias
Iwx = 0.0008; % pointing in drive direction
Iwy = 0.0011; % Rotational DOF inertia ibn kg*m2
Iwz = 0.0008;

% Chasis Inertias
Icx = 0.018; % Roll chasis inertia
Icy = 0.018; % Pitch chasis inertia
Icz = 0.01; % Yaw chasis inertia

% Flywheel Inertias
Ipx = 0.0011; % Rotational DOF Inertia
Ipy = 0.0008;
Ipz = 0.0008; % Pointing 'up'