global Ts
Ts = 1/100;                % sampling time [s]
StopTime = 15;

% Mechanical parameter setting
flag_wheelbot = "v2_5";

% Estimator settings
flag_UseAccNoise = 1; % Add accelerometer noise? (yes=1, no=0)
flag_UseGyroNoise = 1; % Add gyro noise? (yes=1, no=0)
flag_UsePivotAcc = 1; % Feed pivot acc to estimator? (yes=1, no=0)

                      
% Controller settings
flag_UsePitchEst = 1; % Use pitch estimate for control? (yes=1, no=0)
flag_UseRollEst = 1; % Use roll estimate for control? (yes=1, no=0)
flag_UseYawEst = 1; % Use yaw estimate obtained by integrating gyro rates
flag_UseRateEst = 1; % Use pose rate estimate from gyro (tranformed using euler coords)
flag_useControlDelay = 0; % Add two timestep delay to system? (yes=1, no=0)

flag_AddRollBias = 0; % Use roll estimate for control? (yes=1, no=0)
flag_AddPitchBias = 0; % Use roll estimate for control? (yes=1, no=0)
roll_bias = deg2rad(1); % Use roll estimate for control? (yes=1, no=0)
pitch_bias = deg2rad(1); % Use roll estimate for control? (yes=1, no=0)

% Intial Positions
global x0 q1_0 q2_0 q3_0 q4_0 q5_0 q5_d_0
q1_0 = -deg2rad(36); % initial roll (alpha) angle of unicycle to ground before stand up in degree
q2_0 = deg2rad(0); % initial pitch (beta) angle of unicycle to ground before stand up in degree
q3_0 = 0; % initial yaw (delta) angle of unicycle to ground before stand up in degree
q4_0 = 0; % initial wheel angle (theta) of unicycle to ground before stand up in degree
q5_0 = 0; % initial pendulum angle (gamma) of unicycle to ground before stand up in degree

q5_d_0 = convangvel(2000, 'rpm', 'rad/s'); % 250

x0 = [q1_0; 0; q2_0; 0; q3_0; 0; ...
      q4_0; 0; q5_0; q5_d_0; 0; 0; 0; 0];
g = 9.81;

%% Motor parameters
flag_saturateTorque = 1;
stall_torque = 1.6; %Nm
noload_rate = convangvel(2700, 'rpm', 'rad/s');

%% Sensor parameters
variance_IMUacc = 0.0049;
mean_IMUgyro = -0.0065;
variance_IMUgyro = 6.3956e-04;

estimator_bias = 0.5*(2*pi/360); % Encoder bias in degree
%noiserad_ndist.mu
%noiserad_ndist.sigma
%estimator_UsePulse = 0;
%estimator_PulseAmplitude = 0.06;

%% Wheelbot mechanical parameters
if flag_wheelbot == "v2_5"
        %% IMU positions
    B_p1 = [ -0.048; -0.048; 0.119];
    B_p2 = [ 0.048; 0.048; 0.119];
    B_p3 = [ 0.048; -0.048; -0.004];
    B_p4 = [-0.048; 0.048; -0.004];
    Bp = [B_p1 B_p2 B_p3 B_p4];
    P = [[1;B_p1] [1;B_p2] [1;B_p3] [1;B_p4]];
    X = (P.')/(P*P.'); % Estimator IMU position matrix 
    Xp = X(:,1); % With Sebastian's estimator only first column is required for estimating gravity vector

    %% Dimensions
    Rw = 0.053; % Outer wheel radius of disc
    r = Rw - 0.01; % Inner radius of disc (only for visualization)
    b_disc = 0.01; % Width of disc (only for visualization)

    Lc = 0.055; %  Distance between wheel joint and chasis COG
    Lcp = 0.055; %  Distance between 'flywheel' joint and chasis COG 
    Lp = 0.00; % Distance between 'pendulum' joint and 'pendulum COG'
    Lw = 0.07; % Width of chassis (only for visualization)

    %% Masses and inertias
    Mw = 0.3; % Wheel mass in kg
    Mc = 0.75; % Chasis mass in kg
    Mp = 0.3; % Flywheel mass in kg

    % Wheel inertias
    Iwx = 0.000259; % pointing in drive direction
    Iwy = 0.000503; % Rotational DOF inertia ibn kg*m2
    Iwz = 0.000259;

    % Chasis Inertias
    Icx = 0.001746; % Roll chasis inertia
    Icy = 0.00181; % Pitch chasis inertia
    Icz = 0.001585; % Yaw chasis inertia

    % Flywheel Inertias
    Ipx = 0.000503; % Rotational DOF Inertia
    Ipy = 0.000259;
    Ipz = 0.000259; % Pointing 'up'
end