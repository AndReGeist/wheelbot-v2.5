clear all
close all


run(which('./config_FlywUni.m'))
const_values = [Mw; Mc; Mp;...
             Iwx; Iwy; Iwz;...
             Icx; Icy; Icz;...
             Ipx; Ipy; Ipz;...
             Rw; Lc; Lcp; Lp; g];
         
syms Mw Mc Mp ... % Body masses
     Iwx Iwy Iwz ... % Body inertias
     Icx Icy Icz ...
     Ipx Ipy Ipz ...
     Rw Lc Lcp Lp g ... % System geometry and physical constants
     Tw Tp % Actuation Torques
                
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


load('EQS_matrices_nonlin.mat', 'M_nonlin', 'RHS_nonlin'); % Nonlinear simulation
M = M_nonlin;
RHS = RHS_nonlin;

%load('EQS_matrices.mat', 'M', 'RHS') % Linear simulation

% The system dynamics are represented by three subsystems/modes:
% - Longitudinal/ Pitch mode: 
%       Wheel dynamics -> EQS(4) and Longit. chassis dynamics -> EQS(2)
% - Lateral/ Roll mode:
%       Lateral chassis dynamics -> EQS(1) and Pendulum dynamics -> EQS(5)        
% - Turning/ Yaw mode
%       Turning dynamics -> EQS(3)

constants = [Mw; Mc; Mp;...
             Iwx; Iwy; Iwz;...
             Icx; Icy; Icz;...
             Ipx; Ipy; Ipz;...
             Rw; Lc; Lcp; Lp; g];
         
% const_values = [1.42; 6.34; 0.85; ...
%                 0.01; 0.02; 0.01; ...
%                 0.17; 0.11; 0.08; ...
%                 0.03; 0.03; 0.0003;  ...
%                 0.15; 0.15; 0.13; 0.41; 9.81];

% Normalsize, Aluminium
% Roll up Tw = 3 Nm
% const_values = [0.8; 4; 0.8; ...
%                 0.0006; 0.001; 0.0006; ...
%                 0.015; 0.015; 0.008; ...
%                 0.001; 0.0006; 0.0006;  ...
%                 0.075; 0.088; 0.088; 0.0; 9.81];
            
% Normalsize, Aluminium , Worst-case (Gehause schwer, raeder klein)
% Roll up Tw = 3 Nm
%const_values = [1.5; 5; 1.5; ...
%                0.0006; 0.001; 0.0006; ...
%                0.025; 0.025; 0.01; ...
%                0.001; 0.0006; 0.0006;  ...
%                0.075; 0.088; 0.088; 0.0; 9.81];
M = subs(M, [constants], [const_values]);
RHS = subs(RHS, [constants], [const_values]);

% Initial conditions and other settings.
%framespersec=1000;  %if view is not speeded or slowed in dbpend_animate
%T=1;             %duration of animation  in seconds
%tspan=linspace(0,T,T*framespersec);
tf = 1;
tspan = 0:0.01:tf;

%z0 =[q1 dq1 q2 dq2 q3 dq3 q4 dq4 q5 dq5 Tw Tp]
%z0 = [deg2rad(20) 0  deg2rad(20)   0  0  0   0  0  0 0]';
% Roll-up
z0 = [deg2rad(q1_0)  0  deg2rad(q2_0)  0  deg2rad(q3_0) 0 ... % Roll, Pitch, Yaw 
    deg2rad(q4_0) 0 deg2rad(q5_0) 0]'; % Wheel and Disc angle

%options=odeset('abstol',1e-2,'reltol',1e-4);
odefun = @(t,z) FlywUni_odescript(t,z, M, RHS, 0, 0);
[t, z] = ode45(odefun ,tspan, z0);

save('symbolic_results.mat', 't', 'z')

%% Plot states
figure(1)
ax1 = subplot(2,5,1);
plot(t,rad2deg(z(:,1)));
title(ax1,'Roll')
xlabel('time (s)'); ylabel('alpha, q1 (deg)');

ax1 = subplot(2,5,2);
plot(t,rad2deg(z(:,3)));
title(ax1,'Pitch')
xlabel('time (s)'); ylabel('betha, q2 (deg)');

ax1 = subplot(2,5,3);
plot(t,rad2deg(z(:,5)));
title(ax1,'Yaw')
xlabel('time (s)'); ylabel('delta, q3 (deg)');

ax1 = subplot(2,5,4);
plot(t,rad2deg(z(:,7)));
title(ax1,'Wheel angle')
xlabel('time (s)'); ylabel('theta, q4 (deg)');

ax1 = subplot(2,5,5);
plot(t,rad2deg(z(:,9)));
title(ax1,'Pendulum angle')
xlabel('time (s)'); ylabel('gamma, q5 (deg)');


ax1 = subplot(2,5,6);
plot(t,rad2deg(z(:,2)));
title(ax1,'Roll velocity')
xlabel('time (s)'); ylabel('alpha dot, q1 dot (deg/s)');

ax1 = subplot(2,5,7);
plot(t,rad2deg(z(:,4)));
title(ax1,'Pitch velocity')
xlabel('time (s)'); ylabel('betha dot, q2 dot (deg/s)');

ax1 = subplot(2,5,8);
plot(t,rad2deg(z(:,6)));
title(ax1,'Yaw velocity')
xlabel('time (s)'); ylabel('delta dot, q3 dot (deg/s)');

ax1 = subplot(2,5,9);
plot(t,rad2deg(z(:,8)));
title(ax1,'Wheel angular velocity')
xlabel('time (s)'); ylabel('theta dot, q4 dot (deg/s)');

ax1 = subplot(2,5,10);
plot(t,rad2deg(z(:,10)));
title(ax1,'Pendulum angular velocity')
xlabel('time (s)'); ylabel('gamma dot, q5 dot (deg/s)');

% Trajectory plot
Rw=0.15;

x_tmp = zeros(length(t),1);
y_tmp = zeros(length(t),1);
Pp_g_tmp = zeros(length(t),3);
Pc_g_tmp = zeros(length(t),3);
Pw_g_tmp = zeros(length(t),3);
Pturn_g_tmp = zeros(length(t),3);

dx = Rw.*z(:,8).*cos(z(:,5));
dy = Rw.*z(:,8).*sin(z(:,5));

load('position_vectors.mat', 'Pp_g', 'Pc_g', 'Pw_g', 'Pturn_g')
syms Rw x y
for i=1:length(t)
     x1 =+ dx(i)*tspan(2);
     x_tmp(i) = double(x1);
     
     y1 =+ dy(i)*tspan(2);
     y_tmp(i) = double(y1);
     
     Pp_g_prev = subs(Pp_g, [q1 q2 q3 q4 q5 Rw Lc Lcp Lp x y], ...
         [z(i,1) z(i,3) z(i,5) z(i,7) z(i,9) const_values(13:16)' x1 y1]);
     Pp_g_tmp(i,:) = double(Pp_g_prev(1:3));
     
     Pturn_g_prev = subs(Pturn_g, [q1 q2 q3 q4 q5 Rw Lc Lcp Lp x y], ...
         [z(i,1) z(i,3) z(i,5) z(i,7) z(i,9) const_values(13:16)' x1 y1]);
     Pturn_g_tmp(i,:) = double(Pturn_g_prev(1:3));
     
     %Pc_g_prev = subs(Pc_g, [q1 q2 q3 q4 q5 Rw Lc Lcp Lp x y], ...
     %    [z(i,1) z(i,3) z(i,5) z(i,7) z(i,9) 0.15 0.15 0.13 0.41 x1 y1]);
     %Pc_g_tmp(i,:) = double(Pc_g_prev(1:3));
     
     Pw_g_prev = subs(Pw_g, [q1 q2 q3 q4 q5 Rw Lc Lcp Lp x y], ...
         [z(i,1) z(i,3) z(i,5) z(i,7) z(i,9) const_values(13:16)' x1 y1]);
     Pw_g_tmp(i,:) = double(Pw_g_prev(1:3));
     
     % Check Spatial Point distance
%      disp('p_turn'), 0.41
%      norm(Pp_g_tmp(i,:) - Pturn_g_tmp(i,:))
%      disp('c_w'), 0.15
%      norm(Pc_g_tmp(i,:) - Pw_g_tmp(i,:))
%      disp('w_cp'), 0.15
%      norm(Pw_g_tmp(i,:) - [x_tmp(i),y_tmp(i),0])

end

% Plot joint positions over time
figure(2)
% plot3([x_tmp, Pw_g_tmp(:,1), Pc_g_tmp(:,1), Pturn_g_tmp(:,1), Pp_g_tmp(:,1)]', ...
%     [y_tmp, Pw_g_tmp(:,2), Pc_g_tmp(:,2), Pturn_g_tmp(:,2), Pp_g_tmp(:,2)]', ...
%     [zeros(length(t),1), Pw_g_tmp(:,3), Pc_g_tmp(:,3), Pturn_g_tmp(:,3), Pp_g_tmp(:,3)]','-o','MarkerSize',10)
plot3([x_tmp, Pw_g_tmp(:,1), Pturn_g_tmp(:,1), Pp_g_tmp(:,1)]', ...
    [y_tmp, Pw_g_tmp(:,2), Pturn_g_tmp(:,2), Pp_g_tmp(:,2)]', ...
    [zeros(length(t),1), Pw_g_tmp(:,3), Pturn_g_tmp(:,3), Pp_g_tmp(:,3)]','-o','MarkerSize',10)
daspect([1 1 1])
xlabel('x in m')
ylabel('y in m')
zlabel('z in m')
legend('GCP','Pendulum COG','Chassis COG','Wheel COG')
grid on

%% Check energy
load('EnergyTerms.mat', 'TE', 'PE')
TE = subs(TE, [constants], [const_values]);
PE = subs(PE, [constants], [const_values]);
TE_tmp = zeros(length(t),1);
PE_tmp = zeros(length(t),1);

for i=1:length(t)
    i
    TE_tmp(i) = subs(TE, [q1 q1_d q2 q2_d q3 q3_d q4 q4_d q5 q5_d], [z(i,:)]);
    PE_tmp(i) = subs(PE, [q1 q1_d q2 q2_d q3 q3_d q4 q4_d q5 q5_d], [z(i,:)]);
end
TotalEnergy = TE_tmp + PE_tmp;
TotalEnergy_diff = diff(TotalEnergy);

figure(3)
ax1 = subplot(2,1,1);
plot(t(1:end), TotalEnergy)
ax2 = subplot(2,1,2);
plot(t(1:end-1), TotalEnergy_diff)
xlabel('t in s')
ylabel('Change in System Energy')
