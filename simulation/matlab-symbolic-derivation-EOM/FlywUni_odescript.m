function zdot=FlywUni_odescript(t,z, M, RHS, Tw_var, Tp_var)          

t

% q1 = z(1);                          
% q1_d = z(2);                          
% q2 = z(3);                         
% q2_d = z(4);
% q3 = z(5);                         
% q3_d = z(6);
% q4 = z(7);                         
% q4_d = z(8);
% q5 = z(9);                         
% q5_d = z(10);

%load('EQS_matrices_withconstants.mat', 'M','RHS')
syms q1 q1_d q2 q2_d q3 q3_d q4 q4_d q5 q5_d Tw Tp
M_tmp = subs(M, [q1 q1_d q2 q2_d q3 q3_d q4 q4_d q5 q5_d Tw Tp],...
    [z(1) z(2) z(3) z(4) z(5) z(6) z(7) z(8) z(9) z(10) Tw_var Tp_var]);
RHS_tmp = subs(RHS, [q1 q1_d q2 q2_d q3 q3_d q4 q4_d q5 q5_d Tw Tp],...
    [z(1) z(2) z(3) z(4) z(5) z(6) z(7) z(8) z(9) z(10) Tw_var Tp_var]);

%eigs(double(M_tmp))
q_dd =  double(M_tmp) \ double(RHS_tmp);

q1_dd = double(q_dd(1));
q2_dd = double(q_dd(2));
q3_dd = double(q_dd(3));
q4_dd = double(q_dd(4));
q5_dd = double(q_dd(5));

% zdot = [q1_d q1_dd q2_d q2_dd q3_d q3_dd q4_d q4_dd q5_d q5_dd]';            
zdot = [z(2) q1_dd z(4) q2_dd z(6) q3_dd z(8) q4_dd z(10) q5_dd]';