% Lagrange is a function that calculate equations of motion (Lagrange's
% equations) d/dt(dL/d(dq))- dL/dq=0. It Uses  the Lagrangian that is a function that summarizes the
% dynamics of the system.  Symbolic Math Toolbox is required.
% 
% Equations=Lagrange(Lag,V)
%
% Lag = Lagrange of the system (symbolic).
% V   = System Variables (symbolic) [q1 dq1 ddq1 q2 dq2 ddq2... qn dqn
% ddqn].
% Equations   = [1 X DOF] (Degrees of freedom of the system).
%
%
% *******Examples*********
%     *Falling mass*
% 
% syms x dx ddx t m g     %Define the symbolic variables.
% L=0.5*m*dx^2 + m*g*x;  %Define the Lagragian.
% Equations=Lagrange(L,[x dx ddx]) %Calculate the equations
% 
% returns   m*ddx-g*m
%
% *Pendulum on a movable support*
% 
% syms x dx ddx theta dtheta ddtheta t m M   %Define the symbolic variables.
% 
% L=0.5*(M+m)+dx^2+ m*dx*l*dtheta*cos(theta)+ ...
% 0.5*m*l^2*dtheta^2+m*g*l*cos(theta)        %Define the Lagragian.
% Equations=Lagrange(L,[theta,dtheta,ddtheta,x,dx,ddx]) %Calculate the
% equations
% 
% returns   [  m*l*(ddx*cos(theta)+l*ddtheta+g*sin(theta)),
%              2*ddx+m*l*ddtheta*cos(theta)-m*l*dtheta^2*sin(theta)]

function [M]=Lagrange(Lag,V)
syms t f1(t) f2(t) f3(t) f4(t) f5(t) f6(t) f7(t) f8(t);
FF = [f1(t) f2(t) f3(t) f4(t) f5(t) f6(t) f7(t) f8(t)];
Var=length(V)/3;
Vt=V;
    for cont0=1:1:Var
        Vt(cont0*3-2) = FF(cont0);
        Vt(cont0*3-1) = diff(Vt((cont0*3)-2),t);
        Vt(cont0*3) = diff(Vt((cont0*3)-2),t,2);
    end
    for cont0=1:1:Var
        L1 = simplify(diff(Lag,V(cont0*3-1))); % Differentiate wrt general velocities
        L2 = simplify(diff(Lag,V(cont0*3-2))); % Differentiate wrt general coordinates
        Dposx = L1;

        for cont = 1:1:Var*3         
             Dposx = subs(Dposx,V(cont),Vt(cont));
        end
        L1=diff(Dposx,t);  % Differentiate wrt to time

        for cont=Var*3:-1:1         %
             L1=subs(L1,Vt(cont),V(cont));
        end
        L1F=L1-L2;
        L1F=simplify(expand(L1F));
        L1F=collect(L1F,Vt(cont0*3));%*****************
        M(cont0)=L1F;
    end
end