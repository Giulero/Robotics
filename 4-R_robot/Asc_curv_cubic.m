%Ascissa curvilinea cubica

%velocità
s_dot = [];
t=0;
T_tot = 8;
%s(t) = at^3+bt^2+ct+d;
%s_dot(t) = 3at^2+2bt+c;
% c = L/0.5;
% d = -3*c*T_tot/2;
syms a b c d real
eqns = [a*T_tot^5+b*T_tot^4+c*T_tot^3 == L,...
        5*a*T_tot^4+4*b*T_tot^3+3*c*T_tot^2 == 0,...
        20*a*T_tot^3+12*b*T_tot^2+6*c*T_tot == 0];
vars = [a b c d];
[A B C D] = solve(eqns, vars); 
A = double(A); B = double(B); C = double(C); D = double(D);    
dt = T_tot/interv;
%s_dot = 0;

while t < T_tot
    s_dot = [s_dot, abs(5*A*t^4+4*B*t^3+3*C*t^2+2*D*t)];
    t = t+dt;
end