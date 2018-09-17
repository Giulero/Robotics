%Ascissa curvilinea cubica

%velocità
s_dot = [];
s_dot_dot = [];
t=0
T_tot = 3;
%s(t) = at^3+bt^2+ct+d;
%s_dot(t) = 3at^2+2bt+c;
c = L/0.5;
d = -3*c*T_tot/2;
dt = T_tot/interv;
%s_dot = 0;

while t < T_tot
    s_dot = [s_dot, 3*c*t^2+2*d*t];
    s_dot_dot = [s_dot_dot, 6*c*t];
    t = t+dt;
end