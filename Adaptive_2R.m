%% Giuseppe L'Erario -- Adaptive control of a 2R robot

%% Robot parameters

m1 = 1;
m2 = 1;
l1 = 1;
l2 = 1;
d1 = l1/2;
d2 = l2/2;
I1 = 1/3;
I2 = 1/3;
g = 9.81;

q1 = pi/2;
q2 = 0;
p0 = [l1*cos(q1)+q2*cos(q1+q2); l1*sin(q1)+l2*sin(q1+q2)];
p_d = [1; 0.5]; % Desired POosition
t = 0; % Initial time
L = norm(p_d-p0);

q = [q1; q2];

q1_dot = 0;
q2_dot = 0;

a1 = m1*d1^2+I1+m2*l1;
a2 = m2*d2^2+I2;
a3 = l1*d2*m2;
a4 = m1*d1+m2*l1;
a5 = m2*d2;

q_dot = [q1_dot; q2_dot];

B = [[a1+a2+2*a3*cos(q2), a2+a3*cos(q2)];
     [a2+a3*cos(q2), a2]];

S1 = [-a3*sin(q2)*q2_dot; a3*sin(q2)*(q2_dot-q1_dot)];
S2 = [-a3*sin(q2)*(2*q1_dot+q2_dot); -a3*sin(q2)*q1_dot];

c = [-a3*sin(q2)*q2_dot*q1_dot + a3*sin(q2)*(q2_dot-q1_dot)*q2_dot;
     -a3*sin(q2)*(2*q1_dot+q2_dot)*q1_dot + -a3*sin(q2)*q1_dot*q2_dot];

g = g*[a4*cos(q1) + a5*cos(q1+q2); a5*cos(q1+q2)];

Asc_curv_quintic;

t = 0;
i = 1;
q_dot = [];
v_dot = [];
q = [q1; q2];

while t < T_tot
    p0 = [l1*cos(q1)+q2*cos(q1+q2); l1*sin(q1)+l2*sin(q1+q2)];
    J = [[-l1*sin(q1)+l2*sin(q1+q2), -l2*sin(q1+q2)];
         [l1*cos(q1)+l2*cos(q1+q2), l2*cos(q1+q2)]];
    J_dot = [[-l1*sin(q1)*q1_dot+l2*sin(q1+q2)*(q1_dot+q2_dot), -l2*sin(q1+q2)*(q1_dot+q2_dot)];
         [l1*cos(q1)*q1_dot+l2*cos(q1+q2)*(q1_dot+q2_dot), l2*cos(q1+q2)*(q1_dot+q2_dot)]];
    p_s_dot = (p_d-p0)*s_dot(i);
    p_s_ddot = (p_d-p0)*s_dot_dot(i);
    q_dot = pinv(J)*p_s_dot;
    q_dot_dot = pinv(J)*(p_s_ddot - J_dot*q_dot);
    q = q + q_dot*dt;
    q1 = q(1); q2 = q(2);
    t = t+dt;
    i = i + 1;
end
