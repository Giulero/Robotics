%% Giuseppe L'Erario - 4R arm robot
% Inverse kinematic + tajectory planning

clear all
clc
format long

disp('Initial configuration')
q1 = pi/2;
q2 = 0;
q3 = 0;
q4 = 0;
q0 = [q1, q2, q3, q4]'
disp('Initial position')
p0 =[cos(q1)+cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4);
    sin(q1)+sin(q1+q2)+sin(q1+q2+q3)+sin(q1+q2+q3+q4)]
disp('Required final position')
p_d = [3 2]'
q_dot_max = [0.2; 0.5; 0.2; 0.5];
V_max = 1;
a_max = 2;
interv = 150;
t = 0; %initial time
disp('Path length')
L = norm(p_d-p0)

%Bang-coast-bang
%B_C_B
Asc_curv_quintic
disp('Ascissa curvilinea calcolata')

%plot initial configuration
figure(1)
axis square
axis equal
axis([-4 4 -4 4])
grid on
line([p0(1), p_d(1)], [p0(2), p_d(2)], 'Color', 'red', 'LineStyle', '-.', 'LineWidth', 2); hold on;
p_b = plot(0,0,'.', 'Color', 'black'); %Origin

%
t=0; %return to initial time
i = 1;
q_dot_v = [];
v_dot = [];
norm_v_dot = [];
q = [q1, q2, q3, q4]';
q1_m(1) = q(1); q2_m(1)=q(2); q3_m(1)=q(3); q4_m(1)=q(4);
while t < T_tot
    p0_ =[cos(q1)+cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4);
    sin(q1)+sin(q1+q2)+sin(q1+q2+q3)+sin(q1+q2+q3+q4)];
    J=[-sin(q1)-sin(q1+q2)-sin(q1+q2+q3)-sin(q1+q2+q3+q4), -sin(q1+q2)-sin(q1+q2+q3)-sin(q1+q2+q3+q4),...
        -sin(q1+q2+q3)-sin(q1+q2+q3+q4), -sin(q1+q2+q3+q4);
        cos(q1)+cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4),cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4),...
        cos(q1+q2+q3)+cos(q1+q2+q3+q4), cos(q1+q2+q3+q4)];
    J_T = J';
    p_s_dot = (p_d-p0_)*s_dot(i); %-feval(@(t)[11.3137*t^2+2*(-550.9117*t)], t)
    if min(svd(J)) < 0.5
        q_dot = J_T*inv(0.7*eye(2)+J*J_T)*p_s_dot; %DLS method
    else
        q_dot = pinv(J)*p_s_dot;%+(eye(4)-pinv(J)*J)*pinv([1, 1, 1, 1])*0; %pseudoinverse method
    end
    %SNS
    q_dot = sns(q_dot, q_dot_max, J);
    %q_dot = pinv(J)*p_s_dot; %pseudoinverse method
    %q_dot = J_T*inv(0.3*eye(2)+J*J_T)*p_s_dot; %DLS method
    %q_dot = J_PS*p_s_dot+(eye(4)-J_PS*J)*[0.2 0 0 0]'; %Null space method
    q = q + q_dot*dt;
    q1 = q(1); q2=q(2); q3=q(3); q4=q(4);
    q1_m(i) = q(1); q2_m(i)=q(2); q3_m(i)=q(3); q4_m(i)=q(4);
    v_dot(:,i) = J * q_dot;
    q_dot_v(:,i) = q_dot;
    norm_v_dot(i) = norm(v_dot(:,i));
    joint1=[cos(q(1)); sin(q(1))];
    joint2=[cos(q(1))+cos(q(1)+q(2)); sin(q(1))+ sin(q(1)+q(2))];
    joint3=[cos(q(1))+cos(q(1)+q(2))+cos(q(1)+q(2)+q(3)); sin(q(1))+ sin(q(1)+q(2))+sin(q(1)+q(2)+q(3))];
    joint4=[cos(q(1))+cos(q(1)+q(2))+cos(q(1)+q(2)+q(3))+cos(q(1)+q(2)+q(3)+q(4)); sin(q(1))+ sin(q(1)+q(2))+sin(q(1)+q(2)+q(3))+sin(q(1)+q(2)+q(3)+q(4))];

    % Plot
    figure(1),
    l1 = line([0,joint1(1)], [0, joint1(2)], 'LineWidth', 2); hold on;
    l2 = line([joint1(1), joint2(1)],[joint1(2), joint2(2)], 'LineWidth', 2);
    l3 = line([joint2(1), joint3(1)],[joint2(2), joint3(2)], 'LineWidth', 2);
    l4 = line([joint3(1), joint4(1)],[joint3(2), joint4(2)], 'LineWidth', 2);
    p1 = plot(joint1(1), joint1(2), 'o', 'Color', 'red');
    p2 = plot(joint2(1), joint2(2), 'o', 'Color', 'red');
    p3 = plot(joint3(1), joint3(2), 'o', 'Color', 'red');
    p4 = plot(joint4(1), joint4(2), '>', 'Color', 'black');
    
    t=t+dt;
    pause(dt)
    if t < T_tot
        delete(l1), delete(l2), delete(l3), delete(l4), delete(p1), delete(p2), delete(p3), delete(p4),
    end
    i = i+1;
end    

%plot velocity
figure(2), hold on;
axis square
axis equal
%axis([0 T_tot 0 (max(s_dot)+1)]), 
grid on
plot(linspace(0,T_tot, length(s_dot)), s_dot);
title('Cartesian velocity-programmed'), xlabel('Time'), ylabel('Velocity')

figure(3), hold on;
plot(linspace(0, T_tot, length(q1_m)), q1_m),
plot(linspace(0, T_tot, length(q2_m)), q2_m),
plot(linspace(0, T_tot, length(q3_m)), q3_m),
plot(linspace(0, T_tot, length(q4_m)), q4_m), title('Joint coordinates'), xlabel('Time'), ylabel('Rad'), h=legend('$ q_1 $', '$ q_2 $', '$ q_3 $', '$ q_3 $');
set(h, 'Interpreter', 'latex');

figure(4), hold on;
plot(linspace(0,T_tot, length(v_dot)), v_dot(1,:)),
plot(linspace(0,T_tot, length(v_dot)), v_dot(2,:)),
plot(linspace(0,T_tot, length(v_dot)), norm_v_dot), title('Cartesian velocity-effective'), xlabel('Time'), ylabel('m/s');
f=legend('$ \dot{x} $', '$ \dot{y} $', 'Norm vel'); set(f, 'Interpreter', 'latex');

figure(5), hold on;
plot(linspace(0, T_tot, length(q_dot_v)), q_dot_v(1,:)),
plot(linspace(0, T_tot, length(q_dot_v)), q_dot_v(2,:)),
plot(linspace(0, T_tot, length(q_dot_v)), q_dot_v(3,:)),
plot(linspace(0, T_tot, length(q_dot_v)), q_dot_v(4,:)), title('Joint velocities'), xlabel('Time'), ylabel('Rad'), h=legend('$ q_1 $', '$ q_2 $', '$ q_3 $', '$ q_3 $');
set(h, 'Interpreter', 'latex');
