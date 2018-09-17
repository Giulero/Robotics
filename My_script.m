% Giuseppe L'Erario - 4-R arm robot

clear all 
clc
format long

syms q1 q2 q3 q4 real

p =[cos(q1)+cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4);
    sin(q1)+sin(q1+q2)+sin(q1+q2+q3)+sin(q1+q2+q3+q4)];

q=[q1 q2 q3 q4]';

J=jacobian(p,q);
J_T = J';
J_PS = pinv(J);

%Initial configuration
disp('---Initial configuration---')
q0=[pi/2 pi/2 -pi/2 -pi/2]'
p0=subs(p, q, q0);
%Final Position
disp('---Required final position---')
p_d=[2 1]'

% q_g = gradient_method(q0, q, J_T, p_d, p, 0.5);
% q_f = newton(q_g, q, J_PS, p_d, p);
% 
% p_control=double(subs(p, q, q_f))
% 
% %position of various joints (joint4 is E-E))
% joint1=[cos(q_f(1)); sin(q_f(1))];
% joint2=[cos(q_f(1))+cos(q_f(1)+q_f(2)); sin(q_f(1))+ sin(q_f(1)+q_f(2))];
% joint3=[cos(q_f(1))+cos(q_f(1)+q_f(2))+cos(q_f(1)+q_f(2)+q_f(3)); sin(q_f(1))+ sin(q_f(1)+q_f(2))+sin(q_f(1)+q_f(2)+q_f(3))];
% joint4=[cos(q_f(1))+cos(q_f(1)+q_f(2))+cos(q_f(1)+q_f(2)+q_f(3))+cos(q_f(1)+q_f(2)+q_f(3)+q_f(4)); sin(q_f(1))+ sin(q_f(1)+q_f(2))+sin(q_f(1)+q_f(2)+q_f(3))+sin(q_f(1)+q_f(2)+q_f(3)+q_f(4))];
% 
% % Plot
% axis square
% axis equal
% axis([-3 3 -3 3])
% grid on
% line([0,joint1(1)], [0, joint1(2)]), hold on,
% line([joint1(1), joint2(1)],[joint1(2), joint2(2)])
% line([joint2(1), joint3(1)],[joint2(2), joint3(2)])
% line([joint3(1), joint4(1)],[joint3(2), joint4(2)])
% plot(joint1(1), joint1(2), 'o')
% plot(joint2(1), joint2(2), 'o')
% plot(joint3(1), joint3(2), 'o')
% plot(joint4(1), joint4(2), '*')
line([p0(1), p_d(1)], [p0(2), p_d(2)], 'Color', 'red', 'LineStyle', '-.', 'LineWidth', 2); hold on;
% Trajectory planning
for s=double(linspace(0,norm(p_d-p0), 10))
    p_s=p0+(p_d-p0)*s/norm(p_d-p0);
    q_g = gradient_method(q0, q, J_T, p_s, p, 0.5);
    q_f = newton(q_g, q, J_PS, p_s, p);
    
    %position of various joints (joint4 is E-E))
    joint1=[cos(q_f(1)); sin(q_f(1))];
    joint2=[cos(q_f(1))+cos(q_f(1)+q_f(2)); sin(q_f(1))+ sin(q_f(1)+q_f(2))];
    joint3=[cos(q_f(1))+cos(q_f(1)+q_f(2))+cos(q_f(1)+q_f(2)+q_f(3)); sin(q_f(1))+ sin(q_f(1)+q_f(2))+sin(q_f(1)+q_f(2)+q_f(3))];
    joint4=[cos(q_f(1))+cos(q_f(1)+q_f(2))+cos(q_f(1)+q_f(2)+q_f(3))+cos(q_f(1)+q_f(2)+q_f(3)+q_f(4)); sin(q_f(1))+ sin(q_f(1)+q_f(2))+sin(q_f(1)+q_f(2)+q_f(3))+sin(q_f(1)+q_f(2)+q_f(3)+q_f(4))];

    % Plot
    axis square
    axis equal
    axis([-3 3 -3 3])
    grid on
    l1 = line([0,joint1(1)], [0, joint1(2)], 'LineWidth', 2);
    l2 = line([joint1(1), joint2(1)],[joint1(2), joint2(2)], 'LineWidth', 2);
    l3 = line([joint2(1), joint3(1)],[joint2(2), joint3(2)], 'LineWidth', 2);
    l4 = line([joint3(1), joint4(1)],[joint3(2), joint4(2)], 'LineWidth', 2);
    p_b = plot(0,0,'.', 'Color', 'black');
    p1 = plot(joint1(1), joint1(2), 'o', 'Color', 'red');
    p2 = plot(joint2(1), joint2(2), 'o', 'Color', 'red');
    p3 = plot(joint3(1), joint3(2), 'o', 'Color', 'red');
    p4 = plot(joint4(1), joint4(2), '>', 'Color', 'black');
    
    q0=q_f;
    pause(0.05)
    if s ~= norm(p_d-p0)
        delete(l1), delete(l2), delete(l3), delete(l4), delete(p1), delete(p2), delete(p3), delete(p4)
    end
end


