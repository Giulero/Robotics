function [ q_f ] = newton2( q0, J_PS, p_d, p )
%Returns final configuration finded by Newton method
%for k=1:6
p_t = [0 0]';
i = [0];
error = [(norm(p_d-p_t))/norm(p_d)];
while ((norm(p_d-p_t))/norm(p_d)) > 0.01 && max(i)<28
    q_f = q0 + J_PS*(p_d-p);
    q1 = q_f(1); q2 = q_f(2); q3 = q_f(3); q4 = q_f(4);
    p_t = [cos(q1)+cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4);
    sin(q1)+sin(q1+q2)+sin(q1+q2+q3)+sin(q1+q2+q3+q4)];
    q0=[q1, q2, q3, q4]';
    error = [error, (norm(p_d-p_t))/norm(p_d)];
    i = [i, i+1];
    disp(max(i))
    error
    i
end
figure(1);
plot(i, error);
disp('Newton finito');
end
