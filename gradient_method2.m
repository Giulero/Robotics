function [ q_f ] = gradient_method2(q0, J_T, p_d, p, alpha)
%Returns the values of q given by gradient iteration method.
%Take as input q0(initial conf.), J_T(jacobian transpose), p_d(desidered
%position), p(position at actual configuration)
%for k=1:3
p_t = [0 0]';
while ((norm(p_d-p_t))/norm(p_d)) > 0.1
    q_f = q0+alpha*J_T*(p_d-p);
    q1 = q_f(1); q2 = q_f(2); q3 = q_f(3); q4 = q_f(4);
    p_t = [cos(q1)+cos(q1+q2)+cos(q1+q2+q3)+cos(q1+q2+q3+q4);
    sin(q1)+sin(q1+q2)+sin(q1+q2+q3)+sin(q1+q2+q3+q4)];
    q0=[q1, q2, q3, q4]';
end
disp('Gradiente finito');
end
