function [ q_f ] = newton( q0, q, J_PS, p_d, p )
%Returns final configuration finded by Newton method
%for k=1:6
p_t = [0 0]';
while ((norm(p_d-p_t))/norm(p_d)) > 0.01
    q_f = double(q0 + subs(J_PS, q, q0)*(p_d-subs(p, q, q0)));
    p_t = subs(p, q, q0);
    q0 = q_f;
end
end

