function [ q_f ] = gradient_method(q0, q, J_T, p_d, p, alpha)
%Returns the values of q given by gradient iteration method.
%Take as input q0(initial conf.), J_T(jacobian transpose), p_d(desidered
%position), p(position at actual configuration)
%for k=1:3
p_t = [0 0]';
while ((norm(p_d-p_t))/norm(p_d)) > 0.8
    q_f = double(q0+alpha*subs(J_T, q, q0)*(p_d-subs(p, q, q0)));
    p_t = subs(p, q, q0);
    q0=q_f;
end
end
