function [q_dot_sns] = sns (q_dot, q_dot_max, J)
%compute sns
q_dot_max_ = q_dot_max;
q_dot_sns_ = q_dot;
i=1;
for i = 1:length(q_dot_sns_)
    if q_dot(i) > q_dot_max_(i)
        J(:,i) = [];
        q_dot(i) = [];
        x_dot_sns = J*q_dot;
        q_dot_sns_ = pinv(J)*x_dot_sns;
        q_dot_sns = [q_dot_sns(1:i-1); q_dot_max(i); q_dot_sns(i:length(q_dot))];
        q_dot_max(i) = [];
        q_dot_sns_ = sns(q_dot_sns_, q_dot_max_, J);
    else
        q_dot_sns(i) = q_dot(i);
    end
end
end
            
            
        
    