function output = q_cross(q, p)
%Q_CROSS_PRODUCT Summary of this function goes here
%   Detailed explanation goes here

output = 0.5*(quat_mult(q,p)-quat_mult(conjugate_q(p), conjugate_q(q)));
end

