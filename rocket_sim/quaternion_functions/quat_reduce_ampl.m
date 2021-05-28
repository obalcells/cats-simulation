function q_out = quat_reduce_ampl(q,weight)
%QUAT_REDUCE_AMPL Summary of this function goes here
%   Detailed explanation goes here
out = [q(1); weight*q(2:4)];

out(1) = sqrt(1 - (out(2)^2 + out(3)^2 + out(4)^2));

q_out = out;

end

