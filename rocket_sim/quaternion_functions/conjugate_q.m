function [q_star] = conjugate_q(q)
%CONJUGATE_Q Conjugate quaternion
%   q_star = q*
q_star = [q(1);-q(2);-q(3);-q(4)];
end

