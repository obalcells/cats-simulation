function [q_skew] = q_skew(q)
%Q_SKEW Summary of this function goes here
%   Detailed explanation goes here


q_skew = [0, -q(4), q(3);...
       q(4), 0, -q(2);...
       -q(3), q(2), 0];

end

