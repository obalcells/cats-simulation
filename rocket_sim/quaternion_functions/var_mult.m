function [M] = var_mult(q)
%VAR_MULT Summary of this function goes here
%   Detailed explanation goes here
M = [ -q(1), q(4), -q(3);...
      -q(4), -q(1), q(2);...
      q(3), -q(2), -q(1)];
end

