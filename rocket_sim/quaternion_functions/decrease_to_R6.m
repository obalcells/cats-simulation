function [dqR6] = decrease_to_R6(dqR8)
%DECREASE_TO_R6 Summary of this function goes here
%   Detailed explanation goes here

dqR6 = [dqR8(2); dqR8(3); dqR8(4); dqR8(6); dqR8(7); dqR8(8)];
end

