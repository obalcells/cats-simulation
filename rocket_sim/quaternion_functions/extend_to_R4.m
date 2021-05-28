function [q] = extend_to_R4(q_bar)
%EXTEND_TO_R4 Summary of this function goes here
%   Detailed explanation goes here


q = [sqrt(1-(q_bar(1)^2+q_bar(2)^2+q_bar(3)^2)); q_bar];
end

