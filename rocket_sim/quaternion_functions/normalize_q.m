function [norm_qr] = normalize_q(q)
%NORMALIZE_Q Summary of this function goes here
%   Detailed explanation goes here
norm_qr = q(1:4)./(q(1)^2+q(2)^2+q(3)^2+q(4)^2)^0.5;
end

