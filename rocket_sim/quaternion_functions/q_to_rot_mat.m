function [R] = q_to_rot_mat(q)
%DQ_TO_ROT_MAT Summary of this function goes here
%   Detailed explanation goes here
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

R00 = 2 * (q0 * q0 + q1 * q1) - 1;
R01 = 2 * (q1 * q2 - q0 * q3);
R02 = 2 * (q1 * q3 + q0 * q2);

R10 = 2 * (q1 * q2 + q0 * q3);
R11 = 2 * (q0 * q0 + q2 * q2) - 1;
R12 = 2 * (q2 * q3 - q0 * q1);

R20 = 2 * (q1 * q3 - q0 * q2);
R21 = 2 * (q2 * q3 + q0 * q1);
R22 = 2 * (q0 * q0 + q3 * q3) - 1;

R = [R00, R01, R02; R10, R11, R12; R20, R21, R22];

% R = [q0^2+q1^2-q2^2-q3^2, 2*q1*q2-2*q0*q3, 2*q0*q2+2*q1*q3;...
%      2*q0*q3+2*q1*q2, q0^2-q1^2+q2^2-q3^2, 2*q2*q3-2*q0*q1;...
%      2*q1*q3-2*q0*q2, 2*q0*q1+2*q2*q3, q0^2-q1^2-q2^2+q3^2];
end

