function q = rot_mat_to_q(R)
%ROT_MAT_TO_Q Summary of this function goes here
%   Detailed explanation goes here
q0 = sqrt(R(1,1)+R(2,2)+R(3,3) + 1)/2;
q1 = sqrt(R(1,1)-R(2,2)-R(3,3) + 1)/2;
q2 = sqrt(-R(1,1)+R(2,2)-R(3,3) + 1)/2;
q3 = sqrt(-R(1,1)-R(2,2)+R(3,3) + 1)/2;


q = [q0, q1, q2, q3]';
end

