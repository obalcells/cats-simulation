function [dq_star] = conjugate_dq(dq)
%CONJUGATE_DQ Conjugate dual quaternion
%   dq_star = dq*
dq_star = [dq(1);-dq(2);-dq(3);-dq(4);dq(5);-dq(6);-dq(7);-dq(8)];
end

