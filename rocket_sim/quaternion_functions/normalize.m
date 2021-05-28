function [dq_norm] = normalize(dq)
%NORMALIZE Summary of this function goes here
%   Detailed explanation goes here


if(dq(1)^2+dq(2)^2+dq(3)^2+dq(4)^2 == 0)
    dq_norm = dq;
else
%     norm_qr = dq(1:4)./(dq(1)^2+dq(2)^2+dq(3)^2+dq(4)^2)^0.5;
%     
%     
%     norm_qd = (eye(4) - norm_qr(1:4)*norm_qr(1:4)'/(norm_qr(1)^2+norm_qr(2)^2+norm_qr(3)^2+norm_qr(4)^2))*dq(5:8);
%     
%     dq_norm = [norm_qr; norm_qd];
    dummy = dq(1:4);
    norm_qr = dq(1:4)./(dq(1)^2+dq(2)^2+dq(3)^2+dq(4)^2)^0.5;
    
    
    norm_qd = (eye(4) - norm_qr*norm_qr'/(norm_qr(1)^2+norm_qr(2)^2+norm_qr(3)^2+norm_qr(4)^2))*dq(5:8);
    
    dq_norm = [norm_qr; norm_qd];
end
end

