function [dqR8] = extend_to_R8(dqR6)
%EXTEND_TO_R8 Summary of this function goes here
%   Detailed explanation goes here
output = zeros(8,1);

if((dqR6(1)^2+dqR6(2)^2+dqR6(3)^2) < 1)
    output(1) = sqrt(1-(dqR6(1)^2+dqR6(2)^2+dqR6(3)^2));
    output(2:4) = dqR6(1:3);
    output(5) = (-dqR6(1:3)'*dqR6(4:6))/(output(1));
    output(6:8) = dqR6(4:6);
else
    output(1) = 1/sqrt(1+(dqR6(1)^2+dqR6(2)^2+dqR6(3)^2));
    output(2:4) = dqR6(1:3)./sqrt(1+(dqR6(1)^2+dqR6(2)^2+dqR6(3)^2));
    output(5) = (-dqR6(1:3)'*dqR6(4:6))/(output(1));
    output(6:8) = dqR6(4:6);
end


dqR8 = output;
end

