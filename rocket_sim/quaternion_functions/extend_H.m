function x_dash = extend_H(H)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[height, length] = size(H);
x_dash = zeros(height, 1);

for i = 1:height/3
    for k = 1:length/3
        x_dash(3*i-2:3*i, 1) = x_dash(3*i-2:3*i, 1) + ...
                                [H(3*i, 3*k-1); H(3*i-2, 3*k); H(3*i-1, 3*k-2)];
    end
end


end

