function [a_1, v_1, s_1, mass] = rocket_eq(v_0, s_0, m, c, m_dot, A, C_d, H, p_0, g, Ts)
%ROCKET_EQ Summary of this function goes here
%   Detailed explanation goes here

p = p_0*exp(-g*0.0289644*(s_0)/(8.3144598*273.15));
drag = 0.5*p*v_0^2*A*C_d;
mass = m + m_dot/Ts;
a_1 = -g-c*m_dot/mass;%-drag/mass;
v_1 = v_0 + a_1/Ts;
s_1 = s_0 + v_1/Ts;
end

