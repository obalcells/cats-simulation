function equation = rigid_body_model(y, u, air_density, C_d, A, parachute_forces, J, J_inv)
%eq_rigid_solo Propagates the rigid body.
% u = [force; torque]
% y = [attitude; position_dq; angular_velocity; linear_velocity] in R16x1


u_act = [0; u(1); u(2); u(3); 0; u(4); u(5); u(6)];
% u_act = [u(1); u(2); u(3); 0; u(4); u(5); u(6); 0];

% changed_vel = [y(12); y(9:11); y(16); y(13:15)];

q_dot = 0.5*dual_quat_mult(y(1:8), y(9:16));

gravity_B = dual_quat_mult(dual_quat_mult((y(1:8)),[0; 0; 0; -9.81; 0; 0; 0; 0]), conjugate_dq(y(1:8)));
thrust = u_act;
square_velocity = [sign(y(13))*y(13)^2; sign(y(14))*y(14)^2; sign(y(15))*y(15)^2; sign(y(16))*y(16)^2];
drag_B = -0.5*air_density*[square_velocity; zeros(4,1)]*C_d*A;
% drag_B = -0.5*sign(y(13:16))*air_density*[y(13:16); zeros(4,1)].^2*C_d*A;
% drag_B = dual_quat_mult(dual_quat_mult((y(1:8)),drag_B), conjugate_dq(y(1:8)));

% thrust = dual_quat_mult(dual_quat_mult((y(1:8)),u_act), conjugate_dq(y(1:8)));
% gravity_B = [0; 0; 0; -9.81; 0; 0; 0; 0];
parachute_B = dual_quat_mult(dual_quat_mult((y(1:8)),parachute_forces), conjugate_dq(y(1:8)));
force = thrust + gravity_B + drag_B + parachute_B;

w_dot = J_inv*(force - dq_cross(y(9:16), J*y(9:16)));


equation = [q_dot; w_dot];
end

