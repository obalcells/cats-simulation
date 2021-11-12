function equation = rigid_body_model(y, u, air_density, C_d, A, parachute_forces, J, J_inv, center_of_pressure, chute_attachment_point, launch_rail_time, t, chute_open)
%eq_rigid_solo Propagates the rigid body.
% u = [force; torque]
% y = [attitude; position_dq; angular_velocity; linear_velocity] in R16x1


u_act_world = [0; u(1); u(2); u(3)];
% u_act = [u(1); u(2); u(3); 0; u(4); u(5); u(6); 0];

% changed_vel = [y(12); y(9:11); y(16); y(13:15)];

q_dot = 0.5*dual_quat_mult(y(1:8), y(9:16));

gravity_B = quat_mult(quat_mult(conjugate_q(y(1:4)),[0; 0; 0; -9.81]), (y(1:4)));
thrust = u_act_world;
square_velocity = [sign(y(13))*y(13)^2; sign(y(14))*y(14)^2; sign(y(15))*y(15)^2; sign(y(16))*y(16)^2];
A_B_frame = quat_mult(quat_mult(conjugate_q(y(1:4)), [0; A]), (y(1:4)));
drag_B = -0.5*air_density*C_d*square_velocity.*A_B_frame;


force = thrust + gravity_B + drag_B;
torque = [0; cross(center_of_pressure, drag_B(2:4))] + parachute_forces;
if chute_open == 1
%     torque = [0; cross(chute_attachment_point, drag_B(2:4))];
%     torque = zeros(4,1);
end

if(launch_rail_time > t)
    torque = zeros(4,1);
end
dual_force = [force; torque];
w_dot = J_inv*(dual_force - dq_cross(y(9:16), J*y(9:16)));


equation = [q_dot; w_dot];
end

