%% Define Variables %%

% Rocket Parameters %
total_rocket_mass = 2;                          % Rocket Mass loaded[kg]
m_dot = -0.5;                                   % loss of mass [kg/s]
diameter_rocket = 0.2;                          % Rocket Diameter [m]
burnout_time = 0.8;                             % Burnout Time of Motor [s]
average_thrust_motor = 40;                     % Average Motor Thrust [N]
height_rocket = 1.5;                            % Rocket Height [m]
drogue_chute_area = 0.16;                       % Drogue chute Area [m^2]
main_chute_area = 1.47;                         % Main chute Area [m^2]
main_chute_height = 150;                        % Height at which the Main should be triggered [m]

% Simulation Parameters %
sim_time = 1000;                                % sim time [s]
p_0 = 1013250;                                  % Zero Pressure [Pa]
Fs = 100;                                       % sampling period [Hz]
Ts = 1/Fs;
air_density = 1.2754;                           % Air density [kg/m^3]
C_d = 0.2;                                     % Drag Coefficient [-]
C_d_chute = 1.75;
A_x = diameter_rocket*height_rocket;
A_y = diameter_rocket*height_rocket;
A_z = (diameter_rocket/2)^2*pi;                   % Rocket Area for Drag [m^2]
A = [A_x; A_y; A_z];
gravity = 9.81;                                 % Gravity [m/s^2]
launch_rail_angle = 87;                         % Launch rail Angle [°]
launch_rail_time = 0.4*Fs;
center_of_pressure = [0; 0; 0.3];
chute_attachment_point = [0; 0; 1.2];

 % Noise %
 bias_vel = 1;
 std_vel = 0.001;
 bias_acc = 0.1;
 std_acc = 0.1;
 bias_height = 0;
 std_height = 2;
 bias_attitude = 0;
 std_attitude = 0.01;

% Prepare Filter %
full_state_after_prediction =  zeros(10,1);
full_state_after_prediction(1) = 1;
full_state_after_measurement =  zeros(10,1);
full_state_after_measurement(1) = 1;
reduced_state_after_prediction = zeros(9,1);
reduced_state_after_measurement = zeros(9,1);
estimated_velocity = zeros(4,1);
estimated_acc_z = 0;
Q_vel = std_vel^2*ones(1, 3);
Q_bias_vel = 1e-3*ones(1, 3);
Q_acc = std_acc^2;
Q_bias_acc = 1e-6;
Q = diag([Q_vel, Q_bias_vel, Q_acc, Q_bias_acc]);
R = diag([std_attitude^2*ones(1,3), (0.125*std_height)^2]);
G = [-0.5*eye(3), zeros(3), zeros(3,2);...
     zeros(3), eye(3), zeros(3,2);...
     zeros(1,8);...
     zeros(1,6), 1, 0;...
     zeros(1,7), 1];
 P_after_measurement = diag([1e-2*eye(1,3), 1e-6*eye(1,3), 1e-2, 1e-2, 1e-6]);

%% Prepare Variables %%
eul = [(launch_rail_angle-90)/180*pi 3/180*pi 0];
num_iterations = sim_time * Fs;
% Logging %
log_acc = zeros(num_iterations, 3);
log_lin_vel_B = zeros(num_iterations, 3);
log_lin_vel_I = zeros(num_iterations, 3);
log_ang_vel_B = zeros(num_iterations, 3);
log_pressure = zeros(num_iterations, 1);
log_pos = zeros(num_iterations, 3);
log_attitude = zeros(num_iterations, 4);
log_mass = zeros(num_iterations, 1);
log_time = zeros(num_iterations, 1);
log_estimated.pos = zeros(num_iterations, 3);
log_estimated.attitude = zeros(num_iterations, 4);
log_estimated.vel_z = zeros(num_iterations, 1);
log_estimated.bias_acc = zeros(num_iterations, 1);
log_estimated.bias_gyro = zeros(num_iterations,3);
log_noisy.pos = zeros(num_iterations, 3);
log_noisy.attitude = zeros(num_iterations, 4);
test_logger.transformed_acc = zeros(num_iterations, 4);
test_logger.raw_acc = zeros(num_iterations, 4);
% %
init_cond = zeros(1,16);
init_cond(1:4) = eul2quat(eul, 'XYZ');
thrust = zeros(num_iterations, 1);
thrust(1:burnout_time*Fs) = average_thrust_motor*ones(burnout_time*Fs, 1);
u = zeros(num_iterations, 6);
u(:, 3) = thrust;
I = diag([total_rocket_mass*(1/4*(diameter_rocket/2)^2+1/3*height_rocket^2), total_rocket_mass*(1/4*(diameter_rocket/2)^2+1/3*height_rocket^2), 1/2*total_rocket_mass*(diameter_rocket/2)^2]);
J = [zeros(4), [1, zeros(1,3); zeros(3,1), total_rocket_mass*eye(3)];...
    [1, zeros(1,3); zeros(3,1), I], zeros(4)];
J_inv = eye(8)/J;
tspan = 0:1/Fs:sim_time;

mass = total_rocket_mass;
parachute_forces = [0; 0; 0; 0];
activate_parachute = 0;
activate_main = 0;
apogee_time = 0;
for t = 1:num_iterations
    
    if t > burnout_time*Fs
        mass = mass - m_dot/Fs;
    end
    I = diag([1/12*mass*(3*(diameter_rocket/2)^2+height_rocket^2), 1/12*mass*(3*(diameter_rocket/2)^2+height_rocket^2), 1/2*mass*(diameter_rocket/2)^2]);
    J = [zeros(4), [1, zeros(1,3); zeros(3,1), mass*eye(3)];...
        [1, zeros(1,3); zeros(3,1), I], zeros(4)];
    J_inv = eye(8)/J;
    
    % get velocity %
    if t > 10
        if((log_pos(t-2, 3) > log_pos(t-1, 3)) && (activate_parachute == 0))
            activate_parachute = 1;
            apogee_time = t;
            A(1:2) = 0;
            A(3) = drogue_chute_area;
            C_d = C_d_chute;
        end
        
        if(log_pos(t-1, 3) < main_chute_height && activate_main == 0 && activate_parachute)
            activate_main = 1;
            main_time = t;
            A(1:2) = 0;
%             A(3) = main_chute_area;
        end
    end
    
    if activate_parachute
%          parachute_forces = [0; 0; 0; 6];
    end
    if activate_main
        parachute_forces = parachute_forces + [0; 0.0001*rand(); 0.0001*rand(); 0.0001*rand()];
    end
        
    [solver_time, y_out] = ode45(@(solver_time,y_out) rigid_body_model(y_out, u(t,:), air_density, C_d, A, parachute_forces, J, J_inv, center_of_pressure, chute_attachment_point, launch_rail_time, t, activate_parachute), tspan(t):0.01:tspan(t+1), init_cond);
    init_cond = y_out(end,:);
    
    % Compute Forces %
    gravity_B = quat_mult(quat_mult(conjugate_q(init_cond(1:4)'),[0; 0; 0; -9.81]), (init_cond(1:4)'));
    thrust = [0; u(t, 1); u(t, 2); u(t, 3)];
    square_velocity = [sign(init_cond(13))*init_cond(13)^2; sign(init_cond(14))*init_cond(14)^2; sign(init_cond(15))*init_cond(15)^2; sign(init_cond(16))*init_cond(16)^2];
    A_B_frame = quat_mult(quat_mult(conjugate_q(init_cond(1:4)'), [0; A]), (init_cond(1:4)'));
    drag_B = -0.5*air_density*C_d*square_velocity.*A_B_frame;


    force2 = thrust + gravity_B + drag_B;
    torque = [0; cross(center_of_pressure, drag_B(2:4))] + parachute_forces;
    force = [force2; torque];
    pos = 2*quat_mult(init_cond(5:8), conjugate_q(init_cond(1:4)));

%     gravity_B = dual_quat_mult(dual_quat_mult((init_cond(1:8)),[0; 0; 0; -9.81; 0; 0; 0; 0]), conjugate_dq(init_cond(1:8)));
%     u_act = [0; u(t, 1); u(t, 2); u(t, 3); 0; u(t, 4); u(t, 5); u(t, 6)];
%     thrust_B = u_act;
%     square_velocity = [sign(init_cond(13))*init_cond(13)^2; sign(init_cond(14))*init_cond(14)^2; sign(init_cond(15))*init_cond(15)^2; sign(init_cond(16))*init_cond(16)^2];
%     drag_B = -0.5*air_density*[square_velocity; zeros(4,1)]*C_d.*[0; A; zeros(4,1)];
%     parachute_B = quat_mult(quat_mult((init_cond(1:4)),parachute_forces), conjugate_q(init_cond(1:4)));
%     force = gravity_B + thrust_B + drag_B + [parachute_B; zeros(4,1)];
%     pos = 2*quat_mult(init_cond(5:8), conjugate_q(init_cond(1:4)));
    
    
    % Do Kalman %
    % Predict %
    
    measured_ang_velocity = init_cond(10:12)' + normrnd(bias_vel, std_vel, 3, 1);
    measured_acceleration = [0; 1/total_rocket_mass*force(2:4)] + [0; normrnd(bias_acc, std_acc, 3, 1)];
    measured_height = pos(4) + normrnd(bias_height, std_height);
    measured_attitude = init_cond(1:4)' + normrnd(bias_attitude, std_attitude, 4, 1);
    measured_attitude = normalize_q(measured_attitude);
    log_noisy.pos(t, 3) = measured_height;
    log_noisy.attitude(t, :) = measured_attitude;
    
    estimated_velocity = [0; measured_ang_velocity - full_state_after_measurement(5:7)];
    holder = quat_mult(quat_mult((full_state_after_measurement(1:4)), measured_acceleration), conjugate_q(full_state_after_measurement(1:4)));
%     holder = measured_acceleration;
    test_logger.transformed_acc(t, :) = holder;
    test_logger.raw_acc(t, :) = measured_acceleration;
    estimated_acc_z = holder(4);
    
    full_state_after_prediction(1:4) = normalize_q(full_state_after_measurement(1:4) + Ts/2* quat_mult(full_state_after_measurement(1:4), estimated_velocity));
    full_state_after_prediction(5:7) = full_state_after_measurement(5:7);
    full_state_after_prediction(8) = full_state_after_measurement(8) + Ts*full_state_after_measurement(9);
    full_state_after_prediction(9) = full_state_after_measurement(9) + Ts*estimated_acc_z - full_state_after_measurement(10);
    full_state_after_prediction(10) = full_state_after_measurement(10);
    
    % Compute F and G %
    F = zeros(9);
    F(1:3, 1:3) = -q_skew(estimated_velocity);
    F(1:3, 4:6) = -1/2*eye(3);
    F(7, 8) = 1;
    F(8, 9) = -1;
    acc2 = measured_acceleration(2);
    acc3 = measured_acceleration(3);
    acc4 = measured_acceleration(4);
    q_est1 = full_state_after_prediction(1);
    q_est2 = full_state_after_prediction(2);
    q_est3 = full_state_after_prediction(3);
    q_est4 = full_state_after_prediction(4);
    F(8, 1) = 2*acc3*q_est1^2 - 4*acc4*q_est1*q_est2 - 2*acc3*q_est2^2 - 2*acc3*q_est3^2 - 4*acc4*q_est3*q_est4 + 2*acc3*q_est4^2;
    F(8, 2) = - 2*acc2*q_est1^2 - 4*acc4*q_est1*q_est3 + 2*acc2*q_est2^2 + 4*acc4*q_est2*q_est4 + 2*acc2*q_est3^2 - 2*acc2*q_est4^2;
    F(8, 3) = 4*acc2*q_est1*q_est2 + 4*acc3*q_est1*q_est3 + 4*acc2*q_est3*q_est4 - 4*acc3*q_est2*q_est4;
    P_after_prediction = P_after_measurement + Ts*(F*P_after_measurement+P_after_measurement*F' + G*Q*G');
    
    % Measurement Update %
    H = [eye(3), zeros(3, 6);...
        zeros(1, 6), 1, 0, 0];
    error_quat =  quat_mult(conjugate_q(full_state_after_prediction(1:4)), measured_attitude);
    meas_vector = [error_quat(2:4); measured_height - full_state_after_prediction(8)];
    
    K = P_after_prediction*H'*(eye(4)/(H*P_after_prediction*H'+R));
    
    kalman_update = K*meas_vector;
    
    orientation_update = quat_mult(full_state_after_prediction(1:4), extend_to_R4(kalman_update(1:3)));
    full_state_after_measurement(1:4) = orientation_update;
    full_state_after_measurement(5:end) = kalman_update(4:end) + full_state_after_prediction(5:end);
    
    P_after_measurement = (eye(9) - K*H)*P_after_prediction;
    
    
    
    
    % Logging %
    log_estimated.bias_gyro(t,:) = full_state_after_measurement(5:7);
    log_estimated.pos(t, 3) = full_state_after_measurement(8);
    log_estimated.attitude(t, :) = full_state_after_measurement(1:4);
    log_estimated.vel_z(t) = full_state_after_measurement(9);
    log_estimated.bias_acc(t) = full_state_after_measurement(10);
    log_pos(t,:) = pos(2:4);                        % [m]
    log_attitude(t, :) = init_cond(1:4);            % [quat]
    log_lin_vel_B(t, :) = init_cond(14:16);           % [m/s]
    log_ang_vel_B(t, :) = init_cond(10:12)/pi*180;    % [°/s]
    holder = quat_mult(quat_mult((init_cond(1:4)'), [0, init_cond(14:16)]'), conjugate_q(init_cond(1:4)'));
    log_lin_vel_I(t, :) = holder(2:4);
    log_acc(t, :) = 1/total_rocket_mass*force(2:4);  % [m/s^2]
    log_mass(t) = mass;                             % [kg]
    %     log_time(t) = t/Ts;
    if pos(4) < 0
        num_iterations = t;
        break;
    end
end

%% Save in datastructure %
% Pressure Data %
pressure_sensor = p_0*exp(-gravity*0.0289644*(log_pos(1:num_iterations, 3))/(8.3144598*273.15));
press_data = [pressure_sensor, pressure_sensor, pressure_sensor];

% Acceleration Data %
acc_data = [log_acc(1:num_iterations, :), log_acc(1:num_iterations, :), log_acc(1:num_iterations, :)];

% Gyro Data %
gyro_data = [log_ang_vel_B(1:num_iterations, :), log_ang_vel_B(1:num_iterations, :), log_ang_vel_B(1:num_iterations, :)];

% Add Noise %
% parameters %
std_pressure = 100;                     %[Pa]
bias_pressure = 10*rand(1,3);           %[Pa]
std_gyro = 5;                           %[°/s]
bias_gyro = 0.1*rand(3,3);              %[°/s]
std_acc = 0.01;                         %[m/s^2]
bias_acc = 0.1*rand(3,3);               %[m/s^2]

% Adding it %
pressure_noise = [normrnd(bias_pressure(1), std_pressure, num_iterations, 1), normrnd(bias_pressure(1), std_pressure, num_iterations, 1), normrnd(bias_pressure(1), std_pressure, num_iterations, 1)];
gyro_noise = [normrnd(bias_gyro(1,1), std_gyro, num_iterations, 1), normrnd(bias_gyro(1,2), std_gyro, num_iterations, 1), ...
    normrnd(bias_gyro(1,3), std_gyro, num_iterations, 1), normrnd(bias_gyro(2,1), std_gyro, num_iterations, 1), ...
    normrnd(bias_gyro(2,2), std_gyro, num_iterations, 1), normrnd(bias_gyro(2,3), std_gyro, num_iterations, 1), ...
    normrnd(bias_gyro(3,1), std_gyro, num_iterations, 1), normrnd(bias_gyro(3,2), std_gyro, num_iterations, 1), ...
    normrnd(bias_gyro(3,3), std_gyro, num_iterations, 1)];
acc_noise = [normrnd(bias_acc(1,1), std_acc, num_iterations, 1), normrnd(bias_acc(1,2), std_acc, num_iterations, 1), ...
    normrnd(bias_acc(1,3), std_acc, num_iterations, 1), normrnd(bias_acc(2,1), std_acc, num_iterations, 1), ...
    normrnd(bias_acc(2,2), std_acc, num_iterations, 1), normrnd(bias_acc(2,3), std_acc, num_iterations, 1), ...
    normrnd(bias_acc(3,1), std_acc, num_iterations, 1), normrnd(bias_acc(3,2), std_acc, num_iterations, 1), ...
    normrnd(bias_acc(3,3), std_acc, num_iterations, 1)];
pressure_noisy = press_data + pressure_noise;
gyro_noisy = gyro_data + gyro_noise;
acc_noisy = acc_data + acc_noise;

%%
figure(1)
%plot3(log_pos(:, 1), log_pos(:, 2), log_pos(:, 3))

grid on;
time = 1/Fs:1/Fs:sim_time;
subplot(3,2,1)
plot(time, log_pos(:, 3))
hold on;
plot(time, log_estimated.pos(:, 3))
plot(time, log_noisy.pos(:, 3));
xlim([0, num_iterations*Ts]);
xlabel('s')
ylabel('m')
hold off;
title("Height")
subplot(3,2,3)
plot(time, log_lin_vel_I(:, 3))
hold on;
plot(time, log_estimated.vel_z(:))
xlim([0, num_iterations*Ts]);
hold off;
xlabel('s')
ylabel('m/s')
title("Velocity")
subplot(3,2,5)
plot(time, log_estimated.bias_acc(:))
xlim([0, num_iterations*Ts]);
xlabel('s')
ylabel('m/s^2')
title("Acc Offset")
% Quaternion %
subplot(3,2,2)
plot(time, log_attitude(:, 2))
hold on;
plot(time, log_estimated.attitude(:, 2))
plot(time, log_noisy.attitude(:, 2));
xlim([0, num_iterations*Ts]);
xlabel('s')
hold off;
title("Q_1")
subplot(3,2,4)
plot(time, log_attitude(:, 3))
hold on;
plot(time, log_estimated.attitude(:, 3))
plot(time, log_noisy.attitude(:, 3));
xlim([0, num_iterations*Ts]);
xlabel('s')
hold off;
title("Q_2")
subplot(3,2,6)
plot(time, log_attitude(:, 4))
hold on;
plot(time, log_estimated.attitude(:, 4))
plot(time, log_noisy.attitude(:, 4));
xlim([0, num_iterations*Ts]);
xlabel('s')
hold off;
title("Q_3")
legend('true', 'estimated', 'noisy measurement')
figure(2)
subplot(4,2,1)
hold on;
plot(time, test_logger.transformed_acc(:, 1));
plot(time, test_logger.raw_acc(:, 1));
xlim([0, num_iterations*Ts]);
hold off;
subplot(4,2,3)
hold on;
plot(time, test_logger.transformed_acc(:, 2));
plot(time, test_logger.raw_acc(:, 2));
xlim([0, num_iterations*Ts]);
hold off;
subplot(4,2,5)
hold on;
plot(time, test_logger.transformed_acc(:, 3));
plot(time, test_logger.raw_acc(:, 3));
xlim([0, num_iterations*Ts]);
hold off;
subplot(4,2,7)
hold on;
plot(time, test_logger.transformed_acc(:, 4));
plot(time, test_logger.raw_acc(:, 4));
xlim([0, num_iterations*Ts]);
hold off;
legend('transformed', 'raw');
subplot(4,2,2)
plot(time, log_estimated.bias_gyro(:,1));
title('bias gyro_x');
subplot(4,2,4)
plot(time, log_estimated.bias_gyro(:,2));
xlim([0, num_iterations*Ts]);
title('bias gyro_y');
subplot(4,2,6)
plot(time, log_estimated.bias_gyro(:,3));
xlim([0, num_iterations*Ts]);
title('bias gyro_z');

% subplot(2,2,3)
% plot(time, log_acc)
% title("Acceleration")
% subplot(2,2,4)
% plot(time, log_mass)
% title("Mass")
% log_baro = p_0*exp(-gravity*0.0289644*(log_height)/(8.3144598*273.15));
% acc_data = [log_acc + rand(num_iterations, 1), log_acc + rand(num_iterations, 1), log_acc + rand(num_iterations, 1)]*1024/gravity;
% baro_data = [log_baro + 100*rand(num_iterations, 1), log_baro + 100*rand(num_iterations, 1), log_baro + 100*rand(num_iterations, 1)];
% data_struct = [log_time*1000, round(acc_data), log_time*1000+5, round(baro_data)];

return;
%% Simulink to stuzff
log_estimated.pos = out.position.Data;
log_pos = out.position.Data;
for i = 1:length(out.euler_angles.Data)
    log_attitude(i, :) = eul2quat(out.euler_angles.Data(i, :));
    log_estimated.attitude(i, :) = eul2quat(out.euler_angles.Data(i, :));
end
