%% Define Variables %%

% Rocket Parameters %
total_rocket_mass = 2;                          % Rocket Mass loaded[kg]
m_dot = -0.5;                                   % loss of mass [kg/s]
diameter_rocket = 0.2;                          % Rocket Diameter [m]
burnout_time = 0.8;                             % Burnout Time of Motor [s]
average_thrust_motor = 300;                     % Average Motor Thrust [N]
height_rocket = 1.5;                            % Rocket Height [m]
drogue_chute_area = 0.16;                       % Drogue chute Area [m^2]
main_chute_area = 1.47;                         % Main chute Area [m^2]
main_chute_height = 150;                        % Height at which the Main should be triggered [m]

% Simulation Parameters %
sim_time = 1000;                                % sim time [s]
p_0 = 1013250;                                  % Zero Pressure [Pa]
Ts = 50;                                       % sampling period [Hz]
air_density = 1.2754;                           % Air density [kg/m^3]
C_d = 0.75;                                     % Drag Coefficient [-]
C_d_chute = 1.75;
A = (diameter_rocket/2)^2*pi;                   % Rocket Area for Drag [m^2]
gravity = 9.81;                                 % Gravity [m/s^2]
launch_rail_angle = 85;                         % Launch rail Angle [°]

%% Prepare Variables %%
eul = [(launch_rail_angle-90)/180*pi 0 0];
num_iterations = sim_time * Ts;
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
% %
init_cond = zeros(1,16);
init_cond(1:4) = eul2quat(eul, 'XYZ');
thrust = zeros(num_iterations, 1);
thrust(1:burnout_time*Ts) = average_thrust_motor*ones(burnout_time*Ts, 1);
u = zeros(num_iterations, 6);
u(:, 3) = thrust;
I = diag([total_rocket_mass*(1/4*(diameter_rocket/2)^2+1/3*height_rocket^2), total_rocket_mass*(1/4*(diameter_rocket/2)^2+1/3*height_rocket^2), 1/2*total_rocket_mass*(diameter_rocket/2)^2]);
J = [zeros(4), [1, zeros(1,3); zeros(3,1), total_rocket_mass*eye(3)];...
    [1, zeros(1,3); zeros(3,1), I], zeros(4)];
J_inv = eye(8)/J;
tspan = 0:1/Ts:sim_time;

mass = total_rocket_mass;
parachute_forces = [0, 0, 0, 0, 0, 0, 0, 0];
activate_parachute = 0;
activate_main = 0;
apogee_time = 0;
for t = 1:num_iterations
    
    if t > burnout_time*Ts
        mass = mass - m_dot/Ts;
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
            A = drogue_chute_area;
            C_d = C_d_chute;
        end
        
        if(log_pos(t-1, 3) < main_chute_height && activate_main == 0 && activate_parachute)
            activate_main = 1;
            main_time = t;
            A = main_chute_area;
        end
    end
    
    if activate_parachute
        parachute_forces = [0, 0, 6, 0, 0, 0, 0, 0];
    end
    if activate_main
        parachute_forces = parachute_forces + [0, 0, 0, 0, 0, rand(), rand(), rand()];
    end
        
    [solver_time, y_out] = ode45(@(solver_time,y_out) rigid_body_model(y_out, u(t,:), air_density, C_d, A, parachute_forces', J, J_inv), tspan(t):0.01:tspan(t+1), init_cond);
    init_cond = y_out(end,:);
    
    % Compute Forces %
    gravity_B = dual_quat_mult(dual_quat_mult((init_cond(1:8)),[0; 0; 0; -9.81; 0; 0; 0; 0]), conjugate_dq(init_cond(1:8)));
    u_act = [0; u(t, 1); u(t, 2); u(t, 3); 0; u(t, 4); u(t, 5); u(t, 6)];
    thrust_B = u_act;
    square_velocity = [sign(init_cond(13))*init_cond(13)^2; sign(init_cond(14))*init_cond(14)^2; sign(init_cond(15))*init_cond(15)^2; sign(init_cond(16))*init_cond(16)^2];
    drag_B = -0.5*air_density*[square_velocity; zeros(4,1)]*C_d*A;
    parachute_B = dual_quat_mult(dual_quat_mult((init_cond(1:8)),parachute_forces'), conjugate_dq(init_cond(1:8)));
    force = gravity_B + thrust_B + drag_B' + parachute_B;
    
    % Logging %
    pos = 2*quat_mult(init_cond(5:8), conjugate_q(init_cond(1:4)));
    
    log_pos(t,:) = pos(2:4);                        % [m]
    log_attitude(t, :) = init_cond(1:4);            % [quat]
    log_lin_vel_B(t, :) = init_cond(14:16);           % [m/s]
    log_ang_vel_B(t, :) = init_cond(10:12)/pi*180;    % [°/s]
    holder = quat_mult(quat_mult(init_cond(1:4)', [0, init_cond(14:16)]'), conjugate_q(init_cond(1:4)'));
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
time = 1/Ts:1/Ts:sim_time;
subplot(2,1,1)
plot(time, log_pos(:, 3))
% title("Height")
subplot(2,1,2)
plot(time, log_lin_vel_I(:, 3))
title("Velocity")
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
