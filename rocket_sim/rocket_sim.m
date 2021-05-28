% Define Variables %
total_rocket_mass = 20; % kg
m_dot = -0.5;                % loss of mass [kg/s]
sim_time = 100;              % sim time [s]
diameter_rocket = 0.2;      %m
burnout_time = 2;           %s
height_rocket = 1.5;
main_chute_height = 20;
p_0 = 1013250;
Ts = 100;                    % sampling period [Hz]
air_density = 1.2754; %kg/m^3
C_d = 0.2; %drag_coefficient;
A = (diameter_rocket/2)^2*pi;
gravity = 9.81;
launch_rail_angle = 88; % degree
eul = [(launch_rail_angle-90)/180*pi 0 0];
num_iterations = sim_time * Ts;
log_acc = zeros(num_iterations, 1);
log_velocity = zeros(num_iterations, 1);
log_height = zeros(num_iterations, 1);
log_pos = zeros(num_iterations, 3);
log_attitude = zeros(num_iterations, 4);
log_mass = zeros(num_iterations, 1);
log_time = zeros(num_iterations, 1);
acc_sensor_type = 0;
baro_sensor_type = 1;
init_cond = zeros(1,16);
init_cond(1:4) = eul2quat(eul, 'XYZ');
thrust = zeros(num_iterations, 1);
thrust(1:burnout_time*Ts) = 80*ones(burnout_time*Ts, 1);
u = zeros(num_iterations, 6);
u(:, 3) = thrust;
I = diag([total_rocket_mass*(1/4*(diameter_rocket/2)^2+1/3*height_rocket^2), total_rocket_mass*(1/4*(diameter_rocket/2)^2+1/3*height_rocket^2), 1/2*total_rocket_mass*(diameter_rocket/2)^2]);
% I = diag([1/12*total_rocket_mass*(3*(diameter_rocket/2)^2+height_rocket^2), 1/12*total_rocket_mass*(3*(diameter_rocket/2)^2+height_rocket^2), 1/12*total_rocket_mass*(3*(diameter_rocket/2)^2+height_rocket^2)]);
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
    
    %     if t > burnout_time*Ts
    %         mass = mass - m_dot/Ts;
    %     end
    %     I = diag([1/12*mass*(3*(diameter_rocket/2)^2+height_rocket^2), 1/12*mass*(3*(diameter_rocket/2)^2+height_rocket^2), 1/2*mass*(diameter_rocket/2)^2]);
    %     J = [zeros(4), [1, zeros(1,3); zeros(3,1), mass*eye(3)];...
    %                     [1, zeros(1,3); zeros(3,1), I], zeros(4)];
    %     J_inv = eye(8)/J;
    
    % get velocity %
    %     [acceleration, velocity, height, mass] = rocket_eq(velocity, height, mass, c, m_dot, crosssection_rocket, C_d, H, p_0, gravity, Ts);
    if t > 10
        if((log_pos(t-2, 3) > log_pos(t-1, 3)) && (activate_parachute == 0))
            activate_parachute = 1;
            apogee_time = t;
              A = A*5;
        end
        
        if(log_pos(t-1, 3) < main_chute_height && activate_main == 0 && activate_parachute)
            activate_main = 1;
            main_time = t;
            A = A*50;
        end
    end
    
    if activate_parachute
         parachute_forces = [0, 0, 6, 0, 0, 0, 0, 0];
    end
    if activate_main
         parachute_forces = parachute_forces + [0, 0, 0, 0, 0, rand(), rand(), rand()];
    end

    
    
    drag_force = 0.5*air_density*[init_cond(13:16), zeros(1,4)].^2*C_d*A;
    [solver_time, y_out] = ode45(@(solver_time,y_out) rigid_body_model(y_out, u(t,:), drag_force', parachute_forces', J, J_inv), tspan(t):0.01:tspan(t+1), init_cond);
    init_cond = y_out(end,:);
    % Logging %
    % Position %
    pos = 2*quat_mult(init_cond(5:8), conjugate_q(init_cond(1:4)));
    %     log_acc(t) = acceleration;
    %     log_velocity(t) = velocity;
    log_pos(t,:) = pos(2:4);
    log_attitude(t, :) = init_cond(1:4);
    %     log_mass(t) = mass;
    %     log_time(t) = t/Ts;
    if pos(4) < 0
        num_iterations = t;
        break;
    end
end
%%
figure(1)
plot3(log_pos(:, 1), log_pos(:, 2), log_pos(:, 3))
grid on;
% time = 1/Ts:1/Ts:sim_time;
% subplot(2,2,1)
% plot(time, log_height)
% title("Height")
% subplot(2,2,2)
% plot(time, log_velocity)
% title("Velocity")
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
