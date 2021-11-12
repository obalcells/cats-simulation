plot_asteroid = 1;
chosen_sat = 10;
Ts = 1/20;
%% Plot 3D

% curve = animatedline;
tp = theaterPlot('XLimit',[-5 15],'YLimit',[-5 10],'ZLimit',[-5 15]);

time_string = strcat("Time : 0.00 s");
if plot_asteroid
time_handle = text(-35,45,-50,time_string,'FontSize',14, 'Color','white');
else
time_handle = text(10,-20,-10,time_string,'FontSize',14, 'Color','white');
end

%     sat_name_handle{i} = text(6,-6,10,sat_name{i},'FontSize',14);
    traject_plotter = trajectoryPlotter(tp);
grid on;
hold on;

tp.XLimits = [min(log_pos(:,1))-2, max(log_pos(:,1))+2];
tp.YLimits = [min(log_pos(:,2))-2, max(log_pos(:,2))+2];
% tp.YLimits = [-5, 5];
tp.ZLimits = [min(log_pos(:,3))-2, max(log_pos(:,3))+2];

% Load Models
rocket_model = stlread('Rocket_Pen.stl');
estimated_rocket_model = stlread('Rocket_Pen.stl');


% Scale Models %
rocket_model.vertices = 0.01*rocket_model.vertices;
estimated_rocket_model.vertices = 0.01*estimated_rocket_model.vertices;
% Save Initial positions %
rocket_model_fixed_vertices = rocket_model.vertices;
estimated_rocket_model_fixed_vertices = estimated_rocket_model.vertices;
% Render Models
rocket_patch = patch(rocket_model,'FaceColor',       [0.8 0.8 1.0], ...
    'EdgeColor',       'none',        ...
    'FaceLighting',    'gouraud',     ...
    'AmbientStrength', 0.15);

estimated_rocket_patch = patch(estimated_rocket_model,'FaceColor',       [0.6 0.6 1.0], ...
    'EdgeColor',       'none',        ...
    'FaceLighting',    'gouraud',     ...
    'AmbientStrength', 0.15);

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
view([-135 20]);

buttonH = uicontrol('Style', 'togglebutton', ...
    'Units',    'pixels', ...
    'Position', [5, 5, 60, 20], ...
    'String',   'Pause', ...
    'Value',    1);
orientation = cell(1, num_iterations);
estimated_oerientation = cell(1, num_iterations);
for t = 1:num_iterations
    orientation{t} = quaternion(log_attitude(t,1), log_attitude(t,2), log_attitude(t,3), log_attitude(t,4));
    estimated_oerientation{t} = quaternion(log_estimated.attitude(t,1), log_estimated.attitude(t,2), log_estimated.attitude(t,3), log_estimated.attitude(t,4));
end

%% Do the Actual Plotting %
for t = 1:num_iterations
    
        % Move and Rotate the true position
        rotation_matrix = rotmat(orientation{t}, 'point');
        homogenous_transform = [rotation_matrix, log_pos(t,:)'; zeros(1,3), 1];
        vertices = [rocket_model_fixed_vertices, ones(length(rocket_model_fixed_vertices(:,1)), 1)]';
        transformed_vertices = [homogenous_transform*vertices]';
        rocket_patch.Vertices = transformed_vertices(:,1:3);
        
        % Move and Rotate the estimated position
        rotation_matrix = rotmat(estimated_oerientation{t}, 'point');
        homogenous_transform = [rotation_matrix, log_estimated.pos(t,:)'; zeros(1,3), 1];
        vertices = [estimated_rocket_model_fixed_vertices, ones(length(estimated_rocket_model_fixed_vertices(:,1)), 1)]';
        transformed_vertices = [homogenous_transform*vertices]';
        estimated_rocket_patch.Vertices = transformed_vertices(:,1:3);
        
        % Plot the trajectory
%         plotTrajectory(traject_plotter{i},  {position_traj{i}(1:t, :)})
        drawnow
        
    
    % Update the Time string 
    time_string = strcat("Time : ", num2str(t*Ts), " s");
    time_handle.String = time_string;
    
        % Check if the animation is paused %
    if get(buttonH, 'Value') == 1
        while(1)
            pause(1);
            if get(buttonH, 'Value') == 0
                break;
            end
        end
    end
    
end