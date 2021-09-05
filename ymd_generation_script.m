model_name = 'vehicle_model';

% Set to false to connect points with straight lines
USE_SPLINES = false;

limit = 4;
change = 0.5;
 
% Create delta and beta arrays
% delta_vals = 1:0.1:5;
% beta_vals = -8:0.1:-3;
delta_vals = -limit:change:limit;
beta_vals = -limit:change:limit;

% Number of values in each
n_delta = length(delta_vals);
n_beta = length(beta_vals);

inputs = Simulink.SimulationInput(model_name);

set_param(model_name, 'FastRestart', 'on');
set_param(model_name, 'SimulationMode', 'accelerator');

figure
hold on
xlabel('Lateral acceleration (g)')
ylabel('Total yaw moment (N*m)')
title('Yaw moment diagram')

% Set up output arrays (empty for now)
accel_vals = zeros(n_delta, n_beta);
yaw_moment_vals = zeros(n_delta, n_beta);

for delta_index = 1:n_delta
    delta = delta_vals(delta_index);
    for beta_index = 1:n_beta
        beta = beta_vals(beta_index);
        disp(strcat('Simulation delta=', num2str(delta), ', beta=', num2str(beta)))
        
        % Change the delta and beta inputs in the model
        inputs = inputs.setBlockParameter(strcat(model_name, '/Steering angle (deg)'), 'Value', num2str(delta));
        inputs = inputs.setBlockParameter(strcat(model_name, '/Chassis slip angle (deg)'), 'Value', num2str(beta));
        
        % Run the simulation
        outputs = sim(inputs);
        
        % Record the outputs
        accel = outputs.logsout{1}.Values.Data(end)/9.81;
        yaw_moment = outputs.logsout{2}.Values.Data(end);
        accel_vals(delta_index, beta_index) = accel;
        yaw_moment_vals(delta_index, beta_index) = yaw_moment;
        
%         if abs(accel - 1.53201) < 0.00001 && abs(yaw_moment - -1476.49) < 0.1
%             disp(['Beta: ' num2str(beta) ', delta: ' num2str(delta)])
%         end
        
        % Plot a constant-delta line
        if beta_index > 1
            %old_accel = accel_vals(delta_index, beta_index - 1);
            old_yaw_moment = yaw_moment_vals(delta_index, beta_index - 1);
            %plot([old_accel accel], [old_yaw_moment yaw_moment], 'red')
            %
            if delta == 0 && beta == 0
                stability = yaw_moment - old_yaw_moment;
            end
        end
        scatter(accel, yaw_moment, 5, 'k')
    end
    
    if delta_index > 1
        for beta_index = 1:n_beta
            % Plot the constant-beta lines
            %old_accel = accel_vals(delta_index - 1, beta_index);
            old_yaw_moment = yaw_moment_vals(delta_index - 1, beta_index);
            new_accel = accel_vals(delta_index, beta_index);
            new_yaw_moment = yaw_moment_vals(delta_index, beta_index);
            %plot([old_accel new_accel], [old_yaw_moment new_yaw_moment], 'blue')
            
            if delta_vals(delta_index) == 0 && beta_vals(beta_index) == 0
                control = new_yaw_moment - old_yaw_moment;
            end
        end
    end
end

if USE_SPLINES
    for delta_index = 1:n_delta
        accel_xx = linspace(accel_vals(delta_index, 1), accel_vals(delta_index, end), 1000);
        yaw_moment_yy = spline(accel_vals(delta_index, :), yaw_moment_vals(delta_index, :), accel_xx);
        plot(accel_xx, yaw_moment_yy, 'red');
    end
    for beta_index = 1:n_beta
        accel_xx = linspace(accel_vals(1, beta_index), accel_vals(end, beta_index), 1000);
        yaw_moment_yy = spline(accel_vals(:, beta_index), yaw_moment_vals(:, beta_index), accel_xx);
        plot(accel_xx, yaw_moment_yy, 'blue');
    end
else
    for delta_index = 1:n_delta
        plot(accel_vals(delta_index, :), yaw_moment_vals(delta_index, :), 'red');
    end
    for beta_index = 1:n_beta
        plot(accel_vals(:, beta_index), yaw_moment_vals(:, beta_index), 'blue');
    end
end

set_param(model_name, 'FastRestart', 'off');

% Plot x and y axes
line([0 0], ylim)
line(xlim, [0 0])

% Add labels
for i = 1:n_delta
    txt = ['Delta = ', num2str(delta_vals(i))];
    text(accel_vals(i, 1), yaw_moment_vals(i, 1), txt);
end
for i = 1:n_beta
    txt = ['Beta = ', num2str(beta_vals(i))];
    text(accel_vals(1, i), yaw_moment_vals(1, i), txt);
end

% Output the control and stability values
if exist('control', 'var')
    disp(['Control: ', num2str(control), ' Nm/deg'])
    disp(['Stability: ', num2str(stability), ' Nm/deg'])
else
    disp('Either beta and/or delta values did not include 0, no control or stability calculated')
end