model_name = 'vehicle_model';

% Set to false to connect points with straight lines
USE_SPLINES = true;

limit = 5;
change = 0.5;

% Create delta and beta arrays
% delta_vals = 1:0.1:5;
% beta_vals = -8:0.1:-3;
%delta_vals = -25:1:25;
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
        
        if delta == 0 && beta == 0
            stability = yaw_moment - old_yaw_moment;
        end
        
        scatter(accel, yaw_moment, 5, 'k')
    end
    
    for beta_index = 1:n_beta
        if delta_vals(delta_index) == 0 && beta_vals(beta_index) == 0
            control = new_yaw_moment - old_yaw_moment;
        end
    end

end

if USE_SPLINES
    % Draw the constant-beta splines (red)
    for delta_index = 1:n_delta
        fnplt(cscvn([accel_vals(delta_index, :); yaw_moment_vals(delta_index, :)]), 'red');
    end
    % Draw the constant-delta splines (blue)
    for beta_index = 1:n_beta
        fnplt(cscvn([accel_vals(:, beta_index)'; yaw_moment_vals(:, beta_index)']), 'blue');
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
% for i = 1:n_delta
%    txt = ['Delta = ', num2str(delta_vals(i))];
%    text(accel_vals(i, 1), yaw_moment_vals(i, 1), txt);
% end
% for i = 1:n_beta
%    txt = ['Beta = ', num2str(beta_vals(i))];
%    text(accel_vals(1, i), yaw_moment_vals(1, i), txt);
% end

for i = 1:n_delta
    for j = 1:n_beta
        txt = "Delta = " + delta_vals(i) + ", Beta = " + beta_vals(j);
        %text(accel_vals(i, j), yaw_moment_vals(i, j), txt);
    end
end

% Output the control and stability values
if exist('control', 'var')
    disp(['Control: ', num2str(control), ' Nm/deg'])
    disp(['Stability: ', num2str(stability), ' Nm/deg'])
else
    disp('Either beta and/or delta values did not include 0, no control or stability calculated')
end