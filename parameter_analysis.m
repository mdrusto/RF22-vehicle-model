function parameter_analysis(param, vals, titlestring, x_label)

n_vals = length(vals);
control = zeros(n_vals, 1);
stability = zeros(n_vals, 1);

model_name = 'vehicle_model';
inputs = Simulink.SimulationInput(model_name);
model_workspace = get_param(model_name, 'ModelWorkspace');

% Remember the original value of the parameter
original_val = getVariable(model_workspace, param);

for i = 1:length(vals)
    disp(['Starting simulation #' num2str(i) '/' num2str(length(vals))]);
    % Change the parameter of interest
    %inputs = inputs.setBlockParameter([model_name '/' param], 'Value', num2str(vals(i));
    model_workspace.assignin(param, vals(i));
    
    % DELTA = 0, BETA = 0
    % Change the delta and beta inputs in the model
    inputs = inputs.setBlockParameter(strcat(model_name, '/Steering angle, deg'), 'Value', num2str(0));
    inputs = inputs.setBlockParameter(strcat(model_name, '/Chassis slip angle, deg'), 'Value', num2str(0));
    outputs = sim(inputs); % Run the simulation
    original_yaw_moment = outputs.logsout{2}.Values.Data(1); % Record the outputs
    %disp(['Original yaw moment: ' num2str(original_yaw_moment) 'Nm'])
    
    % DELTA = 0.01, BETA = 0
    % Change the delta input in the model
    inputs = inputs.setBlockParameter(strcat(model_name, '/Steering angle, deg'), 'Value', num2str(0.01));
    outputs = sim(inputs); % Run the simulation
    control_yaw_moment = outputs.logsout{2}.Values.Data(1); % Record the outputs
    %disp(['Control yaw moment: ' num2str(control_yaw_moment) 'Nm'])
    
    % DELTA = 0, BETA = 0.01
    % Reset the delta input and change the beta input in the model
    inputs = inputs.setBlockParameter(strcat(model_name, '/Steering angle, deg'), 'Value', num2str(0));
    inputs = inputs.setBlockParameter(strcat(model_name, '/Chassis slip angle, deg'), 'Value', num2str(0.01));
    outputs = sim(inputs); % Run the simulation
    stability_yaw_moment = outputs.logsout{2}.Values.Data(1); % Record the outputs
    %disp(['Stability yaw moment: ' num2str(stability_yaw_moment) 'Nm'])

    control(i) = (control_yaw_moment - original_yaw_moment) / 0.01;
    stability(i) = (stability_yaw_moment - original_yaw_moment) / 0.01;
end

figure
yyaxis left
plot(vals, control)
ylabel('Control (Nm/deg)')
ylim([0 inf])

yyaxis right
plot(vals, stability)
ylabel('Stability (Nm/deg)')
title(['Control and stability with changes in ' titlestring])
xlabel(x_label)
ylim([0 inf])

% Reset the model workspace variable
assignin(model_workspace, param, original_val);

end
