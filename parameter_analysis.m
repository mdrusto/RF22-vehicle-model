
param = 'Weight_dist_front';
vals = 0.4:0.01:0.6;
titlestring = 'weight distribution';
x_label = 'Weight distribution front';

is_block = false;

n_vals = length(vals);
control = zeros(n_vals, 1);
stability = zeros(n_vals, 1);
balance = zeros(n_vals, 1);
grip = zeros(n_vals, 1);

model_name = 'vehicle_model';

%rtp = Simulink.BlockDiagram.buildRapidAcceleratorTarget(model_name);

set_param(model_name, 'FastRestart', 'on');
set_param(model_name, 'SimulationMode', 'accelerator');
inputs = Simulink.SimulationInput(model_name);
model_workspace = get_param(model_name, 'ModelWorkspace');
delta_block = '/Steering angle (deg)';
beta_block = '/Chassis slip angle (deg)';

% Remember the original value of the parameter
if is_block
    %original_val = 
else
    original_val = getVariable(model_workspace, param);
end

delta_block_path = strcat(model_name, delta_block);
beta_block_path = strcat(model_name, beta_block);

opt_options = optimset('fmincon');
opt_options = optimset(opt_options, 'MaxIter', 400);
opt_options = optimset(opt_options, 'TolFun', 1e-3);
opt_options = optimset(opt_options, 'TolX', 1e-3); % This line and above line: change tol from 1e-4 to 1e-3 (less accurate but faster)
opt_options = optimset(opt_options, 'Display', 'off');

for i = 1:length(vals)
    disp(['Starting simulation #' num2str(i) '/' num2str(length(vals))]);
    % Change the parameter of interest
    
    if is_block
        inputs = inputs.setBlockParameter([model_name '/' param], 'Value', num2str(vals(i)));
    else
        model_workspace.assignin(param, vals(i));
    end
    
    % Values at origin: DELTA = 0, BETA = 0
    [~, original_yaw_moment] = simulate(inputs, delta_block_path, beta_block_path, 0, 0);
    
    % For control: DELTA = 0.01, BETA = 0
    [~, control_yaw_moment] = simulate(inputs, delta_block_path, beta_block_path, 0.01, 0);
    
    % For balance: DELTA = 0, BETA = 0.01
    [~, stability_yaw_moment] = simulate(inputs, delta_block_path, beta_block_path, 0, 0.01);

    sim_opt = @(db)acc_simulate(inputs, delta_block_path, beta_block_path, db(1), db(2));
    
    % For balance and grip, optimize simulation function to find delta and beta that produce max grip
    db = fmincon(sim_opt, [0, 6], [], [], [], [], [-7, -7], [7, 7], [], opt_options);
    opt_delta = -db(1);
    opt_beta = -db(2);
    
    % Get the grip and balance that correspond to this delta and beta
    [opt_grip, opt_balance] = simulate(inputs, delta_block_path, beta_block_path, opt_delta, opt_beta);
    
    control(i) = (control_yaw_moment - original_yaw_moment) / 0.01;
    stability(i) = (stability_yaw_moment - original_yaw_moment) / 0.01;
    grip(i) = opt_grip;
    balance(i) = opt_balance;
end

figure
subplot(221)
plot(vals, control, 'red')
ylabel('Control at origin (Nm/deg)')
title(['Control with changes in ' titlestring])
xlabel(x_label)
xlim([vals(1) vals(end)])
%ylim([0 inf])

subplot(222)
plot(vals, stability, 'blue')
ylabel('Stability at origin (Nm/deg)')
title(['Stability with changes in ' titlestring])
xlabel(x_label)
xlim([vals(1) vals(end)])
%ylim([0 inf])

subplot(223)
plot(vals, balance, 'yellow')
ylabel('Balance (Nm) [+ve -> OS]')
title(['Balance with changes in ' titlestring])
xlabel(x_label)
xlim([vals(1) vals(end)])
%ylim([0 inf])

subplot(224)
plot(vals, grip / 9.81, 'green')
ylabel('Grip (g)')
title(['Grip with changes in ' titlestring])
xlabel(x_label)
xlim([vals(1) vals(end)])
%ylim([0 inf])

% Reset the model workspace variable
if is_block
    
else
    assignin(model_workspace, param, original_val);
end
set_param(model_name, 'FastRestart', 'off');
set_param(model_name, 'SimulationMode', 'normal');


function [acc, ym] = simulate(inputs, delta_block_path, beta_block_path, delta, beta)

    %paramSet = Simulink.BlockDiagram.modifyTunableParameters(rtp, 'delta_param', delta, 'beta_param', beta, param_name, val);
    
    %outputs = sim(model_name, 'SimulationMode', 'rapid', ...
    %    'RapidAcceleratorUpToDateCheck', 'off', ...
    %    'RapidAcceleratorParameterSets', paramSet);
    
    inputs = inputs.setBlockParameter(delta_block_path, 'Value', num2str(delta));
    inputs = inputs.setBlockParameter(beta_block_path, 'Value', num2str(beta));
    outputs = sim(inputs);
    
    acc = outputs.logsout{1}.Values.Data(end);
    ym = outputs.logsout{2}.Values.Data(end);
    
    if (outputs.tout(end) < 0.01)
        error(['Simulation ended prematurely at t=' num2str(outputs.tout(end))])
    end

end

function acc = acc_simulate(inputs, delta_block_path, beta_block_path, delta, beta)

[acc, ~] = simulate(inputs, delta_block_path, beta_block_path, delta, beta);

end
