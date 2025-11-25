

clear; clc; close all;

% 1. Load Data
% Ensure training_data.csv is in your path
if isfile("training_data.csv")
    T = readmatrix("training_data.csv");
else
    error("training_data.csv not found.");
end

% --- CONFIGURATION ---
joints = struct();
% Define the map: Name, Velocity Column, Command Column
joints(1).name = "Boom";   joints(1).col_wd = 4; joints(1).col_u = 7;
joints(2).name = "Stick";  joints(2).col_wd = 5; joints(2).col_u = 8;
joints(3).name = "Bucket"; joints(3).col_wd = 6; joints(3).col_u = 9;

% System Parameters
Ts = 0.01;       % Sample time
max_lag = 50;    % Max samples to check for delay
num_bins = 100;  % LUT resolution
smooth_win = 15; % Smoothing factor

% Prepare Figure
figure('Name', 'Hydraulic LUT Generation', 'Color', 'w', 'Position', [50 50 1400 900]);

% --- MAIN LOOP (Process all 3 angles) ---
for i = 1:length(joints)
    
    fprintf('Processing %s...\n', joints(i).name);
    
    % 1. Extract Data
    Wd = T(:, joints(i).col_wd); 
    U  = T(:, joints(i).col_u);
    
    % 2. Automatic Delay Detection
    lags = 0:max_lag;
    correlations = zeros(size(lags));
    
    for k = 1:length(lags)
        d = lags(k);
        U_shifted = U(1:end-d);
        W_truncated = Wd(1+d:end);
        % Use abs() to find best timing regardless of direction
        correlations(k) = abs(corr(U_shifted, W_truncated));
    end
    
    [~, idx] = max(correlations);
    best_delay_samples = lags(idx);
    best_delay_time = best_delay_samples * Ts;
    
    % 3. Align Data
    U_clean = U(1:end-best_delay_samples);
    W_clean = Wd(1+best_delay_samples:end);
    
    % --- DETECT POLARITY (The Fix) ---
    % Check if relation is Positive (Slope Up) or Negative (Slope Down)
    overall_corr = corr(W_clean, U_clean);
    is_inverted = overall_corr < 0;
    
    if is_inverted
        fprintf('  > Detected INVERTED polarity (Negative Slope)\n');
        % Flip command so it looks "Standard" (Positive Slope) for processing
        U_proc = -U_clean; 
    else
        fprintf('  > Detected STANDARD polarity (Positive Slope)\n');
        U_proc = U_clean;
    end
    
    % 4. Generate Basic LUT
    [x_lut, y_lut] = get_lut_curve(W_clean, U_proc, num_bins, smooth_win);
    
    % 5. Enforce Monotonicity (Safety Fix)
    % Now that U_proc is always "Positive Slope", this function works safely
    [x_final, y_final_temp] = force_monotonicity(x_lut, y_lut);
    
    % 6. Restore Polarity
    if is_inverted
        y_final = -y_final_temp; % Flip back to original direction
    else
        y_final = y_final_temp;
    end

    % Store
    joints(i).lut_x = x_final
    joints(i).lut_y = y_final

    % --- PLOTTING ---
    % Row i, Left Plot: Scatter alignment
    subplot(3, 2, (i*2)-1);
    scatter(Wd, U, 5, [0.8 0.8 0.8], 'filled'); hold on;
    scatter(W_clean, U_clean, 5, 'b', 'filled', 'MarkerFaceAlpha', 0.1);
    ylabel(sprintf('%s Cmd', joints(i).name));
    title(sprintf('%s Alignment (Delay: %.2fs)', joints(i).name, best_delay_time));
    grid on; axis tight;
    
    % Row i, Right Plot: Final LUT
    subplot(3, 2, i*2);
    % If inverted, we must flip the 'Raw Smoothed' line manually for the plot
    raw_plot_y = y_lut; 
    if is_inverted, raw_plot_y = -y_lut; end
    
    plot(x_lut, raw_plot_y, 'k--', 'LineWidth', 1.5); hold on;
    plot(x_final, y_final, 'r-', 'LineWidth', 3);
    ylabel('Valve Command');
    title([joints(i).name ' Final LUT']);
    if i == 3; xlabel('Desired Velocity (rad/s)'); end
    legend('Raw Smoothed', 'Safety Monotonic', 'Location', 'best');
    grid on;
    
end

fprintf('\nDone! LUTs generated for all joints.\n');


% --- HELPER FUNCTIONS ---

function [x_out, y_out] = get_lut_curve(x_in, y_in, n_bins, smooth_win)
    edges = linspace(min(x_in), max(x_in), n_bins);
    [idx, ~] = discretize(x_in, edges);
    
    % Filter out NaNs (data outside bins)
    valid = ~isnan(idx);
    idx = idx(valid); x_in = x_in(valid); y_in = y_in(valid);
    
    % Calculate mean X and Y for every bin
    y_means = accumarray(idx, y_in, [], @mean);
    x_means = accumarray(idx, x_in, [], @mean);
    
    % Remove empty bins
    exist = unique(idx);
    y_out = smoothdata(y_means(exist), 'gaussian', smooth_win);
    x_out = x_means(exist);
end

function [x_out, y_out] = force_monotonicity(x_in, y_in)
    % Input: x_in (velocity), y_in (command)
    % Output: Monotonic curve that never "dips" wrongly
    
    x_out = x_in;
    y_out = y_in;
    
    % 1. Find Zero Crossing (Neutral)
    [~, zero_idx] = min(abs(x_in));
    
    % 2. Fix Positive Side (Velocity > 0 implies Command should strictly increase)
    % Use cumulative max to flatten any dips
    if zero_idx < length(y_out)
        pos_segment = y_out(zero_idx:end);
        y_out(zero_idx:end) = max(pos_segment, cummax(pos_segment));
    end
    
    % 3. Fix Negative Side (Velocity < 0 implies Command should strictly decrease)
    % Logic: Flip vector -> treat as positive -> cummax -> flip back -> negate
    if zero_idx > 1
        neg_segment = y_out(1:zero_idx);
        % We want strictly DECREASING as we go left (negative).
        % Let's inverse looking from the center out to the left.
        
        % Flip so index 1 is close to zero, index end is most negative
        neg_flipped = neg_segment(end:-1:1); 
        
        % We expect this to become more negative. 
        % Multiply by -1 to make it "increasing positive", fix it, then revert.
        fixed_flipped = -cummax(-neg_flipped);
        
        % Flip back to original order
        y_out(1:zero_idx) = fixed_flipped(end:-1:1);
    end
end

% --- EXTRACT FINAL LUT VECTORS FOR SIMULINK ---
fprintf('\n\n--- FINAL LUT VECTORS FOR SIMULINK (Copy/Paste Ready) ---\n');

% Function to format vector for display
format_vector = @(v) ['[' sprintf('%.6f, ', v(1:end-1)) sprintf('%.6f]', v(end))];

% Boom Joint LUT
Boom_Vel_LUT = joints(1).lut_x;
Boom_Cmd_LUT = joints(1).lut_y;

fprintf('\n## 💥 BOOM JOINT LUT VECTORS\n');
fprintf('   **Simulink Breakpoints 1 (Velocity):**\n');
fprintf('%s\n', format_vector(Boom_Vel_LUT));
fprintf('   **Simulink Table Data (Command):**\n');
fprintf('%s\n', format_vector(Boom_Cmd_LUT));

% Stick Joint LUT
Stick_Vel_LUT = joints(2).lut_x;
Stick_Cmd_LUT = joints(2).lut_y;

fprintf('\n## 🌳 STICK JOINT LUT VECTORS\n');
fprintf('   **Simulink Breakpoints 1 (Velocity):**\n');
fprintf('%s\n', format_vector(Stick_Vel_LUT));
fprintf('   **Simulink Table Data (Command):**\n');
fprintf('%s\n', format_vector(Stick_Cmd_LUT));

% Bucket Joint LUT
Bucket_Vel_LUT = joints(3).lut_x;
Bucket_Cmd_LUT = joints(3).lut_y;

fprintf('\n## 🧺 BUCKET JOINT LUT VECTORS\n');
fprintf('   **Simulink Breakpoints 1 (Velocity):**\n');
fprintf('%s\n', format_vector(Bucket_Vel_LUT));
fprintf('   **Simulink Table Data (Command):**\n');
fprintf('%s\n', format_vector(Bucket_Cmd_LUT));

%% Neural net STICK

% 1. Load Data
T = readmatrix("training_data.csv");
% Extract Columns
A_boom   = T(:,1); 
A_stick  = T(:,2); 
A_bucket = T(:,3);
Wd_boom = T(:,4);
Wd_stick = T(:,5); % Let's stick with the Stick arm for this example
Wd_bucket = T(:,6);
U_boom = T(:,7);
U_stick  = T(:,8);
U_bucket  = T(:,9);

% --- NEW CODE START ---
% Correction: Stick Angle (A_stick) is 27.5 degrees too much.
% 1. Convert 27.5 degrees to radians:
deg_to_rad = pi / 180;
correction_rad = 27.5 * deg_to_rad; 

% 2. Apply the correction to the entire A_stick column:
A_stick = A_stick - correction_rad;
% --- NEW CODE END ---

% 2. APPLY DELAY COMPENSATION (Crucial Step!)
% Use the delay we found earlier (e.g., 23 samples)
d = 23; 

% We must align ALL inputs to the output
% Inputs (State at time t): Angles and Desired Velocity
% Output (Action at time t): Valve Command (shifted)

% Truncate to valid range
valid_len = length(U_stick) - d;

X_vel    = Wd_stick(1+d : end);      % Aligned Velocity
X_ang1   = A_boom(1+d : end);        % Aligned Angles
X_ang2   = A_stick(1+d : end);
X_ang3   = A_bucket(1+d : end);

Y_target = U_stick(1 : end-d);       % Shifted Valve Command

% 3. Prepare Training Data
% Input Matrix: 4 Rows x N Columns
inputs = [X_vel, X_ang1, X_ang2, X_ang3]'; 
targets = Y_target';

% 4. Create and Train the Network
% We use a "Shallow" network. 
% 15 neurons is enough to learn the shape without "memorizing" noise.
hiddenLayerSize = 15; 
net = fitnet(hiddenLayerSize);

% Setup training parameters (fast and robust)
net.trainFcn = 'trainlm';  % Levenberg-Marquardt (standard for fitting)
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;

fprintf('Training Neural Network LUT...\n');
[net, tr] = train(net, inputs, targets);

% 5. Validate the Results
outputs = net(inputs);
performance = perform(net, targets, outputs);

% 6. Visualization: How Gravity Affects the Valve
% Let's pretend we want 0 velocity (Holding position)
% and see how the valve changes as we move the arm angles.

figure('Name', 'Gravity Compensation Learner');
hold on; grid on;

% Create a sweep of stick angles (while keeping others fixed)
test_angles = linspace(min(X_ang2), max(X_ang2), 100);
dummy_vel = zeros(1, 100); % Zero velocity (holding)
dummy_boom = repmat(mean(X_ang1), 1, 100); % Fixed Boom
dummy_bucket = repmat(mean(X_ang3), 1, 100); % Fixed Bucket

test_inputs = [dummy_vel; dummy_boom; test_angles; dummy_bucket];
predicted_valve = net(test_inputs);

plot(test_angles, predicted_valve, 'r-', 'LineWidth', 3);
xlabel('Stick Angle (rad)');
ylabel('Valve Command required to Hold (Zero Vel)');
title('Learned Gravity Compensation (Holding Current)');

% Save the network for use
save('Stick_SmartLUT.mat', 'net');
% To use later: valve_cmd = net([wd; a1; a2; a3]);

gensim(net)

%% Neural net BOOM
% 1. Load Data
T = readmatrix("training_data.csv");
% Extract Columns
A_boom   = T(:,1); 
A_stick  = T(:,2); 
A_bucket = T(:,3);
Wd_boom  = T(:,4);  % New: Desired Boom Velocity
Wd_stick = T(:,5); 
Wd_bucket= T(:,6);  
U_boom   = T(:,7);  % New: Boom Valve Command (Target)
U_stick  = T(:,8);
U_bucket = T(:,9);  

% --- NEW CODE START ---
% Correction: Stick Angle (A_stick) is 27.5 degrees too much.
% 1. Convert 27.5 degrees to radians:
deg_to_rad = pi / 180;
correction_rad = 27.5 * deg_to_rad; 

% 2. Apply the correction to the entire A_stick column:
A_stick = A_stick - correction_rad;
% --- NEW CODE END ---

% 2. APPLY DELAY COMPENSATION (Crucial Step!)
d = 23; 
% We must align ALL inputs (Angles, Velocities) to the target output (Valve Command)
valid_len = length(U_boom) - d;
% --- INPUTS (State at time t) ---
X_vel    = Wd_boom(1+d : end);      % Aligned Boom Velocity (Flow input)
X_ang1   = A_boom(1+d : end);       % Aligned Boom Angle (Gravity input)
X_ang2   = A_stick(1+d : end);      % Aligned Stick Angle (Gravity input - PLOTTED)
X_ang3   = A_bucket(1+d : end);     % Aligned Bucket Angle (Gravity input)
% --- TARGET (Action at time t+d) ---
Y_target = U_boom(1 : end-d);       % Shifted Boom Valve Command (Target)
% 3. Prepare Training Data
% Input Matrix: 4 Rows x N Columns
inputs = [X_vel, X_ang1, X_ang2, X_ang3]'; 
targets = Y_target';
% 4. Create and Train the Network
hiddenLayerSize = 15; 
net_boom = fitnet(hiddenLayerSize); % Renamed net to net_boom
net_boom.trainFcn = 'trainlm';
net_boom.divideParam.trainRatio = 70/100;
net_boom.divideParam.valRatio = 15/100;
net_boom.divideParam.testRatio = 15/100;
fprintf('Training Neural Network LUT for BOOM...\n');
[net_boom, tr_boom] = train(net_boom, inputs, targets);
% 5. Validate the Results
outputs_boom = net_boom(inputs);
performance_boom = perform(net_boom, targets, outputs_boom);
% 6. Visualization: How Gravity Affects the Valve (1D Plot)
% Plotting Boom Valve command vs STICK Angle (since Stick Angle is the X-axis for the 3D plot)
figure('Name', 'Boom Gravity Compensation Learner');
hold on; grid on;
% Create a sweep of stick angles (X_ang2)
test_angles = linspace(min(X_ang2), max(X_ang2), 100); 
dummy_vel = zeros(1, 100);          % Zero velocity (holding)
dummy_boom = repmat(mean(X_ang1), 1, 100); % Fixed Boom Angle
dummy_bucket = repmat(mean(X_ang3), 1, 100); % Fixed Bucket Angle
test_inputs = [dummy_vel; dummy_boom; test_angles; dummy_bucket]; % Order is crucial
predicted_valve = net_boom(test_inputs);
plot(test_angles, predicted_valve, 'r-', 'LineWidth', 3);
xlabel('Stick Angle (rad)');
ylabel('Valve Command required to Hold (Zero Vel)');
title('Learned Gravity Compensation (Boom Control vs Stick Angle)');
% Save the network for use
save('Boom_SmartLUT.mat', 'net_boom');

gensim(net_boom)
%% Neural Network Inverse Dynamics Model - BOOM JOINT

% 1. Load Data
T = readmatrix("training_data.csv");
% Extract Columns
A_boom   = T(:,1); 
A_stick  = T(:,2); 
A_bucket = T(:,3);
Wd_boom  = T(:,4);  % Desired Boom Velocity
Wd_stick = T(:,5); 
Wd_bucket= T(:,6);  
U_boom   = T(:,7);  % Boom Valve Command (Target)
U_stick  = T(:,8);
U_bucket = T(:,9);  

% 2. APPLY DELAY COMPENSATION (Crucial Step!)
% The delay 'd' aligns the state (angles, velocity) at time t with the 
% action (valve command) at time t+d.
d = 23; 
valid_len = length(U_boom) - d;

% --- INPUTS (State at time t) ---
X_vel    = Wd_boom(1+d : end);      % Aligned Boom Velocity (Flow input)
X_ang1   = A_boom(1+d : end);       % Aligned Boom Angle 
X_ang2   = A_stick(1+d : end);      % Aligned Stick Angle (X-axis for 3D plot)
X_ang3   = A_bucket(1+d : end);     % Aligned Bucket Angle 

% --- TARGET (Action at time t+d) ---
Y_target = U_boom(1 : end-d);       % Shifted Boom Valve Command (Target)

% 3. Prepare Training Data
% Input Matrix: 4 Rows x N Columns: [Velocity; A_boom; A_stick; A_bucket]
inputs = [X_vel, X_ang1, X_ang2, X_ang3]'; 
targets = Y_target';

% 4. Create and Train the Network
hiddenLayerSize = 15; 
net_boom = fitnet(hiddenLayerSize); 
net_boom.trainFcn = 'trainlm';
net_boom.divideParam.trainRatio = 70/100;
net_boom.divideParam.valRatio = 15/100;
net_boom.divideParam.testRatio = 15/100;
fprintf('Training Neural Network LUT for BOOM...\n');
[net_boom, tr_boom] = train(net_boom, inputs, targets);

% 5. Validate the Results
outputs_boom = net_boom(inputs);
performance_boom = perform(net_boom, targets, outputs_boom);
fprintf('Training MSE: %.4f\n', performance_boom);


% 6. Visualization: How Gravity Affects the Valve (1D Plot)
% This plot shows the required valve command when the desired velocity is zero (holding position).
figure('Name', 'Boom Gravity Compensation Learner');
hold on; grid on;
% Create a sweep of stick angles (X_ang2)
test_angles = linspace(min(X_ang2), max(X_ang2), 100); 
dummy_vel = zeros(1, 100);          % Zero velocity (holding)
dummy_boom = repmat(mean(X_ang1), 1, 100); % Fixed Boom Angle
dummy_bucket = repmat(mean(X_ang3), 1, 100); % Fixed Bucket Angle
test_inputs = [dummy_vel; dummy_boom; test_angles; dummy_bucket]; % Order is crucial
predicted_valve = net_boom(test_inputs);
plot(test_angles, predicted_valve, 'r-', 'LineWidth', 3);
xlabel('Stick Angle (rad)');
ylabel('Valve Command required to Hold (Zero Vel)');
title('Learned Gravity Compensation (Boom Control vs Stick Angle)');
% Save the network for use
save('Boom_SmartLUT.mat', 'net_boom');


%% SECTION 7: 3D SURFACE PLOT OF THE LEARNED FUNCTION
% X-axis: Stick Angle (A_stick)
% Y-axis: Desired Boom Velocity (Wd_boom)
% Z-axis: Predicted Boom Valve Command (U_boom)

% 1. Define the range for the two plotted variables (X and Y axes)
angle_range = linspace(min(X_ang2), max(X_ang2), 50); % Stick Angle Range (X_ang2)
velocity_range = linspace(min(X_vel), max(X_vel), 50); % Boom Velocity Range (X_vel)

% 2. Create a grid of all combinations of Angle (X) and Velocity (Y)
[A_grid, V_grid] = meshgrid(angle_range, velocity_range);

% 3. Fix the two non-plotted variables (Boom Angle and Bucket Angle) to their mean
fixed_boom_angle = mean(X_ang1);
fixed_bucket_angle = mean(X_ang3);

% 4. Structure the input data for the Neural Network (4 inputs)
% INPUT ORDER IS CRUCIAL: [Velocity; Boom Angle; Stick Angle; Bucket Angle]
NN_input_matrix = [
    V_grid(:)';                                   % Row 1: Boom Velocity
    repmat(fixed_boom_angle, 1, numel(A_grid));   % Row 2: Fixed Boom Angle
    A_grid(:)';                                   % Row 3: Stick Angle
    repmat(fixed_bucket_angle, 1, numel(A_grid)); % Row 4: Fixed Bucket Angle
];

% 5. Run the Neural Network prediction
Z_pred = net_boom(NN_input_matrix); 

% 6. Reshape the predicted Z-data back into a 50x50 grid
Z_surface = reshape(Z_pred, size(A_grid));

% 7. Create the 3D Plot
figure('Name', '3D Learned Inverse Dynamics - BOOM');
surf(A_grid, V_grid, Z_surface, 'EdgeColor', 'none'); 

% 8. Label and Format the Plot
colormap jet; 
colorbar;    
xlabel('Stick Angle (rad)');
ylabel('Desired Boom Velocity (rad/s)');
zlabel('Predicted Boom Valve Command (%)');
title('Neural Network Inverse Dynamics Surface: Boom Valve = f(Stick Angle, Velocity)');
view(3); 
grid on;


%% SECTION 7: 3D SURFACE PLOT OF THE LEARNED FUNCTION
% X-axis: Stick Angle (A_stick)
% Y-axis: Desired Boom Velocity (Wd_boom)
% Z-axis: Predicted Boom Valve Command (U_boom)

% 1. Define the range for the two plotted variables (X and Y axes)
angle_range = linspace(min(X_ang2), max(X_ang2), 50); % Stick Angle Range (X_ang2)
velocity_range = linspace(min(X_vel), max(X_vel), 50); % Boom Velocity Range (X_vel)

% 2. Create a grid of all combinations of Angle (X) and Velocity (Y)
[A_grid, V_grid] = meshgrid(angle_range, velocity_range);

% 3. Fix the two non-plotted variables (Boom Angle and Bucket Angle) to their mean
fixed_boom_angle = mean(X_ang1);
fixed_bucket_angle = mean(X_ang3);

% 4. Structure the input data for the Neural Network (4 inputs)
% INPUT ORDER IS CRUCIAL: [Velocity; Boom Angle; Stick Angle; Bucket Angle]
NN_input_matrix = [
    V_grid(:)';                                   % Row 1: Boom Velocity
    repmat(fixed_boom_angle, 1, numel(A_grid));   % Row 2: Fixed Boom Angle
    A_grid(:)';                                   % Row 3: Stick Angle
    repmat(fixed_bucket_angle, 1, numel(A_grid)); % Row 4: Fixed Bucket Angle
];

% 5. Run the Neural Network prediction
Z_pred = net_boom(NN_input_matrix);

% 6. Reshape the predicted Z-data back into a 50x50 grid
Z_surface = reshape(Z_pred, size(A_grid));

% 7. Create the 3D Plot
figure('Name', '3D Learned Inverse Dynamics - BOOM');
surf(A_grid, V_grid, Z_surface, 'EdgeColor', 'none'); 

% 8. Label and Format the Plot
colormap jet; 
colorbar;    
xlabel('Stick Angle (rad)');
ylabel('Desired Boom Velocity (rad/s)');
zlabel('Predicted Boom Valve Command (%)');
title('Neural Network Inverse Dynamics Surface: Boom Valve = f(Stick Angle, Velocity)');
view(3); 
grid on;

%% Structured neural net with gravity

% 1. Load and Align Data (Same as before)
T = readmatrix("training_data.csv");
A_boom = T(:,1); A_stick = T(:,2); A_bucket = T(:,3);
Wd_stick = T(:,5); U_stick = T(:,8);

% Apply Delay (e.g., 23 samples)
d = 23;
X_vel = Wd_stick(1+d : end);
X_ang = [A_boom(1+d:end), A_stick(1+d:end), A_bucket(1+d:end)];
Y_valve = U_stick(1 : end-d);

% FILTER: Remove End-Stops (Crucial for Safety)
% Remove data where Valve is high but Velocity is zero (Pushing against wall)
safe_mask = ~(abs(Y_valve) > 20 & abs(X_vel) < 0.05);

X_vel = X_vel(safe_mask);
X_ang = X_ang(safe_mask, :);
Y_valve = Y_valve(safe_mask);


% 2. STEP 1: Train the Flow Network (Velocity Only)
% This learns the S-Curve (Deadband) on average
fprintf('Training Base Flow Network...\n');

net_flow = fitnet(10); % Small network for 1D curve
net_flow.trainParam.showWindow = 0;
[net_flow, tr] = train(net_flow, X_vel', Y_valve');

% Predict the valve based ONLY on velocity
Y_pred_flow = net_flow(X_vel')';


% 3. STEP 2: Train the Gravity Network (The Residuals)
% The "Error" (Residual) from Step 1 is caused by Gravity/Configuration
Residuals = Y_valve - Y_pred_flow;

fprintf('Training Gravity Configuration Network...\n');

% Input: 3 Angles. Output: The "Bias" needed to fix the error.
net_gravity = fitnet([10 5]); % Deeper network for complex geometry
net_gravity.trainParam.showWindow = 0;
[net_gravity, tr] = train(net_gravity, X_ang', Residuals');


% 4. VERIFICATION PLOT
figure('Name', 'Structured Learning Results', 'Color', 'w');

% Plot A: The Flow Curve (What the operator feels)
subplot(1,2,1);
vel_range = linspace(-0.3, 0.3, 100);
plot(vel_range, net_flow(vel_range), 'b-', 'LineWidth', 3);
grid on; title('Learned Flow Curve (Deadband)');
xlabel('Velocity'); ylabel('Valve Command');

% Plot B: The Gravity Bias (What the angles do)
subplot(1,2,2);
% Sweep Stick Angle while holding others constant
test_boom = mean(X_ang(:,1));
test_bucket = mean(X_ang(:,3));
stick_range = linspace(min(X_ang(:,2)), max(X_ang(:,2)), 100);

% Build input matrix for query
query_angles = [repmat(test_boom, 1, 100); stick_range; repmat(test_bucket, 1, 100)];
bias_curve = net_gravity(query_angles);

plot(stick_range, bias_curve, 'r-', 'LineWidth', 3);
grid on; title('Learned Gravity Bias (Shift)');
xlabel('Stick Angle (rad)'); ylabel('Valve Offset');

% Save both networks
save('Excavator_Controller.mat', 'net_flow', 'net_gravity');