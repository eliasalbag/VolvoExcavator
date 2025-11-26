clear; clc; close all;

% 1. Load Data
% Ensure training_data.csv is in your path
if isfile("training_data.csv")
    T = readmatrix("training_data.csv");
else
    error("training_data.csv not found. Please ensure the file exists.");
end

% -------------------------------------------------------------------
% --- CONSTANTS & PRE-PROCESSING ---
% -------------------------------------------------------------------
STICK_ANGLE_COL = 2; 
VALVE_COMMAND_COLS = [7, 8]; 
DEGREES_TO_RADIANS = pi / 180;
RADIANS_TO_DEGREES = 1 / DEGREES_TO_RADIANS; % New constant for plotting
ANGLE_OFFSET_RAD = 27.5 * DEGREES_TO_RADIANS; 

% Apply the subtraction to the stick angle column
T(:, STICK_ANGLE_COL) = T(:, STICK_ANGLE_COL) - ANGLE_OFFSET_RAD;

% --- DATA CLEANING: CLIPPING SATURATED COMMANDS ---
CMD_CLIP_MAX = 95;
CMD_CLIP_MIN = -95;

T(:, VALVE_COMMAND_COLS) = max(min(T(:, VALVE_COMMAND_COLS), CMD_CLIP_MAX), CMD_CLIP_MIN);

% -------------------------------------------------------------------

% --- GLOBAL DATA ---
A_boom_col = 1;
A_stick_col = 2; 

% --- CONFIGURATION ---
joints = struct();

% Boom Joint Config
joints(1).name = "Boom";   
joints(1).col_wd = 4; 
joints(1).col_u = 7;  
joints(1).polarity = 'inverted'; % Boom: Wd > 0 requires U < 0

% Stick Joint Config
joints(2).name = "Stick";  
joints(2).col_wd = 5; 
joints(2).col_u = 8;
joints(2).polarity = 'normal'; % Stick: Wd > 0 requires U > 0

% Parameters
Ts = 0.01;        % Sample time
max_lag = 50;     % Max samples to check for delay
vel_bins = 50;    % Resolution for Axes
ang_bins = 50;    % Resolution for Angles
smooth_sigma = 3.0; % Smoothing Factor
smooth_kernel = [11 11 11]; % Kernel size for wider smoothing


% Prepare Figure
figure('Name', '3D Hydraulic LUT Generation', 'Color', 'w', 'Position', [50 50 1400 700]);

% --- MAIN LOOP ---
for i = 1:length(joints)
    
    fprintf('Processing %s Joint (Inputs: Velocity, Boom Ang, Stick Ang)...\n', joints(i).name);
    
    % 1. Extract Raw Columns
    Wd_raw = T(:, joints(i).col_wd); 
    U_raw  = T(:, joints(i).col_u);
    
    % Global Angle Columns (Raw)
    Boom_Ang_raw  = T(:, A_boom_col);
    Stick_Ang_raw = T(:, A_stick_col);
    
    % 2. Automatic Delay Detection (Correlation between Velocity and Command)
    lags = 0:max_lag;
    correlations = zeros(size(lags));
    
    for k = 1:length(lags)
        d = lags(k);
        U_shifted = U_raw(1:end-d);
        W_truncated = Wd_raw(1+d:end);
        correlations(k) = abs(corr(U_shifted, W_truncated));
    end
    
    [~, idx] = max(correlations);
    best_delay_samples = lags(idx);
    fprintf('  > Best Delay: %d samples (%.2fs)\n', best_delay_samples, best_delay_samples*Ts);
    
    % 3. Align Data
    range_in = 1+best_delay_samples:length(U_raw);
    range_out = 1:length(U_raw)-best_delay_samples;
    
    U_clean   = U_raw(range_out);
    W_clean   = Wd_raw(range_in);
    Boom_clean  = Boom_Ang_raw(range_in);
    Stick_clean = Stick_Ang_raw(range_in);
    
    % --- FORCE COLUMN VECTORS ---
    U_clean = U_clean(:); W_clean = W_clean(:); Boom_clean = Boom_clean(:); Stick_clean = Stick_clean(:);
    
    % 4. --- IMPLEMENT QUADRANTAL CONSTRAINTS (Data Filtering) ---
    quadrant_mask = true(size(W_clean));
    
    if strcmp(joints(i).polarity, 'inverted') % Boom: Wd > 0 -> U < 0, Wd < 0 -> U > 0
        quadrant_mask = quadrant_mask & ~((W_clean > 0) & (U_clean >= 0));
        quadrant_mask = quadrant_mask & ~((W_clean < 0) & (U_clean <= 0));
        
    elseif strcmp(joints(i).polarity, 'normal') % Stick: Wd > 0 -> U > 0, Wd < 0 -> U < 0
        quadrant_mask = quadrant_mask & ~((W_clean > 0) & (U_clean <= 0));
        quadrant_mask = quadrant_mask & ~((W_clean < 0) & (U_clean >= 0));
    end
    
    W_proc_filtered = W_clean(quadrant_mask);
    U_proc_filtered = U_clean(quadrant_mask);
    Boom_filtered = Boom_clean(quadrant_mask);
    Stick_filtered = Stick_clean(quadrant_mask);
    
    fprintf('  > Quadrantal Constraint applied. Removed %d data points.\n', length(W_clean) - length(W_proc_filtered));
    
    % 5. --- DETECT POLARITY & PREPARE FOR UNIFORM BINNING (Magnitude preparation) ---
    overall_corr = corr(W_proc_filtered, U_proc_filtered);
    is_inverted = overall_corr < 0; 
    
    if is_inverted
        fprintf('  > Polarity detected (Inverted). Flipping command for positive magnitude processing.\n');
        U_binned = -U_proc_filtered; 
    else
        fprintf('  > Polarity detected (Normal). Keeping command as is for positive magnitude processing.\n');
        U_binned = U_proc_filtered;
    end
    
    % 6. Generate 3D Grid (Centers)
    vel_edges  = linspace(min(W_clean), max(W_clean), vel_bins + 1);
    boom_edges = linspace(min(Boom_clean), max(Boom_clean), ang_bins + 1);
    stick_edges = linspace(min(Stick_clean), max(Stick_clean), ang_bins + 1);
    
    vel_centers  = (vel_edges(1:end-1) + vel_edges(2:end)) / 2;
    boom_centers = (boom_edges(1:end-1) + boom_edges(2:end)) / 2;
    stick_centers = (stick_edges(1:end-1) + stick_edges(2:end)) / 2;
    
    % Create Grid based on Centers
    [V_GRID, B_GRID, S_GRID] = ndgrid(vel_centers, boom_centers, stick_centers);
    
    % 7. Binning and Averaging (Using Filtered Data)
    v_idx = discretize(W_proc_filtered, vel_edges);
    b_idx = discretize(Boom_filtered, boom_edges);
    s_idx = discretize(Stick_filtered, stick_edges);
    
    valid_bin = ~isnan(v_idx) & ~isnan(b_idx) & ~isnan(s_idx);
    v_idx = v_idx(valid_bin); b_idx = b_idx(valid_bin); s_idx = s_idx(valid_bin); U_valid = U_binned(valid_bin);
    
    % Force indices/data to be columns
    v_idx = v_idx(:); b_idx = b_idx(:); s_idx = s_idx(:); U_valid = U_valid(:);
    
    % Accumulate into 3D Array
    subs = [v_idx, b_idx, s_idx]; 
    sz = [length(vel_centers), length(boom_centers), length(stick_centers)];
    
    sum_grid = accumarray(subs, U_valid, sz);
    count_grid = accumarray(subs, 1, sz);
    
    % Calculate Mean
    LUT_Vol = sum_grid ./ count_grid;
    LUT_Vol(count_grid == 0) = NaN; 
    
    % 8. Fill Gaps & Smooth (3D)
    fprintf('  > Interpolating gaps and smoothing...\n');
    
    % Find valid voxels
    idx_lin = find(~isnan(LUT_Vol));
    [v_sub, b_sub, s_sub] = ind2sub(sz, idx_lin);
    vals = LUT_Vol(idx_lin);
    
    % Create 3D interpolant
    points = [vel_centers(v_sub)', boom_centers(b_sub)', stick_centers(s_sub)'];
    F = scatteredInterpolant(points, vals, 'linear', 'nearest');
    
    % Evaluate on full grid
    LUT_Filled = F(V_GRID, B_GRID, S_GRID);
    
    % Smooth 3D Volume
    LUT_Final_Proc = smooth3(LUT_Filled, 'gaussian', smooth_kernel, smooth_sigma); 
    
    % 9. Restore Polarity
    if is_inverted
        LUT_Final = -LUT_Final_Proc;
    else
        LUT_Final = LUT_Final_Proc;
    end
    
    % 10. --- ENFORCE QUADRANTAL CONSTRAINT ON FINAL SMOOTHED LUT ---
    [V_INDEX, ~, ~] = ndgrid(1:length(vel_centers), 1:length(boom_centers), 1:length(stick_centers));
    
    V_POS_MASK = vel_centers(V_INDEX) > 0;
    V_NEG_MASK = vel_centers(V_INDEX) < 0;
    
    if strcmp(joints(i).polarity, 'inverted') % Boom: V_POS -> U <= 0, V_NEG -> U >= 0
        LUT_Final(V_POS_MASK) = min(LUT_Final(V_POS_MASK), 0);
        LUT_Final(V_NEG_MASK) = max(LUT_Final(V_NEG_MASK), 0);
        
    elseif strcmp(joints(i).polarity, 'normal') % Stick: V_POS -> U >= 0, V_NEG -> U <= 0
        LUT_Final(V_POS_MASK) = max(LUT_Final(V_POS_MASK), 0);
        LUT_Final(V_NEG_MASK) = min(LUT_Final(V_NEG_MASK), 0);
    end
    
    % Store Results
    joints(i).lut_3d = LUT_Final;
    joints(i).axes = {vel_centers, boom_centers, stick_centers};
    
    % 11. --- PLOTTING (Slice Planes) - CONVERT TO DEGREES FOR DISPLAY ---
    subplot(1, 2, i);
    
    % Convert angle coordinates to degrees for plotting axes
    B_DEG = boom_centers * RADIANS_TO_DEGREES;
    S_DEG = stick_centers * RADIANS_TO_DEGREES;

    % Prepare meshgrid for plotting (V=Dim2, B=Dim1, S=Dim3)
    [V_PLOT, B_PLOT_DEG, S_PLOT_DEG] = meshgrid(vel_centers, B_DEG, S_DEG);
    LUT_PLOT = permute(LUT_Final, [2 1 3]);
    
    % Define slice planes in the new degree coordinates
    x_slice = []; 
    y_slice = mean(B_DEG);  
    z_slice = mean(S_DEG); 
    
    % Plot using the Degree Grid
    h = slice(V_PLOT, B_PLOT_DEG, S_PLOT_DEG, LUT_PLOT, x_slice, y_slice, z_slice);
    set(h, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    
    title(sprintf('%s Joint 3D LUT', joints(i).name));
    xlabel('Velocity (rad/s)');
    ylabel('Boom Angle (deg)'); % Updated label
    zlabel('Stick Angle (deg)'); % Updated label
    
    cb = colorbar;
    cb.Label.String = 'Valve Command';
    colormap jet;
    
    view(3); grid on;
    axis tight;
    
end

fprintf('\nDone. Access results in "joints(i).lut_3d".\n');
fprintf('Dimensions are: Velocity x BoomAngle x StickAngle\n');

% -------------------------------------------------------------------------
% --- SPECIFIC ANALYSIS: BOOM VALVE COMMAND SLICE AT Wd = 0.1 rad/s ---
% -------------------------------------------------------------------------

if length(joints) >= 1 && strcmp(joints(1).name, 'Boom')
    j = joints(1); % Boom Joint is index 1
    
    % Define the target velocity slice
    Wd_target = 0.1;
    
    % Extract the axes (still in Radians)
    vel_centers   = j.axes{1}; 
    boom_centers  = j.axes{2}; 
    stick_centers = j.axes{3}; 
    
    % Find the index closest to the target velocity
    [~, v_idx] = min(abs(vel_centers - Wd_target));
    
    % Extract the 2D slice
    Cmd_Slice = squeeze(j.lut_3d(v_idx, :, :));
    
    % --- CONVERT ANGLES TO DEGREES FOR 2D PLOT ---
    B_DEG = boom_centers * RADIANS_TO_DEGREES;
    S_DEG = stick_centers * RADIANS_TO_DEGREES;

    % Create the 2D grid for plotting Boom Angle (X) vs Stick Angle (Y)
    [B_PLOT_2D_DEG, S_PLOT_2D_DEG] = meshgrid(B_DEG, S_DEG);
    
    % Cmd_Plot_Data must be transposed to match meshgrid
    Cmd_Plot_Data = Cmd_Slice';
    
    
    figure('Name', sprintf('Boom Command at Wd = %.2f rad/s', Wd_target), ...
           'Color', 'w', 'Position', [100 700 800 600]);
    
    % Use surf plot with Degree Grid
    surf(B_PLOT_2D_DEG, S_PLOT_2D_DEG, Cmd_Plot_Data, 'EdgeColor', 'none');
    
    title(sprintf('Boom Valve Command (Wd = %.2f rad/s Slice) - Final Constraint Applied', Wd_target));
    xlabel('Boom Angle (deg)'); % Updated label
    ylabel('Stick Angle (deg)'); % Updated label
    zlabel('Boom Valve Command');
    
    % Reset Z-axis limits for the specific slice plot to avoid saturating colorbar
    zlim([CMD_CLIP_MIN, CMD_CLIP_MAX]);
    
    cb = colorbar;
    cb.Label.String = 'Valve Command';
    colormap jet;
    
    view(3); grid on;
    
    fprintf('\nGenerated 2D surface plot for Boom Command at Wd = %.2f rad/s.\n', Wd_target);

else
    fprintf('\nSkipping specific analysis: Boom joint data not found or not processed.\n');
end

% -------------------------------------------------------------------------
% 12. --- EXPORT FINAL LUT VECTORS AND DATA FOR ND LOOKUP TABLE BLOCK ---
% -------------------------------------------------------------------------

% Note: The axis vectors (Boom Angle, Stick Angle) are exported in Radians 
% because they correspond directly to the input columns used in the model.

% --- BOOM JOINT (Index 1) ---
fprintf('\n======================================================\n');
fprintf('  > BOOM JOINT LOOKUP TABLE VARIABLES (DIM: V x B x S)\n');
fprintf('======================================================\n');

if length(joints) >= 1 && strcmp(joints(1).name, 'Boom')
    j_boom = joints(1);
    
    % Dimension 1: Velocity (rad/s)
    Boom_Vel_Vector = j_boom.axes{1};
    fprintf('BOOM_VEL_VECTOR (Input 1, rad/s): Length %d\n', length(Boom_Vel_Vector));
    
    
    % Dimension 2: Boom Angle (rad)
    Boom_Ang_Vector = j_boom.axes{2};
    fprintf('BOOM_ANGLE_VECTOR (Input 2, rad): Length %d\n', length(Boom_Ang_Vector));
    
    
    % Dimension 3: Stick Angle (rad)
    Stick_Ang_Vector_Boom = j_boom.axes{3};
    fprintf('STICK_ANGLE_VECTOR_BOOM (Input 3, rad): Length %d\n', length(Stick_Ang_Vector_Boom));
   
    
    % Table Data (Valve Command)
    Boom_LUT_Data = j_boom.lut_3d; % Dimensions: Velocity x Boom Angle x Stick Angle
    fprintf('BOOM_LUT_DATA (Output, Command): Size [%s]\n', num2str(size(Boom_LUT_Data)));
    
else
    fprintf('Boom data is not available.\n');
end


% --- STICK JOINT (Index 2) ---
fprintf('\n======================================================\n');
fprintf('  > STICK JOINT LOOKUP TABLE VARIABLES (DIM: V x B x S)\n');
fprintf('======================================================\n');

if length(joints) >= 2 && strcmp(joints(2).name, 'Stick')
    j_stick = joints(2);
    
    % Dimension 1: Velocity (rad/s)
    Stick_Vel_Vector = j_stick.axes{1};
    fprintf('STICK_VEL_VECTOR (Input 1, rad/s): Length %d\n', length(Stick_Vel_Vector));
   
    
    % Dimension 2: Boom Angle (rad)
    Boom_Ang_Vector_Stick = j_stick.axes{2};
    fprintf('BOOM_ANGLE_VECTOR_STICK (Input 2, rad): Length %d\n', length(Boom_Ang_Vector_Stick));
   
    
    % Dimension 3: Stick Angle (rad)
    Stick_Ang_Vector = j_stick.axes{3};
    fprintf('STICK_ANGLE_VECTOR (Input 3, rad): Length %d\n', length(Stick_Ang_Vector));

    
    % Table Data (Valve Command)
    Stick_LUT_Data = j_stick.lut_3d; % Dimensions: Velocity x Boom Angle x Stick Angle
    fprintf('STICK_LUT_DATA (Output, Command): Size [%s]\n', num2str(size(Stick_LUT_Data)));

else
    fprintf('Stick data is not available.\n');
end
fprintf('======================================================\n');