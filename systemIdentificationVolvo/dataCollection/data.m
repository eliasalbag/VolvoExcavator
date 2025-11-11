% %use this script when you want to save a run from the excavator of inputs
% %and outputs. change the name to relevant cylinder
% 
% % Assuming 'out' is already in the workspace
% input = out.stickInput.Data;   % Replace with the actual field name for input data
% output = out.stickAngle.Data;  % Replace with the actual field name for output data
% Ts = 0.01;                   % Sample time
% 
% % Create the iddata object
% idDataStick = iddata(output, input, Ts);
% 
% % Save the iddata object to a .mat file
% save('idDataStick251107.mat', 'idDataStick251107');


%% Align and create iddata object from timeseries

% Extract input and output from relevant structure
input_ts  = RightJoystickInput;   % Timeseries for input
output_ts = stickAngle;   % Timeseries for output

% Sample time (adjust if known)
Ts = 0.01;  % 10 ms sample time

%% Extract time and data
t_in = input_ts.Time;
u = input_ts.Data;

t_out = output_ts.Time;
y = output_ts.Data;

%% Define a common time vector (overlapping range)
t_common_start = max(t_in(1), t_out(1));
t_common_end   = min(t_in(end), t_out(end));
t_common = (t_common_start:Ts:t_common_end)';

%% Resample both signals to the common time base
u_resampled_stick = interp1(t_in, u, t_common, 'linear', 'extrap');
y_resampled_stick = interp1(t_out, y, t_common, 'linear', 'extrap');

% %% Create the iddata object
% idDataBom = iddata(y_resampled, u_resampled, Ts);
% 
% %% Optional: visualize
% figure;
% subplot(2,1,1);
% plot(t_common, u_resampled);
% title('Resampled Input');
% xlabel('Time [s]'); ylabel('Input');
% 
% subplot(2,1,2);
% plot(t_common, y_resampled);
% title('Resampled Output');
% xlabel('Time [s]'); ylabel('Output');
% 
% %% Save the iddata object
% save('idDataBom251107.mat', 'idDataBom');
% 
% disp('✅ idDataBom object created and saved successfully as idDataStick251107.mat');

