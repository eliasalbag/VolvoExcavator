%% Align and create iddata object from timeseries
RightJoystickInput = out{7}.Values;
LeftJoystickInput = out{6}.Values;

bomAngle = out{3}.Values;
stickAngle = out{2}.Values;

% Extract input and output from relevant structure
% Stick
input_ts_stick  = RightJoystickInput;   % Timeseries for input
output_ts_stick = stickAngle;            % Timeseries for output

% Bom
input_ts_bom = LeftJoystickInput;        % Timeseries for input
output_ts_bom = bomAngle;                % Timeseries for output

% Sample time (adjust if known)
Ts = 0.01;  % 10 ms sample time

%% Extract time and data
% Stick
t_in_stick = input_ts_stick.Time;
u_stick = input_ts_stick.Data;

t_out_stick = output_ts_stick.Time;
y_stick = output_ts_stick.Data;

% Bom
t_in_bom = input_ts_bom.Time;
u_bom = input_ts_bom.Data;  % Corrected variable name from u_stick to u_bom

t_out_bom = output_ts_bom.Time;
y_bom = output_ts_bom.Data;

%% Define a common time vector (overlapping range)
% Stick
t_common_start_stick = max(t_in_stick(1), t_out_stick(1));
t_common_end_stick = min(t_in_stick(end), t_out_stick(end));
t_common_stick = (t_common_start_stick:Ts:t_common_end_stick)';

% Bom
t_common_start_bom = max(t_in_bom(1), t_out_bom(1));
t_common_end_bom = min(t_in_bom(end), t_out_bom(end));
t_common_bom = (t_common_start_bom:Ts:t_common_end_bom)';

%% Resample both signals to the common time base
% Stick
u_resampled_stick = interp1(t_in_stick, u_stick, t_common_stick, 'linear', 'extrap');
y_resampled_stick = interp1(t_out_stick, y_stick, t_common_stick, 'linear', 'extrap');

% Bom
u_resampled_bom = interp1(t_in_bom, u_bom, t_common_bom, 'linear', 'extrap');  % Corrected variable name
y_resampled_bom = interp1(t_out_bom, y_bom, t_common_bom, 'linear', 'extrap');

%% Create iddata objects
data_stick = iddata(y_resampled_stick, u_resampled_stick, Ts);
data_bom = iddata(y_resampled_bom, u_resampled_bom, Ts);
