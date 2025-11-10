%% ======================= USER DATA SECTION ============================
% Paste/assign your raw vectors and sample times here.
% th1_raw, th2_raw: Nx1 (rad), sampled at Ts_y
% u1_raw, u2_raw  : Mx1 (V),  sampled at Ts_u (about half the length of outputs)
%
% Example placeholders (REMOVE when you paste your real data):
% load your_data.mat  % -> th1_raw, th2_raw, u1_raw, u2_raw, Ts_y, Ts_u

load("systemIdentificationVolvo/bomRunTimeSeries.mat")

% assert(exist('out.bomPos.Data','var')==1 && exist('out.stickPos.Data','var')==1, 'Missing th1_raw/th2_raw');
% assert(exist('out.bomInput.Data','var')==1   && exist('out.stickInput.Data','var')==1, 'Missing u1_raw/u2_raw');
% assert(exist('out.bomPos.TimeInfo.Increment','var')==1, 'Missing Ts_y (output sample time)');
% assert(exist('out.bomInput.TimeInfo.Increment','var')==1, 'Missing Ts_u (input sample time)');

% Optional: pick resampling method for inputs: 'zoh' or 'linear'
RESAMPLE_METHOD = 'zoh';   % zero-order hold (recommended for valve voltages)
% RESAMPLE_METHOD = 'linear';

% Initial guess for input->output transport delay (seconds). Hydraulics often ~20–50 ms.
INIT_DELAY_S = 0.03;  % 30 ms
%% =====================================================================

%% 0) Basic checks
th1_raw = out.bomPos.Data(:);
th2_raw = out.stickPos.Data(:);
u1_raw  = out.bomInput.Data(:);
u2_raw  = out.stickInput.Data(:);

Ts_y = out.bomPos.TimeInfo.Increment;  % Output sample time
Ts_u = out.bomInput.TimeInfo.Increment; % Input sample time

N_y = numel(th1_raw);
assert(numel(th2_raw)==N_y, 'th1_raw and th2_raw must have equal length');

N_u = numel(u1_raw);
assert(numel(u2_raw)==N_u, 'u1_raw and u2_raw must have equal length');

fprintf('Output samples: %d @ Ts_y=%.6g s\n', N_y, Ts_y);
fprintf('Input  samples: %d @ Ts_u=%.6g s\n', N_u, Ts_u);

%% 1) Resample inputs to the output timeline (ZOH or linear)
t_y = (0:N_y-1)' * Ts_y;       % output time stamps
t_u = (0:N_u-1)' * Ts_u;       % input  time stamps

switch lower(RESAMPLE_METHOD)
    case 'zoh'
        % zero-order hold via 'previous' interpolation
        u1 = interp1(t_u, u1_raw, t_y, 'previous', 'extrap');
        u2 = interp1(t_u, u2_raw, t_y, 'previous', 'extrap');
    case 'linear'
        u1 = interp1(t_u, u1_raw, t_y, 'linear', 'extrap');
        u2 = interp1(t_u, u2_raw, t_y, 'linear', 'extrap');
    otherwise
        error('Unknown RESAMPLE_METHOD: %s', RESAMPLE_METHOD);
end

% Stack signals
y = [th1_raw, th2_raw];     % Nx2
u = [u1, u2];               % Nx2 (now matched to outputs)

%% 2) Build causal physics-inspired features (at t-1)
% Create strictly "past" versions to keep ARX causality.
th1m1 = [y(1,1); y(1:end-1,1)];
th2m1 = [y(1,2); y(1:end-1,2)];

sin_th1  = sin(th1m1);
sin_t1p2 = sin(th1m1 + th2m1);
cos_t1p2 = cos(th1m1 + th2m1);

% Causal finite-difference velocities: diff at t uses (t) and (t-1), then shift to be "previous"
dth1_now = [0; diff(y(:,1))] / Ts_y;
dth2_now = [0; diff(y(:,2))] / Ts_y;
dth1 = [dth1_now(1); dth1_now(1:end-1)];   % shift by 1 so it's usable at time t as past info
dth2 = [dth2_now(1); dth2_now(1:end-1)];

% Optional magnitude features for valve nonlinearity/deadband
u1abs = abs(u(:,1));
u2abs = abs(u(:,2));

% Augment the inputs with these exogenous, causal features
u_aug = [u, sin_th1, sin_t1p2, cos_t1p2, dth1, dth2, u1abs, u2abs];
nu_aug = size(u_aug,2);

% Build iddata at output sample time
z_full = iddata(y, u_aug, Ts_y, ...
    'OutputName', {'theta1','theta2'}, ...
    'InputName',  {'u1','u2','sin(th1)','sin(t1+t2)','cos(t1+t2)','dth1','dth2','|u1|','|u2|'});
z_full = detrend(z_full, 0);   % remove means only (preserve operating point)

%% 3) Split into estimation (70%) and validation (30%) — contiguous split
N    = size(z_full.y,1);
N_id = round(0.70 * N);
id   = z_full(1:N_id);
val  = z_full(N_id+1:N);

fprintf('Estimation samples: %d, Validation samples: %d\n', size(id.y,1), size(val.y,1));

%% 4) Define NLARX orders (MIMO): na, nb, nk
% Cross-coupled outputs, modest complexity to start
na = [2 1; 1 2];

% Two taps from each augmented input per output (tune later if needed)
nb = [2*ones(1,nu_aug);
      2*ones(1,nu_aug)];

% Input delay in samples (apply to ALL augmented inputs for simplicity)
nk_samp = max(1, round(INIT_DELAY_S / Ts_y));  % convert seconds to samples
nk = nk_samp * ones(2, nu_aug);

orders = {na, nb, nk};

%% 4b) (Optional but recommended) lighten nb taps for engineered features
% Keep 2 taps for real inputs u1,u2; 1 tap for features (sin/cos/dth/|u|)
% nu_aug = size(u_aug,2); % should already exist
nu_aug = size(u_aug,2);                 % already defined
nb_row  = [2 2  ones(1, nu_aug-2)];     % 2 taps for u1,u2; 1 for engineered features
nb      = [nb_row; nb_row];
orders  = {na, nb, nk};

%% 5) Choose ONE nonlinear mapping (shared by both outputs) and estimate
nl = idSigmoidNetwork('NumberOfUnits', 24);

fprintf('Estimating NLARX (this can take a bit for larger datasets)...\n');
sys = nlarx(id, 'na', na, 'nb', nb, 'nk', nk, 'OutputFcn', nl, ...
            'Focus','simulation', 'Display','on');


%% 6) Validation: 1-step prediction vs free-run simulation
% 1-step prediction on validation set
% 1-step prediction
yp1 = predict(sys, val, 1);
[~, fit_pred] = compare(val, yp1);

% Free-run simulation
ys = sim(sys, val.u);
[~, fit_sim] = compare(val, ys);

fprintf('\n=== Validation metrics (NRMSE %% per output) ===\n');
fprintf('Prediction (1-step):  y1=%5.1f %%   y2=%5.1f %%\n', fit_pred(1), fit_pred(2));
fprintf('Simulation (free):    y1=%5.1f %%   y2=%5.1f %%\n', fit_sim(1),  fit_sim(2));

% Residual checks (should look white and input-independent)
figure; resid(val, sys);

%% 7) Print quick metrics
fprintf('\n=== Validation metrics ===\n');
fprintf('NRMSE fit (prediction, 1-step): %5.1f %%\n', fit_pred);
fprintf('NRMSE fit (simulation, free-run): %5.1f %%\n', fit_sim);

%% 8) Save the model
save('nlarx_two_link_sys.mat', 'sys', 'orders', 'nl', 'nk_samp', 'Ts_y');

%% 9) (Optional) Inspect learned nonlinearities
% getreg(sys)                 % list regressors
% figure; plot(sys.OutputFcn) % visualize the nonlinear maps
