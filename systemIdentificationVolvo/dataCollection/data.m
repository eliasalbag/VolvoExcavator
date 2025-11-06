%use this script when you want to save a run from the excavator of inputs
%and outputs. change the name to relevant cylinder

% Assuming 'out' is already in the workspace
input = out.bucketInput.Data;   % Replace with the actual field name for input data
output = out.bucketAngle.Data;  % Replace with the actual field name for output data
Ts = 0.01;                   % Sample time

% Create the iddata object
idDataBucket = iddata(output, input, Ts);

% Save the iddata object to a .mat file
save('idDataBucket.mat', 'idDataBucket');