classdef KalmanFilter < handle
    properties
        F
        B
        H
        Q
        R
        x           % state vector [angle; velocity]
        P           % covariance
    end

    methods
        function obj = KalmanFilter(dt)
            if nargin < 1
                dt = 0.01;
            end

            obj.F = [1, dt; 0, 1];
            obj.B = [0; 0];
            obj.H = [1, 0;   % encoder angle
                     0, 1;   % encoder velocity
                     1, 0;   % IMU angle
                     0, 1];  % IMU velocity
            obj.Q = [0.01, 0; 0, 0.1];
            obj.x = [0; 0];
            obj.P = [5, 0; 0, 5];
            obj.R = [0.001, 0, 0, 0;    % encoders vinkel är väldigt pålitlig
                0, 0.001, 0, 0;         % encoders hastighet, bra men lite brusig
                0, 0, 0.01, 0;          % IMU vinkel, mer brusig
                0, 0, 0, 0.01];         % IMU hastighet
        end

        function xpred = predict(obj)
            obj.x = obj.F * obj.x;
            obj.P = obj.F * obj.P * obj.F' +obj.Q;
            xpred = obj.x; % Store the predicted state
        end

        function xupd = update(obj, EncoderPos33X, EncoderVel33X, joint1_pos, joint1_vel)
            % z must be 4x1 measurment vector
            S = obj.H * obj.P * obj.H' + obj.R;
            K = obj.P * obj.H' / S;
            y = [EncoderPos33X, EncoderVel33X, joint1_pos, joint1_vel] - obj.H * obj.x;
            obj.x = obj.x + K * y;
            I = eye(size(obj.P));
            obj.P = (I - K * obj.H) * obj.P;
            xupd = obj.x; % Store the updated state
        end
    end
end
