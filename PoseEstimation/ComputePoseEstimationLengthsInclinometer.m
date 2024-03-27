function pose_est = ComputePoseEstimationLengthsInclinometer(st,eq_pose,myUACDPR)

% structure picking
dt = st.t(2)-st.t(1);
m = length(st.t);
length_real_meas = st.cable_length + st.length_initial_offset;
Lengths_meas = length_real_meas;
Inc_meas = st.epsilon;

amperr_l = 0.01; %[m]               
amperr_sigma = 3*pi/180; %[rad]           
amperr_rollpitch = 1*pi/180; %[rad]         
amperr_yaw = 3*pi/180; %[rad]    

% Define percentage tollerance on each equation
percentage_toll = 5;
eps = percentage_toll/100;  
eps_firsguess = 1.05;

% WLS - MODIFIED NEWTON - LENGTH ANGLES
S = [];
S = WLSMN_lengths_inc_yaw(S,myUACDPR,amperr_l,amperr_rollpitch,amperr_yaw,eps);

% First solution guess
S.Equations.guess.x = eps_firsguess*eq_pose;
S.Equations.guess.P = eye(6);
for i=1:m
    % Algorithm application
    S.Equations.measures = [Lengths_meas(:,i); Inc_meas(:,i)];

    % POSE ESTIMATION ALGORITHM CALL
    tic
    [Ret,Output] = S.Optimization.Algorithm(S.Equations,S.Optimization);
    comptime(i) = toc;
    %         % Use result as next point solution guess
    S.Equations.guess = Ret;

    % Save Results
    Results.Poses(:,i) = Ret.x;
    if any(isnan(Ret.x))
        %             error('Error, pose element equal to NaN')
    end
    Results.cov_Poses(:,i) = sqrt(diag(Ret.P));
    if i~=1
        Results.Poses_dt(:,i) = (Results.Poses(:,i)-Results.Poses(:,i-1))./dt;
    end
    Results.Residual(:,i) = Output.Residual;
    Results.Stepsize(:,i) = Output.Stepsize;
    Results.Iterations(:,i) = Output.Iterations;
end

pose_est = Results.Poses;
end