%%% This script is an adaptation of Coccia's script on Initial Pose
%%% Estimation for UACDPRs

clear
close all

% load config file
load('IRMA4U_Mar25.mat');
opts = Utilities;
s.DependencyVect=[1,1,1,0,0,1];
MyUACDPR=UACDPR(s);
n = double(MyUACDPR.CablesNumber);
n_d = length(MyUACDPR.DependencyVect);
P = MyUACDPR.PermutMatrix; 
MyUACDPR= SetOrientType(MyUACDPR,'TaitBryan');
disturb=zeros(6,1);

% write trial file name here:
filename = "sc_8_medium_a";

% load experimental data
if ~isfile(strcat(filename,'_parsed.mat'))
    error("Logging file not handled. Run CdprDataHandling before.");
end
load(strcat(filename,'_parsed.mat'));


% extract calibration data
k = 27;
meas_idx = round(linspace(1,length(st.t),k));
loadcell_meas = st.tensions(:,meas_idx);
cable_length_meas = st.cable_length(:,meas_idx);
swivel_meas = st.swivel(:,meas_idx);
epsilon_meas = st.epsilon(:,meas_idx);

% compute initial eq pose guess
load("sc_control_target_27.mat");
tau_zero = loadcell_meas(:,1);
zita_eq_guess = out.opt_meas_config(:,1);
MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,zita_eq_guess);
fs_opts = opts.FsolveEqPoses;
sol_zeta_lengths = fsolve(@(zita_lengths)DGSP(MyUACDPR,zita_lengths,tau_zero), ...
    [zita_eq_guess;MyUACDPR.Trasmission.CableLengths],fs_opts);
zeta_0_guess = sol_zeta_lengths(1:6);
length_0_guess = sol_zeta_lengths(7:end);

% compute real initial pose (through Gabaldo's pose estimation)
zeta_0_real = ComputePoseEstimationLengthsInclinometer(cable_length_meas(:,1),epsilon_meas(:,1),zita_eq_guess,MyUACDPR);
% % uncomment for pose estimation check
% x = zeros(6,length(st.t));
% for i = 1:length(st.t)
%     x(:,i) = ComputePoseEstimationLengthsInclinometer(st.cable_length(:,i),st.epsilon(:,i),zita_eq_guess,MyUACDPR);
%     zita_eq_guess = x(:,i);
% end
% FilmDrawRobotPippo(x,MyUACDPR);

% Self-Calibration optimization algorithm
sensor_disturb.swivel_noise = deg2rad(1);                               %[rad]
sensor_disturb.length_noise = 0.01;                                     %[m]
sensor_disturb.AHRS_noise = deg2rad(2);                                 %[rad]
sensor_disturb.loadcell_noise = 10;                                     %[N]

delta_swivel_meas = swivel_meas(:,2:end)-swivel_meas(:,1);
delta_length_meas = cable_length_meas(:,2:end)-cable_length_meas(:,1);
roll_meas = epsilon_meas(1,2:end);
pitch_meas = epsilon_meas(2,2:end);
delta_yaw_meas = epsilon_meas(3,2:end)-epsilon_meas(3,1);

% compute initial guess
X_guess = zeros(6,k);
X_guess(:,1) = zeta_0_guess;
for i = 2:length(loadcell_meas)
    length_guess = length_0_guess+delta_length_meas(:,i-1);
    epsilon_guess = [roll_meas(i-1); pitch_meas(i-1); zeta_0_guess(6)] + ...
        [0;0;delta_yaw_meas(i-1)];
    X_guess(:,i) = ComputePoseEstimationLengthsInclinometer(length_guess,epsilon_guess, X_guess(:,i-1),MyUACDPR);
end
X_guess = reshape(X_guess,[k*n_d 1]);


opts = optimoptions('fmincon','FunctionTolerance',1e-10,'OptimalityTolerance',1e-8, ...
    'StepTolerance',1e-10,'UseParallel',true,'MaxFunctionEvaluations',1e+6,'MaxIterations',1e+5);
tic
X_sol = fmincon(@(X)CostFunLengthSwivelAHRS(MyUACDPR,X,k-1,delta_length_meas,...
        delta_swivel_meas,roll_meas,pitch_meas,delta_yaw_meas,sensor_disturb),X_guess,[],[],[],[],[],[],[],opts);
self_calib_comp_time=toc;

InitialPositionErrorNorm = norm(zeta_0_real(1:3)-X_sol(1:3));
MyUACDPR=SetPoseAndUpdate0KIN(MyUACDPR,zeta_0_real(1:6));
angle_init_real = acos((MyUACDPR.EndEffector.RotMatrix(1,1)+MyUACDPR.EndEffector.RotMatrix(2,2)+MyUACDPR.EndEffector.RotMatrix(3,3)-1)/2);
MyUACDPR=SetPoseAndUpdate0KIN(MyUACDPR,X_sol(1:6));
angle_init_sol = acos((MyUACDPR.EndEffector.RotMatrix(1,1)+MyUACDPR.EndEffector.RotMatrix(2,2)+MyUACDPR.EndEffector.RotMatrix(3,3)-1)/2);
InitialOrientationError = rad2deg(abs(angle_init_sol-angle_init_real));