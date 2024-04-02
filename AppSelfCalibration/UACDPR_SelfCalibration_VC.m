%%% This script is an adaptation of Coccia's script on Initial Pose
%%% Estimation for UACDPRs

clear
close all

% load config file
load('UACDPR_LAB3.mat');
opts = Utilities;
s.DependencyVect=[1,1,1,0,0,1];
MyUACDPR=UACDPR(s);
n = double(MyUACDPR.CablesNumber);
P = MyUACDPR.PermutMatrix; 
MyUACDPR= SetOrientType(MyUACDPR,'TaitBryan');
disturb=zeros(6,1);

% load experimental data
load('..\UACDPR_SelfCalibration\FreeDrive60_continuous_motion2_parsed.mat');
% compute equilibrium pose
tau = st.tensions(:,1);
zita_eq_guess = [0;0;1;0;0;0];
fs_opts = opts.FsolveEqPoses;
zita_eq = fsolve(@(zita) Static(zita,MyUACDPR,disturb, tau),zita_eq_guess,fs_opts);

% real pose estimation
length_real = st.cable_length + st.length_initial_offset;
pose_real = ComputePoseEstimationLengthsInclinometer(length_real(:,1),st.epsilon(:,1),zita_eq,MyUACDPR);

% reduce the number of samplings
N = 500;
lengths = st.cable_length(:,1:N:end);
swivel = st.swivel(:,1:N:end);
epsilon = st.epsilon(:,1:N:end);
delta_length = lengths-lengths(:,1);
delta_swivel = swivel-swivel(:,1);

% errors introduction
pos_errors = GenerateSphericalPoints();

for point = 1:length(pos_errors)
    
    
    % guess computation
    tic
    pose_with_errors = pose_real + [pos_errors(:,point); zeros(3,1)];
    MyUACDPR_temp = SetPoseAndUpdate0KIN(MyUACDPR, pose_with_errors);
    length_real_err(:,1) = MyUACDPR_temp.CableLengths_;
    lengths_real_err = delta_length + length_real_err;
    pose_est = ComputePoseEstimationLengthsInclinometer(lengths_real_err,epsilon,zita_eq,MyUACDPR);
%     guess_time = toc

    flag_integration = 0;
    if flag_integration
        dt = st.t(2)-st.t(1);
        Tmax=length(st.tensions)*dt;
        sol = HuenDiscreteSolver(@(t,x) classic_dynamics_log(MyUACDPR, t, x, st.tensions, dt), dt:dt:Tmax, [zita_eq; zeros(6,1)]);
        x = sol.y(1:6,:);
    else
        x = pose_est;
    end

    % % extract data at vicon frequency
    % X = x(:,st_out.vicon_idx);
    % cable_length = st_out.cable_length(:,st_out.vicon_idx);
    % swivel = st_out.swivel(:,st_out.vicon_idx);
    % epsilon = st_out.epsilon(:,st_out.vicon_idx);

    % selfcalibration optimization
    ZV_guess = reshape(x,[6*length(x) 1]);
    opts = optimoptions('fminunc','Display', 'iter-detailed',...
        'FunctionTolerance',1e-2,'MaxFunctionEvaluation',1e12,'SpecifyObjectiveGradient',true,'CheckGradient',false,...
        'MaxIterations',1000,'OptimalityTolerance',1e-3,'StepTolerance',1e-6,'UseParallel',true);
    [ZV,fval] = fminunc(@(ZV) WLS(ZV, MyUACDPR, delta_swivel, delta_length, epsilon),ZV_guess,opts);
    Z = reshape(ZV,[6 length(x)]);
    save("Z",'Z');

    % initial length solution
    load("Z.mat");
    MyUACDPR_temp = SetPoseAndUpdate0KIN(MyUACDPR,Z(:,1));
    length_real_est = MyUACDPR_temp.CableLengths_;
    sigma_real_est = zeros(4,1);
    for j = 1:4
    	sigma_real_est(j,1) = MyUACDPR_temp.Trasmission.Pulley{j}.SwivelAngle;
    end

    % errors
    % err_length = length_real_est - length_real_meas(1);
    err_pos(point) = norm(Z(1:3,1)-pose_real(1:3,1))*1000;
    err_rot(point) = rad2deg(norm(Z(4:6,1)-pose_real(4:6,1)));
    cost_fun(point) = fval;

    point/length(pos_errors)
    opt_time = toc
end

save('convergence_evaluation','err_pos','err_rot','cost_fun');
%% Plot and graphs
figure()
subplot(4,1,1)
plot(cost_fun,'LineWidth',2);
grid on
ylabel('$f(Z)$','interpreter','latex');
xlabel('iterations','interpreter','latex');
subplot(4,1,2)
plot(err_pos,'LineWidth',2);
grid on
ylabel('$\epsilon_p$ [mm]','interpreter','latex');
xlabel('iterations','interpreter','latex');
subplot(4,1,3)
plot(err_rot,'LineWidth',2);
grid on
ylabel('$\epsilon_r$ [deg]','interpreter','latex');
xlabel('iterations','interpreter','latex');
subplot(4,1,4)
plot(vecnorm(pos_errors),'LineWidth',2);
grid on
ylabel('error norm [m]','interpreter','latex');
xlabel('iterations','interpreter','latex');

% FilmDrawRobotPippo(x,MyUACDPR);