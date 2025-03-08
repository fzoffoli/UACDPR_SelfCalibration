%%% This script is an adaptation of Coccia's script on Initial Pose
%%% Estimation for UACDPRs

clear
close all

InitialPoseSelfCalibMeasureComputation

% load config file
load('IRMA4U_Mar25.mat');
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
length_real_meas = st.cable_length + st.length_initial_offset;
pose_real = ComputePoseEstimationLengthsInclinometer(length_real_meas(:,1),st.epsilon(:,1),zita_eq,MyUACDPR);

% % extract data at vicon frequency
% X = x(:,st_out.vicon_idx);
% cable_length = st_out.cable_length(:,st_out.vicon_idx);
% swivel = st_out.swivel(:,st_out.vicon_idx);
% epsilon = st_out.epsilon(:,st_out.vicon_idx);

% reduce the number of samplings
N = 500;
swivel = st.swivel(:,1:N:end);
epsilon = st.epsilon(:,1:N:end);
cable_lengths = st.cable_length(:,1:N:end);

% set zero delta_l and delta_sigma
delta_swivel = swivel-swivel(:,1);
delta_length = cable_lengths-cable_lengths(:,1);


% convergence evaluation
position_error = GenerateSphericalPoints();
position_error(:,end+1) = zeros(3,1);
for p_num = 1:length(position_error)

    % error computation
    pose_with_errors = pose_real + [position_error(:,p_num); zeros(3,1)];
    MyUACDPR_temp = SetPoseAndUpdate0KIN(MyUACDPR, pose_with_errors);
    length_real_err = MyUACDPR_temp.CableLengths_;
    lengths_real_err = delta_length + length_real_err;

    % pose estimation
    pose_est = ComputePoseEstimationLengthsInclinometer(lengths_real_err,epsilon,zita_eq,MyUACDPR);

    % integration for guess computation
    flag_integration = 0;
    if flag_integration
        dt = st.t(2)-st.t(1);
        Tmax=length(st.tensions)*dt;
        sol = HuenDiscreteSolver(@(t,x) classic_dynamics_log(MyUACDPR, t, x, st.tensions, dt), dt:dt:Tmax, [zita_eq; zeros(6,1)]);
        x = sol.y(1:6,:);
    else
        x = pose_est;
    end

    % selfcalibration optimization
    ZV_guess = reshape(x,[6*length(x) 1]);
    opts = optimoptions('fminunc','Display', 'iter-detailed',...
        'FunctionTolerance',1e-3,'MaxFunctionEvaluation',1e12,'SpecifyObjectiveGradient',true,'CheckGradient',false,...
        'MaxIterations',1000,'OptimalityTolerance',1e-4,'StepTolerance',1e-6,'UseParallel',true);
    [ZV, fval] = fminunc(@(ZV) WLS(ZV, MyUACDPR, delta_swivel, delta_length, epsilon),ZV_guess,opts);
    Z = reshape(ZV,[6 length(x)]);

    % initial length solution
    temp = SetPoseAndUpdate0KIN(MyUACDPR,Z(:,1));
    length_real_est = temp.CableLengths_;
    sigma_real_est = zeros(4,1);
    for j = 1:4
    	sigma_real_est(j,1) = temp.Trasmission.Pulley{j}.SwivelAngle;
    end

    % selfcalibration results
    % err_length = length_real_est - length_real_meas(1);
    err_pos(p_num) = norm(Z(1:3,1)-pose_real(1:3,1))*1000;
    err_rot(p_num) = rad2deg(norm(Z(4:6,1)-pose_real(4:6,1)));
    cost_fun(p_num) = fval;
    initial_pose(:,p_num) = Z(:,1);

    simulation_percentage=p_num/length(position_error)
end

save('selfcalib_convergence_analysis_40cm_4a_8p.mat','err_rot','err_pos',"cost_fun",'initial_pose','position_error');
%% Plots and graphs
figure()
subplot(4,1,1)
plot(cost_fun,'LineWidth',2);
grid on
xlabel('iterations','Interpreter','latex')
ylabel('$f(\mathbf{Z}$)','Interpreter','latex')
subplot(4,1,2)
plot(err_pos,'LineWidth',2);
grid on
xlabel('iterations','Interpreter','latex')
ylabel('$\mathbf{\epsilon}_p$ [mm]','Interpreter','latex')
subplot(4,1,3)
plot(err_rot,'LineWidth',2);
grid on
xlabel('iterations','Interpreter','latex')
ylabel('$\mathbf{\epsilon}_r$ [deg]','Interpreter','latex')
subplot(4,1,4)
plot(vecnorm(position_error),'LineWidth',2);
grid on
xlabel('iterations','Interpreter','latex')
ylabel('$\mathbf{e}_P$ [m]','Interpreter','latex')


% guess video
% FilmDrawRobotPippo(x,MyUACDPR);