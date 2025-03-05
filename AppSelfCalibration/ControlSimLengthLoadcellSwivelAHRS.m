function [X_real, loadcell_meas, delta_length_meas, delta_sigma_meas, delta_psi_meas, phi_meas, theta_meas] = ControlSimLengthLoadcellSwivelAHRS(MyUACDPR,Z_ideal,k,control_disturb,sensor_disturb)

n_d = length(MyUACDPR.DependencyVect);

% add control disturbances
pose_bias = repmat([control_disturb.position_bias*ones(3,1);...
    control_disturb.orientation_bias*ones(3,1)],k,1);
pose_noise = zeros(n_d*k,1);
for i=1:k
    pose_noise(i*6-5:i*6) = [control_disturb.position_noise*(2*rand-1); ...
        control_disturb.position_noise*(2*rand-1);
        control_disturb.position_noise*(2*rand-1);
        control_disturb.orientation_noise*(2*rand-1);
        control_disturb.orientation_noise*(2*rand-1);
        control_disturb.orientation_noise*(2*rand-1)];
end
Z_real=Z_ideal+pose_bias+pose_noise;

% compute sensor disturbances
loadcell_noise = sensor_disturb.loadcell_noise;
length_noise = sensor_disturb.length_noise;
swivel_noise = sensor_disturb.swivel_noise;
AHRS_noise = sensor_disturb.AHRS_noise;

% data acquisition simulation through IK
loadcell_opt_meas = zeros(MyUACDPR.CablesNumber,k);
delta_length_opt_meas = zeros(MyUACDPR.CablesNumber,k);
delta_sigma_opt_meas = zeros(MyUACDPR.CablesNumber,k);
delta_psi_opt_meas = zeros(k,1);
phi_opt_meas = zeros(k,1);
theta_opt_meas = zeros(k,1);
loadcell_meas = zeros(MyUACDPR.CablesNumber,k);
delta_length_meas = zeros(MyUACDPR.CablesNumber,k);
delta_sigma_meas = zeros(MyUACDPR.CablesNumber,k);
delta_psi_meas = zeros(k,1);
phi_meas = zeros(k,1);
theta_meas = zeros(k,1);
length_0 = zeros(MyUACDPR.CablesNumber,1);
sigma_0 = zeros(MyUACDPR.CablesNumber,1);
psi_0 = 0;
for i = 1:k
    zeta_i = Z_real(6*i-5:6*i);
    MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,zeta_i);
    tau = CalcInverseStaticsAndGradient(MyUACDPR,zeta_i);

    % delta length and swivel IK simulation
    for j = 1:MyUACDPR.CablesNumber
        if i==1
            length_0(j) = MyUACDPR.Trasmission.Cable{1,j}.Length;
            sigma_0(j) = MyUACDPR.Trasmission.Pulley{1,j}.SwivelAngle;
            delta_length_opt_meas(j,i) = 0;
            delta_sigma_opt_meas(j,i) = 0;
            delta_length_meas(j,i) = delta_length_opt_meas(j,i)+(2*rand-1)*length_noise;
            delta_sigma_meas(j,i) = delta_sigma_opt_meas(j,i)+(2*rand-1)*swivel_noise;
        else
            delta_length_opt_meas(j,i) = MyUACDPR.Trasmission.Cable{1,j}.Length-length_0(j);
            delta_length_meas(j,i) = delta_length_opt_meas(j,i)+(2*rand-1)*length_noise;
            delta_sigma_opt_meas(j,i) = MyUACDPR.Trasmission.Pulley{1,j}.SwivelAngle-sigma_0(j);
            delta_sigma_meas(j,i) = delta_sigma_opt_meas(j,i)+(2*rand-1)*swivel_noise;
        end
    end
    % delta yaw IK simulation and control law assignment
    if i==1
        psi_0 = MyUACDPR.EndEffector.Pose(6);
        delta_psi_opt_meas(i) = 0;
        delta_psi_meas(i) = delta_psi_opt_meas(i)+(2*rand-1)*AHRS_noise;
        loadcell_opt_meas(:,i) = zeros(MyUACDPR.CablesNumber,1);
        loadcell_meas(:,i) = loadcell_opt_meas(:,i)+(2*rand(MyUACDPR.CablesNumber,1) ...
            -ones(MyUACDPR.CablesNumber,1))*loadcell_noise;
    else
        delta_psi_opt_meas(i) = MyUACDPR.EndEffector.Pose(6)-psi_0;
        delta_psi_meas(i) = delta_psi_opt_meas(i)+(2*rand-1)*AHRS_noise;
        loadcell_opt_meas(:,i) = tau;
        loadcell_meas(:,i) = loadcell_opt_meas(:,i)+(2*rand(MyUACDPR.CablesNumber,1) ...
            -ones(MyUACDPR.CablesNumber,1))*loadcell_noise;
    end
    % roll and pitch IK simulation
    phi_opt_meas(i) = MyUACDPR.EndEffector.Pose(4);
    theta_opt_meas(i) = MyUACDPR.EndEffector.Pose(5);
    phi_meas(i) = phi_opt_meas(i)+(2*rand-1)*AHRS_noise;
    theta_meas(i) = theta_opt_meas(i)+(2*rand-1)*AHRS_noise;
end

loadcell_meas(:,1) = [];
delta_length_meas(:,1) = [];
delta_sigma_meas(:,1) = [];
delta_psi_meas(1) = [];
phi_meas(1) = [];
theta_meas(1) = [];

X_real = Z_real;
end