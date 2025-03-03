function [fun,grad] = CostFunLoadcellLengthSwivelAHRS(MyUACDPR, X, tau, delta_length, delta_swivel, roll, pitch, delta_yaw, sensor_disturb)
%CostFunLoadcellLengthSwivelAHRS is the cost function for a self-calibration optimization problem
% where the initial pose of a CDPR has to be estimated using loadcells 
% measures, swivel angle variation measures and euler angles from an AHRS.
% Read Zoffoli2025 Robotica for the complete formulation.

n_d = length(MyUACDPR.DependencyVect);
k = length(X)/n_d;

% set constants for the weighing matrix
force_max = sensor_disturb.loadcell_noise;
length_max = sensor_disturb.length_noise;
sigma_max = sensor_disturb.swivel_noise;
epsilon_max = sensor_disturb.AHRS_noise;

% extract variables and parameters 
zeta_0 = X(1:n_d);
Z = X(n_d+1:length(n_d)*(k+1));
MyUACDPR = MyUACDPR.SetPoseAndUpdate0KIN(zeta_0);
length_0 = MyUACDPR.Trasmission.CableLengths;
sigma_0 = zeros(MyUACDPR.CablesNumber,1);
for j_=1:MyUACDPR.CablesNumber
    sigma_0(j_) = MyUACDPR.Trasmission.Pulley{1,j_}.SwivelAngle;
end
psi_0 = zeta_0(end);

% fill the residual vector
n = MyUACDPR.CablesNumber;
f_tau = zeros(k*6,1);
f_length = zeros(k*n,1);
f_sigma = zeros(k*n,1);
f_epsilon = zeros(k*3,1);
for i=1:k
    zeta_k = Z(i*n_d-(n_d-1):i*n_d);
    MyUACDPR = MyUACDPR.SetPoseAndUpdate0KIN(zeta_k);
    MyUACDPR = ComputeGravityWrench(MyUACDPR);
    length_model = zeros(n,1);
    sigma_model=zeros(n,1);
    for j_=1:n
        length_model(j_) = MyUACDPR.Trasmission.Cable{1,j_}.Length;
        sigma_model(j_) = MyUACDPR.Trasmission.Pulley{1,j_}.SwivelAngle;
    end
    MyUACDPR = ComputeJaclOrtPar(MyUACDPR);
    tau_model = ComputeStaticTensions(UACDPR);
    f_tau(i*n-(n-1):i*n) = tau_model - tau(:,i);
    f_length(i*n-(n-1):i*n) = length_model-length_0-delta_length(:,i); 
    f_sigma(i*n-(n-1):i*n) = sigma_model-sigma_0-delta_swivel(:,i);
    f_epsilon(i*3-2:i*3) = (zeta_k(4:6)-[roll(i);pitch(i);delta_yaw(i)]-[0;0;psi_0]);
end
F = [f_tau; f_length; f_sigma; f_epsilon];

% build the weighing matrix and compute the cost function
W = diag([ones(size(f_tau))./(force_max^2); ...
            ones(size(f_length))./(length_max^2); ...
            ones(size(f_sigma))./(sigma_max^2); ...
            ones(size(f_epsilon))./(epsilon_max^2)]);
fun = 0.5*F'*W*F;
end