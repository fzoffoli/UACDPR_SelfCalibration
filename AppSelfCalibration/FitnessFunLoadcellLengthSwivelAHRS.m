function F = FitnessFunLoadcellLengthSwivelAHRS(cdpr_v,cdpr_p,Z,sensor_disturb)
%FITNESSFUNSWIVELAHRSMOTOR is a function that can be used by an optimization
% algorithm. It computes the minimum singular value of the Identification
% matrix for an initial-pose self-calibration using an AHRS for euler angles
% measurement, swivel angles variation measurements, motor angle variation.
% for cable length measurements and loadcells for tensions.

n_d = length(UACDPR.DependencyVect);
k = length(Z)/n_d;
Z=reshape(Z,[n_d*k 1]);  % ga works with row vectors, flip it

% compute the jacobian matrix
X = Z;
tau = zeros(UACDPR.CablesNumber,k-1);
delta_length = zeros(UACDPR.CablesNumbers,k-1);
delta_sigma = zeros(UACDPR.CablesNumber,k-1);
delta_yaw = zeros(k-1,1);
roll = zeros(k-1,1);
pitch = zeros(k-1,1);
[~,J] = CostFunLoadcellLengthSwivelAHRS(cdpr_v,cdpr_p,X,tau,delta_length,delta_sigma,roll,pitch,delta_yaw,sensor_disturb);

% compute observabilty index
F = -min(svd(J))/max(svd(J));

end