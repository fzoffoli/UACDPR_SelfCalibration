%%% This script allows to find the rotation matrices between the vicon
%%% frame and the inclinometer frame, with the aim to compare the results
%%% of the two sensors. For the sampled data is MANDATORY to 
%%%     - switch on the inclinometer (launch the app) inside the workspace;
%%%     - excite before the sampling the orientation of the platform enough to
%%%         let the inclinometer calibrate itself;
%%%     - start the acquisition from the homing base and bring the
%%%         platform where it can be excited.
%%% INPUT a structure containing two matrices with the orientation
%%% samplings of the inclinometers and the vicon aligned in time and with
%%% the same sampling frequency
%%% OUTPUT the value of the RPY angles to build the constant rotation
%%% matrix between the mobile frames and the value of the RPY angles to
%%% build the constant rotation matrix between the fixed frames


clear
close all

sensors_data = load('DataframeRot01.mat');
eps_vicon = sensors_data.vicon;
eps_inclinometer = sensors_data.inclinometer;
eps_inclinometer_base = sensors_data.inclinometer_base;
t_cut=sensors_data.t_cut;

phi_0 = mean(eps_inclinometer_base(3,:));
% n = 4200;
% eps_vicon = rand(3,n);
% eps_inclinometer = rand(size(eps_vicon));

fun = @(x)V_I_transformation(x,eps_vicon,eps_inclinometer,phi_0); % fun = err_eps_vicon /------/  x = [eps_c; eps_vi]
x0=zeros(6,1); % hip small difference between mobile and fixed frames
x = lsqnonlin(fun,x0);

eps_c = x(1:3);
eps_vi = x(4:6);
R_c = RotMatrixRPY(eps_c);
R_vi = RotMatrixRPY(eps_vi);

save('RotMatCalib01.mat',"R_c","R_vi","eps_c","eps_vi")

% load("RotMatrices4.mat")
%%% check if rotation matrices are valid
eps_inclinometer_to_vicon=zeros(size(eps_inclinometer));
R_z0 = Rz(phi_0);
for i = 1:length(eps_inclinometer)
    R_i = R_z0'*RotMatrixRPY(eps_inclinometer(:,i));
    eps_inclinometer_to_vicon(:,i) = Angles_RPY(R_c*R_i*R_vi);
end

% figure()
% subplot(3,1,1)
% plot(t_cut,eps_vicon(1,:),t_cut,eps_inclinometer_to_vicon(1,:))
% legend('v','i')
% grid on
% subplot(3,1,2)
% hold on
% grid on
% plot(t_cut,eps_vicon(2,:),t_cut,eps_inclinometer_to_vicon(2,:))
% legend('v','i')
% hold on
% subplot(3,1,3)
% hold on
% grid on
% plot(t_cut,eps_vicon(3,:),t_cut,eps_inclinometer_to_vicon(3,:))
% legend('v','i')

% figure()
% grid on
% subplot(3,1,1)
% plot(t_cut,eps_vicon(1,:)*180/pi,t_cut,eps_inclinometer_to_vicon(1,:)*180/pi)
% legend('v','i')
% grid on
% subplot(3,1,2)
% hold on
% grid on
% plot(t_cut,eps_vicon(2,:)*180/pi,t_cut,eps_inclinometer_to_vicon(2,:)*180/pi)
% legend('v','i')
% hold on
% subplot(3,1,3)
% hold on
% grid on
% plot(t_cut,eps_vicon(3,:)*180/pi,t_cut,eps_inclinometer_to_vicon(3,:)*180/pi)
% legend('v','i')

figure()
grid on
subplot(3,1,1)
plot(t_cut,eps_vicon(1,:)*180/pi,t_cut,eps_inclinometer(1,:)*180/pi,t_cut,eps_inclinometer_to_vicon(1,:)*180/pi)
legend('v','i','i_c')
grid on
subplot(3,1,2)
hold on
grid on
plot(t_cut,eps_vicon(2,:)*180/pi,t_cut,eps_inclinometer(2,:)*180/pi,t_cut,eps_inclinometer_to_vicon(2,:)*180/pi)
legend('v','i','i_c')
hold on
subplot(3,1,3)
hold on
grid on
plot(t_cut,eps_vicon(3,:)*180/pi,t_cut,eps_inclinometer(3,:)*180/pi,t_cut,eps_inclinometer_to_vicon(3,:)*180/pi)
legend('v','i','i_c')


save('epsilon_i_50N04A25FoptPhase_fromBase',"eps_inclinometer_to_vicon")