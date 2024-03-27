% Pose estimation: considers an INCLINOMETER measuring ROLL PITCH and YAW angles

%% _____________________________ LOAD DATA _____________________________ %%
clear all
clc
close all
fig = 0;

% load config file
load('UACDPR_LAB3.mat');
opts = Utilities;
s.DependencyVect=[1,1,1,0,0,1];
myUACDPR=UACDPR(s);
n = double(myUACDPR.CablesNumber);
P = myUACDPR.PermutMatrix; 
myUACDPR= SetOrientType(myUACDPR,'TaitBryan');
disturb=zeros(6,1);

% load experimental data
load('..\UACDPR_SelfCalibration\FreeDrive60_4p_parsed.mat');

% compute equilibrium pose
tau = st.tensions(:,1);
zita_eq_guess = [0;0;1;0;0;0];
fs_opts = opts.FsolveEqPoses;
Poses = fsolve(@(zita) Static(zita,myUACDPR,disturb, tau),zita_eq_guess,fs_opts);

dt = st.t(2)-st.t(1);
m = length(st.t);
length_real_meas = st.cable_length + st.length_initial_offset;
Lengths_meas = length_real_meas;
% Lengths_meas = st.cable_length;
Sigma_meas = st.swivel;
Inc_meas = st.epsilon;

amperr_l = 0.01; %[m]               
amperr_sigma = 3*pi/180; %[rad]           
amperr_rollpitch = 1*pi/180; %[rad]         
amperr_yaw = 3*pi/180; %[rad]    

% amperr_l = mean(abs(Lengths-Lengths_meas),2);               
% amperr_sigma = mean(abs(Sigma-Sigma_meas),2);            
% amperr_rollpitch = mean(abs(Poses(4:5,:)-Inc_meas(1:2,:)),2);      
% amperr_yaw = mean(abs(Poses(6,:)-Inc_meas(3,:)),2);    

% Define percentage tollerance on each equation
percentage_toll = 5;
eps = percentage_toll/100;  
eps_firsguess = 1.05;
% Characters of Methods name
nchar = 14;
j = 0;

%% WLS METHODS DEFINITION
S = [];

% WLS - MODIFIED NEWTON - LENGTH ANGLES
[S,j] = WLSMN_lengths_inc_yaw(S,j,myUACDPR,amperr_l,amperr_rollpitch,amperr_yaw,eps);

%% COMPUTE RESULTS
for jj=1:j
    %         % First solution guess
    S(jj).Equations.guess.x = Poses(:,1);
    S(jj).Equations.guess.P = eye(6);
    for i=1:m 
        % Algorithm application
        S(jj).Equations.measures = [Lengths_meas(:,i); Inc_meas(:,i)];

% POSE ESTIMATION ALGORITHM CALL   
        tic
        [Ret,Output] = S(jj).Optimization.Algorithm(S(jj).Equations,S(jj).Optimization);
        comptime(i) = toc;
%         % Use result as next point solution guess
        S(jj).Equations.guess = Ret;

        % Save Results
        Results.Poses(:,i) = Ret.x;
        if any(isnan(Ret.x))
%             error('Error, pose element equal to NaN')
        end
        Results.cov_Poses(:,i) = sqrt(diag(Ret.P));
%         Results.err_Poses(:,i) = abs(Results.Poses(:,i) - Poses(:,i));
        if i~=1
            Results.Poses_dt(:,i) = (Results.Poses(:,i)-Results.Poses(:,i-1))./dt;
        end
        Results.Residual(:,i) = Output.Residual;
        Results.Stepsize(:,i) = Output.Stepsize;
        Results.Iterations(:,i) = Output.Iterations;    
    end    
    Results.StepComputingTime = comptime;
    Results.ComputingTime = sum(comptime);
    Results.StepMediumComputingTime = Results.ComputingTime/m;
    % Save Results in structure
    clear S(jj).Equations.measures;
    S(jj).Results = Results;
    clear Results;    
end

%% PLOTS
plot_legend_plus{1} = "Vicon";
for jj=1:j  
    plot_legend_plus{jj+1} = S(jj).Method;
end
for jj=1:j  
    plot_legend{jj} = S(jj).Method;
end
lw = 1.5;
ls = '-';

% % % % % % % % % Dynamic plot motion
% % Measure parsing
% n_draw_poses = 1000;
% d = floor(m/n_draw_poses);
% jj = 1;
% for k = 1:n_draw_poses
%     Poses_draw(:,k) = S(jj).Results.Poses(:,k*d);
% end
% Poses_draw = [Poses_draw S(jj).Results.Poses(:,end)];
% FilmDrawRobot(Poses_draw,myUACDPR);

% % % % % % % % Dynamic plot motion
% Measure parsing
% n_draw_poses = 1200;
% Poses_draw = spline(1:1:m,S(jj).Results.Poses,linspace(1,m,n_draw_poses));
% FilmDrawRobot(Poses_draw,myUACDPR);

t = linspace(0,t(end)-t(1),length(t));

% Computed Newton poses plot x[cm], y[cm], z[cm], roll[°], pitch[°], yaw[°]
% fig = fig+1;
% fh= figure(fig);
% fh.WindowState = 'maximized';
% subplot_label= {'$x$','$y$','$z$'};
% y_label = {'[cm]','[cm]','[cm]'};
% coeff = [100; 100; 100];
% for kk=1:3
%     subplot(3,1,kk)
%     hold on
%     grid on
%     plot(t,Poses(kk,:)*coeff(kk),'LineWidth',lw);
%     for jj=1:j        
%         plot(t,S(jj).Results.Poses(kk,:)*coeff(kk),ls,'LineWidth',lw);  
%     end
%     hold off
%     ylim([-60 60]);
%     legend(plot_legend_plus,'Interpreter','Latex','FontSize',14,'Location','eastoutside');    
%     subtitle(subplot_label{kk},'Interpreter','Latex','FontSize',18);
%     xlabel('$t$ [s]','Interpreter','Latex','FontSize',18);
%     ylabel(y_label{kk},'Interpreter','Latex','FontSize',18);
% end
% fig = fig+1;
% fh= figure(fig);
% fh.WindowState = 'maximized';
% subplot_label= {'$\phi$','$\theta$','$\psi$'};
% y_label = {'[$^\circ$]','[$^\circ$]','[$^\circ$]'};
% coeff = [180/pi; 180/pi; 180/pi];
% for kk=1:3
%     subplot(3,1,kk)
%     hold on
%     grid on
%     plot(t,Poses(kk+3,:)*coeff(kk),'LineWidth',lw);
%     for jj=1:j        
%         plot(t,S(jj).Results.Poses(kk+3,:)*coeff(kk),ls,'LineWidth',lw);  
%     end
%     hold off
%     ylim([-15 15]);
%     legend(plot_legend_plus,'Interpreter','Latex','FontSize',14,'Location','eastoutside');
%     subtitle(subplot_label{kk},'Interpreter','Latex','FontSize',18);
%     xlabel('$t$ [s]','Interpreter','Latex','FontSize',18);
%     ylabel(y_label{kk},'Interpreter','Latex','FontSize',18);
% end

% Computed Newton poses plot x[cm], y[cm], z[cm], roll[°], pitch[°], yaw[°]
fig = fig+1;
fh= figure(fig);
fh.WindowState = 'maximized';
subplot_label= {'$x$','$y$','$z$'};
y_label = {'[cm]','[cm]','[cm]'};
coeff = [100; 100; 100];
v = [1,3,5];
for kk=1:3
    subplot(3,2,v(kk))
    hold on
    grid on
    plot(t,Poses(kk,:)*coeff(kk),'LineWidth',lw);
    for jj=1:j        
        plot(t,S(jj).Results.Poses(kk,:)*coeff(kk),ls,'LineWidth',lw);  
    end
    hold off
    ylim([-60 60]);
%     legend(plot_legend_plus,'Interpreter','Latex','FontSize',14,'Location','eastoutside');    
    subtitle(subplot_label{kk},'Interpreter','Latex','FontSize',18);
    xlabel('$t$ [s]','Interpreter','Latex','FontSize',18);
    ylabel(y_label{kk},'Interpreter','Latex','FontSize',18);
end
subplot_label= {'$\phi$','$\theta$','$\psi$'};
y_label = {'[$^\circ$]','[$^\circ$]','[$^\circ$]'};
coeff = [180/pi; 180/pi; 180/pi];
v = [2,4,6];
for kk=1:3
    subplot(3,2,v(kk))
    hold on
    grid on
    plot(t,Poses(kk+3,:)*coeff(kk),'LineWidth',lw);
    for jj=1:j        
        plot(t,S(jj).Results.Poses(kk+3,:)*coeff(kk),ls,'LineWidth',lw);  
    end
    hold off
    ylim([-15 15]);
    subtitle(subplot_label{kk},'Interpreter','Latex','FontSize',18);
    xlabel('$t$ [s]','Interpreter','Latex','FontSize',18);
    ylabel(y_label{kk},'Interpreter','Latex','FontSize',18);
end
% legend({'Vicon','Alg. A','Alg. B'},'Interpreter','Latex','FontSize',14,'Location','southoutside');
legend(plot_legend_plus,'Interpreter','Latex','FontSize',14,'Location','eastoutside');

% % Computed Newton ERROR on poses plot x[cm], y[cm], z[cm], roll[°], pitch[°], yaw[°]
% fig = fig+1;
% fh= figure(fig);
% fh.WindowState = 'maximized';
% subplot_label= {'$\epsilon_x=|x-x_{C}|$','$\epsilon_y=|y-y_{C}|$','$\epsilon_z=|y-y_{C}|$'};
% y_label = {'[cm]','[cm]','[cm]'};
% coeff = [100; 100; 100];
% for kk=1:3
%     subplot(3,1,kk)
%     for jj=1:j
%         hold on
%         grid on
%         plot(t,S(jj).Results.err_Poses(kk,:)*coeff(kk),ls,'LineWidth',lw);  
%     end
%     hold off
%     legend(plot_legend,'Interpreter','Latex','FontSize',14,'Location','eastoutside');
%     subtitle(subplot_label{kk},'Interpreter','Latex','FontSize',18);
%     xlabel('$t$ [s]','Interpreter','Latex','FontSize',18);
%     ylabel(y_label{kk},'Interpreter','Latex','FontSize',18);
% end
% fig = fig+1;
% fh= figure(fig);
% fh.WindowState = 'maximized';
% subplot_label= {'$\epsilon_\phi=|\phi-\phi_{C}|$','$\epsilon_\theta=|\theta-\theta_{C}|$','$\epsilon_\psi=|\psi-\psi_{C}|$'};
% y_label = {'[$^\circ$]','[$^\circ$]','[$^\circ$]'};
% coeff = [180/pi; 180/pi; 180/pi];
% for kk=1:3
%     subplot(3,1,kk)
%     for jj=1:j
%         hold on
%         grid on
%         plot(t,S(jj).Results.err_Poses(kk+3,:)*coeff(kk),ls,'LineWidth',lw);  
%     end
%     hold off
%     legend(plot_legend,'Interpreter','Latex','FontSize',14,'Location','eastoutside');
%     subtitle(subplot_label{kk},'Interpreter','Latex','FontSize',18);
%     xlabel('$t$ [s]','Interpreter','Latex','FontSize',18);
%     ylabel(y_label{kk},'Interpreter','Latex','FontSize',18);
% end

% Residuals, steps, number of function evaluations and step medium computing time plot
fig = fig+1;
fh= figure(fig);
fh.WindowState = 'maximized';
subplot(4,1,1)
    for jj=1:j
        hold on
        grid on
        plot(t,S(jj).Results.Residual,ls,'LineWidth',lw);        
    end
    hold off
    legend(plot_legend,'Interpreter','Latex','FontSize',14,'Location','eastoutside');
    subtitle('$\| F(\zeta_i) \|$','Interpreter','Latex','FontSize',18);
    xlabel('t [s]','Interpreter','Latex','FontSize',18);
subplot(4,1,2)
    for jj=1:j
        hold on
        grid on
        plot(t,S(jj).Results.Stepsize,ls,'LineWidth',lw);        
    end
    hold off
    legend(plot_legend,'Interpreter','Latex','FontSize',14,'Location','eastoutside');
    subtitle('$\| \mathrm{d}\zeta_i \| $','Interpreter','Latex','FontSize',18);
    xlabel('t [s]','Interpreter','Latex','FontSize',18);
subplot(4,1,3)
    for jj=1:j
        hold on
        grid on
        plot(t,S(jj).Results.Iterations,ls,'LineWidth',lw);        
    end
    hold off
    legend(plot_legend,'Interpreter','Latex','FontSize',14,'Location','eastoutside');
    subtitle('Numero di iterazioni $i$','Interpreter','Latex','FontSize',18);
    xlabel('t [s]','Interpreter','Latex','FontSize',18);
subplot(4,1,4)
    for jj=1:j
        hold on
        grid on
        plot(t,S(jj).Results.StepComputingTime*1000,ls,'LineWidth',lw);        
    end
    hold off
    legend(plot_legend,'Interpreter','Latex','FontSize',14,'Location','eastoutside');
    subtitle('Tempo computazionale','Interpreter','Latex','FontSize',18);
    xlabel('t [s]','Interpreter','Latex','FontSize',18);
    ylabel('t [ms]','Interpreter','Latex','FontSize',18);

% % Medium error
% EM = cell(7,j+1);
% EM{1,1} = 'Err med:';
% EM{2,1} = 'x [cm]';
% EM{3,1} = 'y [cm]';
% EM{4,1} = 'z [cm]';
% EM{5,1} = 'roll [°]';
% EM{6,1} = 'pitch [°]';
% EM{7,1} = 'yaw [°]';
% for jj=1:j
%     EM{1,jj+1} = plot_legend{jj};
% end
% for kk = 1:3
%     for jj = 1:j
%         EM{kk+1,jj+1} = sum(abs(S(jj).Results.err_Poses(kk,:)))/length(S(jj).Results.err_Poses(kk,:))*100;
%     end
% end
% for kk = 4:6
%     for jj = 1:j
%         EM{kk+1,jj+1} = sum(abs(S(jj).Results.err_Poses(kk,:)))/length(S(jj).Results.err_Poses(kk,:))*180/pi;
%     end
% end
% 
% EM

% Medium error and percentage
s = 0;
EM = cell(7,j+1+1);
EM{1,1} = 'Err med:';
EM{2,1} = 'x [cm]';
EM{3,1} = 'y [cm]';
EM{4,1} = 'z [cm]';
EM{5,1} = 'roll [°]';
EM{6,1} = 'pitch [°]';
EM{7,1} = 'yaw [°]';
for jj=1:j
    EM{1,jj+1} = plot_legend{jj};
end
for kk = 1:3
    for jj = 1:j
        EM{kk+1,jj+1} = sum(abs(S(jj).Results.err_Poses(kk,:)))/length(S(jj).Results.err_Poses(kk,:))*100;
    end
    EM{kk+1,4} = abs(EM{kk+1,3}-EM{kk+1,2})/EM{kk+1,3}*100;
    s = s+EM{kk+1,4};
end
for kk = 4:6
    for jj = 1:j
        EM{kk+1,jj+1} = sum(abs(S(jj).Results.err_Poses(kk,:)))/length(S(jj).Results.err_Poses(kk,:))*180/pi;
    end
    EM{kk+1,4} = abs(EM{kk+1,3}-EM{kk+1,2})/EM{kk+1,3}*100;
    s = s+EM{kk+1,4};
end

EM
s = s/6;
s

% % Maximum error
% EMAX = cell(7,j+1);
% EMAX{1,1} = 'Err MAX:';
% EMAX{2,1} = 'x [cm]';
% EMAX{3,1} = 'y [cm]';
% EMAX{4,1} = 'z [cm]';
% EMAX{5,1} = 'roll [°]';
% EMAX{6,1} = 'pitch [°]';
% EMAX{7,1} = 'yaw [°]';
% for jj=1:j
%     EMAX{1,jj+1} = plot_legend{jj};
% end
% for kk = 1:3
%     for jj = 1:j
%         EMAX{kk+1,jj+1} = max(abs(S(jj).Results.err_Poses(kk,:)))*100;
%     end
% end
% for kk = 4:6
%     for jj = 1:j
%         EMAX{kk+1,jj+1} = max(abs(S(jj).Results.err_Poses(kk,:)))*180/pi;
%     end
% end
% 
% EMAX