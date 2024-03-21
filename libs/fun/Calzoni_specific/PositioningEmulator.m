
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%***********
% Positioning System Emulator based on TWR measurements 
% 
% Copyright by Davide Dardari, 
% Department of Electrical and Information Engineering, 
% University of Bologna
% October 2021
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%**********

clc
close all
clear all

sigma=0.03; % ranging accuracy std (meters)

Latency=0.025; % Positioning latency in seconds

RefreshRate=10; % Refresh rate in Hz


%% Define the position of the anchor nodes (reference nodes)
% The origin of the coordinate reference system is at the center of the handler 
NAnchors=4;
Anchors=zeros(NAnchors,3);

L=1; % Anchor nodes are located at the vertices of the header whose size is 'L' in meters 

Anchors(1,1)=-L/2;   % x coordinate of Anchor 1
Anchors(1,2)=-L/2;   % y coordinate of Anchor 1
Anchors(1,3)=0;      % z coordinate of Anchor 1
Anchors(2,1)=L/2;    % x coordinate of Anchor 2 
Anchors(2,2)=-L/2;   % y coordinate of Anchor 2
Anchors(2,3)=0;      % z coordinate of Anchor 2
Anchors(3,1)=-L/2;   % x coordinate of Anchor 3
Anchors(3,2)=L/2;    % y coordinate of Anchor 3
Anchors(3,3)=0;      % z coordinate of Anchor 3
Anchors(4,1)=L/2;    % x coordinate of Anchor 4
Anchors(4,2)=L/2;    % y coordinate of Anchor 4
Anchors(4,3)=0;      % z coordinate of Anchor 4


% Simulate the movement of the tag along a straigth line (constant speed)

dt=1e-2; % simulation time step (seconds)

T=4; % simulation time (seconds)

N=round(T/dt); % total numer of simulated time instants 

TagSpeed=[1 1 0.95]; % Speed of the tag (vx, vy,vz) m/s
TagPos=[-4 -4 -4]; % Initial tag's position (x,y,z) m


%% Prepare for plotting 
figure
hold on
l1=animatedline('Color',[0 0 1],'LineWidth',2);
l2=animatedline('Color',[1 0 0],'marker','o','LineStyle','none');
xlabel('{\it x} meters');
ylabel('{\it y} meters');
zlabel('{\it z} meters');
axis([-5 5 -5 5 -5 0]);
ax=gca;
ax.FontSize=20;
grid on
view(45,20);

% Plot the reference nodes
for k=1:NAnchors
    plot3(Anchors(k,1),Anchors(k,2),Anchors(k,3),'o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');   
end

legend('True Trajectory','Measurements','Location','northwest');

htext=text(0,0,0,' ','Color','black','FontSize',14);

%% Start simulation
% Simulate the movement of the tag along a straigth line (constant speed)

for n=1:N % discrete time index (the actual time instant is n*dt)
   
   t=n*dt; % current time instant (s)
   
   % new tag actual position 
   TagPos=TagPos+TagSpeed*dt;
   
   % Call the position estimator emulator
   [EstPos,TruePos,ErrStd]=EmulatePosition(TagPos,Anchors,n,dt,sigma,Latency,RefreshRate); 
   
   % draw the actual and estimated position
   addpoints(l1,TagPos(1),TagPos(2),TagPos(3));
   addpoints(l2,EstPos(1),EstPos(2),EstPos(3));
   ss=sprintf('Pos=(%0.2f,%0.2f,%0.2f) [m] -- Time %.3f [secs]',TagPos(1),TagPos(2),TagPos(3),t);
   set(htext,'string',ss);
   set(htext,'Position',TagPos+[0.1,0.1,0.1]);
   drawnow 
end


function [EstPos,TruePos,ErrStd]=EmulatePosition(ActualPos,Anchors,n,dt,sigma,Latency,RefreshRate) 
   % *** Emulate the tag position ****
   % * Input parameters:
   % ActualPos(3): Actual position of the tag (x,y,z)
   % Anchors(n,3): Matrix contaning the coordinates of the reference nodes
   % n: current time instant 
   % dt: time step (seconds)
   % sigma: ranging error standard deviation (meters)
   % Latency: latency in seconds
   % RefreshRate: refresh rate in Hz
   %
   % * Output parameters: 
   % EstPos(3): Instance of the measured tag position (it accounts for latency and refresh rate) 
   % TruePos(3): Actual tag position when the measurement was taken
   % (it accounts for latency and refresh rate)
   % ErrStd(3): Theoretical position estimation error standard deviation when the measurement was taken
   % (it accounts for latency and refresh rate) 
   %
   
   persistent Pos EPos ES;
   
   nl=round(Latency/dt); % Discrete-time latency
   nr=round(1/RefreshRate/dt); % Discrete-time refresh period
   
   if n==1 || mod(n,nr)==0 % generate a measurement every refresh period    
       Pos(n,:)=ActualPos;
       
       %ranges for receiver position estimate
        Ri = sqrt((Anchors(:,1)-ActualPos(1,1)).^2 + (Anchors(:,2)-ActualPos(1,2)).^2+ (Anchors(:,3)-ActualPos(1,3)).^2);

        Dx = (Anchors(:,1)-ActualPos(1))./Ri; % directional derivative
        Dy = (Anchors(:,2)-ActualPos(2))./Ri; % directional derivative
        Dz = (Anchors(:,3)-ActualPos(3))./Ri; % directional derivative

        % Fisher information matrix
        A = horzcat(Dx, Dy, Dz); %[Dx0, Dy0, Dz0; Dx1, Dy1, Dz1;  Dx2, Dy2, Dz2]
        P = inv(A'*A)*sigma^2;
        % GDOP in the x,y, and z axes
        ES(n,1)=sqrt(P(1,1));
        ES(n,2)=sqrt(P(2,2));
        ES(n,3)=sqrt(P(3,3));

       % generate an instance of the Gaussian measurement noise according to the theoretical
       % std
       noise(1)=randn()*ES(n,1);
       noise(2)=randn()*ES(n,2);
       noise(3)=randn()*ES(n,3);

       % add the measurement noise to the true pos 
       EPos(n,:)=ActualPos+noise;
   else 
      EPos(n,:)=EPos(n-1,:);
      Pos(n,:)=Pos(n-1,:);
      ES(n,:)=ES(n-1,:);
   end
   
   % emulate latency
   if n>nl
    TruePos=Pos(n-nl,:);
    EstPos=EPos(n-nl,:);
    ErrStd=ES(n-nl,:);
   else % the first nl timee instant returs zero
    TruePos=[0 0 0];
    EstPos=[0 0 0];
    ErrStd=[0 0 0];
   end
end


