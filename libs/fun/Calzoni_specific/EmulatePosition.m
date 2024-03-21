
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
   
   persistent Pos EPos ES start_flag;
   

   nl=round(Latency/dt); % Discrete-time latency
   nr=round(1/RefreshRate/dt); % Discrete-time refresh period
   
   if n==1 || mod(n,nr)==0 %|| start_flag==1% generate a measurement every refresh period    
       
       start_flag=0;
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

