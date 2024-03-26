% this function is the dynamics in state form of a generic UACDPR, where 
% are explicit the joint variables, to compute the direct dynamics
% HIP. The free variables are pitch and yaw

function x_dt = DynamicsJointSpace(MyUACDPR,t,x)

zeta_f = x(1:2);

% set up control law
l = ones(4,1);
l_dt = ones(4,1);

% zero order kinematics
Disturb=zeros(6,1);
zeta_c_guess = zeros(size(l));
zeta_c = fsolve(@(zeta_c)SetControlledPose0KIN(MyUACDPR,zeta_c,zeta_f,l),zeta_c_guess); % zero kin controlled variables
MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,pose);

pose_dt=(MyUACDPR.EndEffector.D\eye(6))*x(7:end);
MyUACDPR = SetPoseAndUpdate1KIN(MyUACDPR,pose,pose_dt);

MyUACDPR = MyUACDPR.ComputeWrench(Disturb);
MyUACDPR = ComputeJaclOrtPar(MyUACDPR);
MyUACDPR = ComputeJaclOrtPar_dt(MyUACDPR);
JacPar = ComputeParJac(MyUACDPR);
[h_dt, l_dt] = diffeomorphism(MyUACDPR);
JacPar_dt = ComputeParJac_dt(MyUACDPR);
[ort, par] = ComputeParOrtMatrices(MyUACDPR,JacPar,JacPar_dt);


M = MyUACDPR.EndEffector.M;
C = MyUACDPR.EndEffector.C;
f = MyUACDPR.Wrench;

J  = MyUACDPR.GeomJac.Cables(1:4,:).'; %estrazione jacobiana per i cavi controllati in tensione (6x4)

%%%%%% BUILDING CONTROL LAW
k=int64(t/dt);
kp=10;
kd=1;
e(:,k)=MyUACDPR.Trasmission.CableLengths-l_ss;
% tau = -par.C_h*h_dt-par.C_l*l_dt+par.f + kp*e(:,k)+kd*l_dt;
beta=0.1;
tau = SlidingModeControl(MyUACDPR,beta,kd,kp,h_dt,l_dt,ort,par,e(:,k),k);

B=-C*MyUACDPR.Twist+f-J*tau;
twist_dt=linsolve(M,B);

x_dt=[pose_dt;twist_dt];

end