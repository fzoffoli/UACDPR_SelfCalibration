function [c,ceq] = NonlconWorkspaceBelonging(MyUACDPR,Z,ForceBounds)
%NONLCONWORKSPACEBELONGING  is a nonlinear constraint function on a vector 
% of poses Z to ensure that the static equilibrium constraint and tension
% fesibility at each pose

n_d = length(MyUACDPR.DependencyVect);
mu = n_d-MyUACDPR.CablesNumber;
k = length(Z)/n_d;
Z=reshape(Z,[n_d*k 1]);  % ga works with row vectors, so, flip it

eq_stability_constraints = zeros(mu*k,1);
tension_feasibility_constraints = zeros(2*MyUACDPR.CablesNumber*k,1);
for i=1:k
    pose = Z(i*n_d-(n_d-1):i*n_d);
    MyUACDPR = SetPoseAndUpdate0KIN(MyUACDPR,pose);
    MyUACDPR = ComputeJaclOrtPar(MyUACDPR);
    MyUACDPR = ComputeGravityWrench(MyUACDPR);

    tau = ComputeStaticTensions(MyUACDPR);
    tension_feasibility_constraints(2*i*MyUACDPR.CablesNumber-7:2*i*MyUACDPR.CablesNumber) = ... 
        [ones(MyUACDPR.CablesNumber,1).*ForceBounds(1)-tau; tau-ones(MyUACDPR.CablesNumber,1).*ForceBounds(2)];


    eq_stability_constraints(i*mu-1:i*mu) = MyUACDPR.GeomJac.Cables_ort'*MyUACDPR.Wrench;
end

c = tension_feasibility_constraints;
ceq = eq_stability_constraints;

end