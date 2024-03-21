function UACDPR = InverseDynamic(UACDPR,Wrenchext,Pose,Pose_dt,Pose_ddt)

UACDPR=UACDPR.SetTwistAndUpdateKin(Pose,Pose_dt,Pose_ddt);

UACDPR.Wrench=UACDPR.WrenchPar+Wrenchext;

M=UACDPR.EndEffector.M;
C=UACDPR.EndEffector.C;

Twist=UACDPR.Twist;
Twist_dt=UACDPR.Twist_dt;
GeomJacl=UACDPR.FirstOrderKin.GeomJacl;

tau=linsolve(-GeomJacl,M*Twist_dt+C*Twist-UACDPR.Wrench);

UACDPR.Trasmission.CableTensions=tau;

end

