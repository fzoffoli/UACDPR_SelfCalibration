% Method to compute active stiffness. 

function UACDPR = ComputeActiveStiffness(UACDPR, tensions)
        % Method to generate active stiffness from geometry and cable tensions
        UACDPR.Trasmission=UACDPR.Trasmission.UpdateTMatrixes; %Updating necessary informations
        K=zeros(6);
        
        for i=1:nnz(UACDPR.DependencyVect)
           %First layer: Support Variable Instantiation
           T=UACDPR.Trasmission.TMatrixes{i};
           tau = tensions(i);
           a=UACDPR.EndEffector.GlobalAttachPoints{i}-UACDPR.EndEffector.Pose(1:3);
           t=UACDPR.Trasmission.Pulley{i}.CableFrame{1};

           %Second layer: Support Variable Instantiation
           alfa=[T              -T*skew(a);
                 skew(a)*T      -skew(a)*T*skew(a)];

           beta=[zeros(3,6);
                 zeros(3)  skew(t)*skew(a)];

           %Active Stiffness computation    
           K=K+tau*(alfa+beta);
        end
        UACDPR.ActiveStiffness=K;

        [~,temp2]=size(UACDPR.GlobalForces.Points);
        phie=zeros(3);


        for i=1:temp2
            phie=phie+skew(-UACDPR.GlobalForces.Forces(:,i))*skew(UACDPR.EndEffector.RotMatrix*UACDPR.GlobalForces.Points(:,i));
        end

        UACDPR.MatrixE=[zeros(3,6);zeros(3) phie];
        UACDPR.MatrixF=zeros(6);

end

