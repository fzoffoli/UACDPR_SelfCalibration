function UACDPR = ComputeJac_dpose(UACDPR)
% To call this 0th order kinematics has to be computed - function SetPoseAndUpdate0KIN(UACDPR,Pose)
    n= length(UACDPR.EndEffector.LocalAttachPoints_);
    Xil_dpose=zeros(n,6,6);
    Xis_dpose=zeros(n,6,6);
    Xit_dpose=zeros(n,6,6); 
    
    UACDPR.EndEffector = ComputeRotMatrix_dpose(UACDPR.EndEffector);
    UACDPR.EndEffector = ComputeHD_dpose(UACDPR.EndEffector);

     for i=1:n     

        UACDPR.Trasmission.Pulley{i}= UACDPR.Trasmission.Pulley{i}.ComputePulley_dpose(UACDPR);
        UACDPR.Trasmission.Cable{i}= UACDPR.Trasmission.Cable{i}.ComputeCable_dpose(UACDPR);

        Xil_dpose(i,1:3,:)= permute(UACDPR.Trasmission.Pulley{i}.CableFrame_dpose{1},[3 1 2]);
        Xil_dpose(i,4:6,:)= permute(...
            (skew(UACDPR.EndEffector.LocalAttachPoints_{i})*UACDPR.Trasmission.Pulley{i}.CableFrame_dpose{1} - ...
            skew(UACDPR.Trasmission.Pulley{i}.CableFrame{1})*UACDPR.Trasmission.Cable{i}.ModelVectors_dpose.ap),...
            [1 3 2]);               
        
        Xis_dpose(i,1:3,:)= permute(UACDPR.Trasmission.Pulley{i}.PulleyFrame_dpose{2},[3 1 2]);
        Xis_dpose(i,4:6,:)= permute((skew(UACDPR.EndEffector.LocalAttachPoints_{i})*UACDPR.Trasmission.Pulley{i}.PulleyFrame_dpose{2} - ...
            skew(UACDPR.Trasmission.Pulley{i}.PulleyFrame{2})*UACDPR.Trasmission.Cable{i}.ModelVectors_dpose.ap),[3 1 2]);
        Xis_dpose(i,:,:)= -1/UACDPR.Trasmission.Cable{i}.ModelVectors.varrho_u   *matrixwrtvector_matrix(UACDPR.Trasmission.Cable{i}.ModelVectors_dpose.varrho_u,...
            UACDPR.GeomJac.Swivel(i,1:6)) + 1/UACDPR.Trasmission.Cable{i}.ModelVectors.varrho_u   *Xis_dpose(i,:,:);

        Xit_dpose(i,1:3,:)= permute(UACDPR.Trasmission.Pulley{i}.CableFrame_dpose{3},[3 1 2]);
        Xit_dpose(i,4:6,:)= permute((skew(UACDPR.EndEffector.LocalAttachPoints_{i})*UACDPR.Trasmission.Pulley{i}.CableFrame_dpose{3} - ...
            skew(UACDPR.Trasmission.Pulley{i}.CableFrame{3})*UACDPR.Trasmission.Cable{i}.ModelVectors_dpose.ap),[3 1 2]);
        Xit_dpose(i,:,:)= matrixwrtvector_matrix(...
            permute(UACDPR.Trasmission.Cable{i}.ModelVectors_dpose.inv_norm_rho,[1 3 2]),...
            norm(UACDPR.Trasmission.Cable{i}.ModelVectors.rho)*UACDPR.GeomJac.Tangency(i,1:6)) + ...
            1/norm(UACDPR.Trasmission.Cable{i}.ModelVectors.rho) *Xit_dpose(i,:,:);
        
    end

    UACDPR.GeomJac_dpose.Cables = Xil_dpose;
    UACDPR.GeomJac_dpose.Swivel = Xis_dpose;
    UACDPR.GeomJac_dpose.Tangency = Xit_dpose;

    UACDPR.AnalJac_dpose.Cables = matrixwrtvector_matrix(Xil_dpose,UACDPR.EndEffector.D) + ...
                                  matrix_matrixwrtvector(UACDPR.GeomJac.Cables,UACDPR.EndEffector.D_dpose);
    UACDPR.AnalJac_dpose.Swivel = matrixwrtvector_matrix(Xis_dpose,UACDPR.EndEffector.D) + ...
                                  matrix_matrixwrtvector(UACDPR.GeomJac.Swivel,UACDPR.EndEffector.D_dpose);
    UACDPR.AnalJac_dpose.Tangency = matrixwrtvector_matrix(Xit_dpose,UACDPR.EndEffector.D) + ...
                                    matrix_matrixwrtvector(UACDPR.GeomJac.Tangency,UACDPR.EndEffector.D_dpose);
end