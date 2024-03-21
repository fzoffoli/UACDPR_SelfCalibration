function Cable = UpdateModelVectors(Cable,UACDPR)
    
    Cable.ModelVectors.ap= UACDPR.EndEffector.RotMatrix*UACDPR.EndEffector.Platform_Param.AttachPoint{Cable.Id};

    Cable.ModelVectors.varrho= UACDPR.EndEffector.GlobalAttachPoints{Cable.Id} - UACDPR.Trasmission.Pulley{Cable.Id}.CableEnterPosition;  
    Cable.ModelVectors.varrho_u= (Cable.ModelVectors.varrho)'*UACDPR.Trasmission.Pulley{Cable.Id}.PulleyFrame{1};

    Cable.ModelVectors.rho= Cable.CableVector ;
    
end

