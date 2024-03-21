function Cable = ComputeCable_dpose(Cable,UACDPR)

        Cable.Length_dpose= UACDPR.GeomJac.Cables(Cable.Id,1:6)*UACDPR.EndEffector.D;

        Cable=UpdateModelVectors(Cable,UACDPR);

        Cable=UpdateModelVectors_dpose(Cable,UACDPR);
            
end

