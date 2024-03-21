function Trasmission = UpdateTMatrixes(Trasmission)
%Method to Update matrix T as for Edo PhD Chapter 2.4

% Trasmission.TMatrixes=cell(1,length(Trasmission.CableLengths));
    for i=1:length(Trasmission.Pulley)
        % support variables instantiation
        psi = Trasmission.Pulley{i}.TangencyAngle;
        w = Trasmission.Pulley{i}.PulleyFrame{2};
        n = Trasmission.Pulley{i}.CableFrame{3};
        rho_u = dot(Trasmission.Pulley{i}.DA,Trasmission.Pulley{i}.PulleyFrame{1});
        
        % matrix computation
        Trasmission.TMatrixes{1,i} = sin(psi)*(w*w.')/rho_u+(n*n.')/norm(Trasmission.Cable{i}.CableVector);
    end
    
    
end

