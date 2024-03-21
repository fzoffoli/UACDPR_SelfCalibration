function R = matrix_matrixwrtvector(M,NV)
    % M: a x b matrix
    % NV: b x c x d matrix (derivative of matrix N wrt vector V)
    [Mr,Mc] = size(M);
    [Nr,Nc,V] = size(NV);
    
    R = zeros(Mr,Nc,V);
    if Mc==Nr
        for i=1:Mr
            for j=1:Nc
                for k=1:V
                    for s=1:Mc
                        R(i,j,k)=R(i,j,k)+M(i,s)*NV(s,j,k);
                    end
                end
            end
        end
    end
end