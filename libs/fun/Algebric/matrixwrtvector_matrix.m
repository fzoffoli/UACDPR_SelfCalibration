function R = matrixwrtvector_matrix(MV,N)
    % M: a x b matrix
    % NV: b x c x d matrix (derivative of matrix N wrt vector V)
    [Mr,Mc,V] = size(MV);
    [Nr,Nc] = size(N);
    
    R = zeros(Mr,Nc,V);
    if Mc==Nr
        for i=1:Mr
            for j=1:Nc
                for k=1:V
                    for s=1:Mc
                        R(i,j,k)=R(i,j,k)+MV(i,s,k)*N(s,j);
                    end
                end
            end
        end
    end
end