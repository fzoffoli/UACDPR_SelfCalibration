%Generation of Permutation Matrix, given a vector that states controlled
%coordinates

function  UACDPR = GeneratePermutMatrix(UACDPR)
%INPUT: DoF-Dimensional vector of 0s and 1s
%OUTPUT: Permutation Matrix that allows for 1s to be grouped in the first
%elements of the vector and move 0s at the end.

%EXAMPLE: considering the vector temp=[x;y;z;th;psi;phi], having [y,z,psi] as
%the dependent variables---> the input vector will be [0;1;1;0;1;0]---->
%the permutation matrix multiplied by the vector "temp"  will yeld [y;z;psi;x;th;phi]
    vect=UACDPR.DependencyVect;

    P=zeros(length(vect));
    
    temp=find(~vect);
    temp2=find(vect);
    
    for i=1:length(temp2)
        P(i,temp2(i))=1;
    end
    
    for i=1:length(temp)
        P(length(temp2)+i,temp(i))=1;
    end
    UACDPR.PermutMatrix=P;

end

