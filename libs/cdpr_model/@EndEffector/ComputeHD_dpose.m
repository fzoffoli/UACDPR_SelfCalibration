function obj = ComputeHD_dpose(obj)
%H_DPOSECOMP H matrix derivative wrt pose
    OrientType = obj.OrientType;
    s1=sin(obj.Pose(4));
    s2=sin(obj.Pose(5));
    c1=cos(obj.Pose(4));
    c2=cos(obj.Pose(5));

    H_dpose = zeros(3,3,6);   
    D_dpose = zeros(6,6,6);

    if OrientType=="Edo"
        H_dpose(:,:,4)= [0  -c1   -s1*c2;
                         0  -s1    c1*c2;
                         0   0     0];     
        H_dpose(:,:,5)= [0 0 -c1*s2;
                         0 0 -s1*s2;
                         0 0 -c2];    
    elseif OrientType=="TaitBryan"
        H_dpose(:,:,4)= [0   0    0;
                         0  -s1  -c1*c2;
                         0   c1  -s1*c2]; 
        H_dpose(:,:,5)= [0   0   c2;
                         0   0   s1*s2;
                         0   0  -c1*s2]; 
    else
        H_dpose(:,:,4)= [0  -c1   -s1*c2;
                         0  -s1    c1*c2;
                         0   0     0];
        H_dpose(:,:,5)= [0 0 -c1*s2;
                         0 0 -s1*s2;
                         0 0 -c2];
    end
    D_dpose(4:6,4:6,:) = H_dpose;

    obj.H_dpose = H_dpose;
    obj.D_dpose = D_dpose;
end