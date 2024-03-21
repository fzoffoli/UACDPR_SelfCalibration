function out = CreatePlatform()

    prompt = {['PLATFORM PARAMETERS:',newline,newline,'Enter Attach-point number'],'Enter Platform Mass','Enter Reference-point to Center-of-Mass vector',['Enter Barycentric Inertia Matrix w.r.t. reference point',newline,'(e.g.: a b c; d e f; g h i)']};
    definput = {'6','10','0 0 0','1 0 0;0 1 0;0 0 1'};
    dlgtitle='Creating Platform';
    dims=[1 35];
    answer = inputdlg(prompt,dlgtitle,dims,definput);
    
    for i=1:str2double(answer{1})
        
         prompt = {['Enter Attach-point n.',num2str(i),' coordinates w.r.t. reference point']};
         definput={'0 0 0'};
         dlgtitle='Setting attach point on platform';
         dims=[1 35];
         temp=inputdlg(prompt,dlgtitle,dims,definput);

        answ(:,i)=str2num(temp{1});
        points{i}=answ(:,i)';

    end
    
    Inertia=str2num(answer{4});
    
    out.m=str2double(answer{2});
    out.PtoG=str2num(answer{3})';
    out.points=points;
    out.InertialMatrix=Inertia;
   

end

