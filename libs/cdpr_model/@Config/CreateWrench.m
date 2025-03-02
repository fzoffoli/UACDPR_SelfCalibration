function s = CreateWrench(s)
 prompt = {['NB: Fixed refers to forces which are constant with respect to the fixed frame. Local refers to forces which are constant with respect to plaform reference frame',newline,'Number of fixed forces'],'Number of local forces','Number of fixed moments','Set Gravity acceleration (absolute value)'};
 
    definput = {'0','0','0','0'};
    dlgtitle='Forces and Moments';
    dims=[1 35];
    opts.Interpreter='tex';

    answer = inputdlg(prompt,dlgtitle,dims,definput,opts);

    answF=[0;0;0];
    answF2=[0;0;0];
    answM=[0;0;0];
    answFP=[0;0;0];
    answFP2=[0;0;0];
    
    for i=1:str2double(answer{1})
        prompt = {['Enter Fixed force n.',num2str(i)],'Enter Application point w.r.t. reference point'};
        definput={'0 0 0','0 0 0'};
        dlgtitle='Setting Fixed forces';
        dims=[1 35];
        temp=inputdlg(prompt,dlgtitle,dims,definput);

        answF(:,i)=str2num(temp{1});
        answFP(:,i)=str2num(temp{2});
    end
    %%%%%%%%%%TO BE implemented
    for i=1:str2double(answer{2})
        prompt = {['Enter Local force n.',num2str(i)],'Enter Application point w.r.t. reference point'};
        definput={'0 0 0','0 0 0'};
        dlgtitle='Setting Local forces';
        dims=[1 35];
        temp=inputdlg(prompt,dlgtitle,dims,definput);

        answF2(:,i)=str2num(temp{1});
        answFP2(:,i)=str2num(temp{2});
    end

    for i=1:str2double(answer{3})
        prompt = {['Enter Fixed Moment n.',num2str(i)]};
        definput={'0 0 0'};
        dlgtitle='Setting Fixed Moments';
        dims=[1 35];
        temp=inputdlg(prompt,dlgtitle,dims,definput);

        answM(:,i)=str2num(temp{1});
    end
        
        Fg=[0;0;-s.EndEffector.Platform.m*str2double(answer{4})];
        
        s.GlobalForces.Forces=[Fg,answF];
        s.GlobalForces.Points=[s.EndEffector.Platform.PtoG,answFP];
        
        s.LocalForces.Forces=answF2;
        s.LocalForces.Points=answFP2;
        
        s.GlobalMoments=sum(answM,2);
end

