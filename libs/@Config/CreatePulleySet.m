function out = CreatePulleySet(n)
tmp=1;
    for i=1:n
        prompt = {['ENTER PULLEY n.',num2str(i),' PARAMETERS:',newline,'NB: Pulley number should reflect attach point number: first pulley will be associated with first platform attach-point and so on',newline,newline,'Enter radius'],'Enter i','Enter j','Enter k','Enter Cable Enter Position'};
        dlgtitle = 'Input';
        dims = [1 50];
        definput = {num2str(tmp),'1 0 0','0 1 0','0 0 1','1 1 1'};
        answer = inputdlg(prompt,dlgtitle,dims,definput);

        Radius=str2double(answer{1});
        FixedFrame{1}=str2num(answer{2})';
        FixedFrame{2}=str2num(answer{3})';
        FixedFrame{3}=str2num(answer{4})';
        CableEnterPosition=str2num(answer{5})';
        tmp=Radius;
%         newPulley=Pulley(int8(i),Radius,CableEnterPosition,FixedFrame);
        
        
        out{i}.Id=int8(i);
        out{i}.r=Radius;
        out{i}.CableEnterPosition=CableEnterPosition;
        out{i}.FixedFrame=FixedFrame;
        
        
%         newPulleySet{i}=newPulley;
    end
end

