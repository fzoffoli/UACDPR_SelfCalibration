function s=CreateTrasmissionSet(s)

prompt={['Enter Pulleys number']};
dlgtitle = 'Input';
dims = [1 50];
definput = {'0'};
answer = inputdlg(prompt,dlgtitle,dims,definput);

if str2num(answer{1})==0
    error('Insert a valid number of pulleys')
else
n=str2num(answer{1});

    for i=1:n
        s.Trasmission.Cable(i)=i;
%         newPulleySet(i)=Pulley(i);
        s.Trasmission.Winch(i)=i;
    end
    
%     Pulleytmp=Config.CreatePulleySet(n);
    s.Trasmission.Pulley=Config.CreatePulleySet(n); 
end
end

