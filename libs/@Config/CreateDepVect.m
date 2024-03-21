function s = CreateDepVect(s)
 prompt = {['Define motion-variables dependency vector:',newline,newline,...
     'Considering the vector [x,y,z,roll,pitch,yaw], put 1 in the position corresponding to a dependent variable,', ...
     '0 otherwise (e.g. if y, pitch and yaw are dependent, insert 0 1 0 0 1 1)']};
 
    definput = {'1 1 1 1 1 1'};
    dlgtitle='Dependency';
    dims=[1 60];
    opts.Interpreter='tex';

    answer = inputdlg(prompt,dlgtitle,dims,definput,opts);
    
    answ=str2num(answer{1});
    
    
    

    if ~any (answ~=1 & answ~=0)
            if nnz(answ) ~= length(s.Trasmission.Cable) 
                error('Number of dependent variables different from number of cables, please retry')
            else
                s.DependencyVect=answ;
            end
    else 

            error('Invalid Entry, please retry');
    end
end

