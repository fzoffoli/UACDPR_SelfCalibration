function [out,counter]=simulate_SamplingDelay(t,dt,fs,A,A_prev,counter)
global flag_save
% idx= find(A_prev(1,1:int64(t/dt))-A_prev(1,int64(t/dt)),1,'last');
if t-counter>=1/fs-5e-05
    if flag_save==0
    counter=t; 
    end
     out=A;
else
%     if flag_save==0
%     counter=counter+dt;
%     end
    out=A_prev;
end
end