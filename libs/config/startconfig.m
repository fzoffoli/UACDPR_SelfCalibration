%%% this simple program allows with a UI to build a structured to be used by UACDPR constructor to instantiate the object
clear all
close all

alfa=Config;

s=struct;

s=alfa.CreateEndEffector(s);

s=alfa.CreateTrasmissionSet(s);

s=alfa.CreateDepVect(s);

% s=alfa.CreateKinVar(s);

s=alfa.CreateWrench(s);

save 'data\blabla' s
