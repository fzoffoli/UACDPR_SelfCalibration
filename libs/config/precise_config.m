fig = uifigure('Position',[680   358   760   620],'Name','Robot configuration');
% pos = get(gcf,'position');
tg = uitabgroup(fig,'Position',[0 0 fig.Position(3:4)]);

t1 = uitab(tg,'Title','Robot Parameters');
% 
load_namel = uilabel(t1,'Text','Name to load','Position',[350 511 300 20]);
container.load_name = uieditfield(t1,'Text','Position',[350 490 300 20],'Value','file name');

save_namel = uilabel(t1,'Text','Name to save','Position',[20 511 300 20]);
container.save_name = uieditfield(t1,'Text','Position',[20 490 300 20],'Value','file name');

pointnumberl = uilabel(t1,'Text','Enter Cable number','Position',[20 461 300 20]);
pointnumber = uieditfield(t1,'numeric','Position',[20 440 100 20],'Value',3);

% 
platformmassl = uilabel(t1,'Text','Enter Platform Mass','Position',[20 411 150 20]);
container.platformmass = uieditfield(t1,'numeric','Position',[20 390 150 20],'Value',3);

gravityl = uilabel(t1,'Text','Set Gravity Acceleration (absolute value)','Position',[200 411 300 20]);
container.gravity = uieditfield(t1,'numeric','Position',[200 390 100 20],'Value',3);
% 
Ptogl = uilabel(t1,'Text','Enter Reference-point to Center-of-Mass vector','Position',[20 361 300 20]);
xl = uilabel(t1,'Text','x:','Position',[20 341 30 20]);
yl = uilabel(t1,'Text','y:','Position',[130 341 30 20]);
zl = uilabel(t1,'Text','z:','Position',[240 341 30 20]);

container.Ptogx = uieditfield(t1,'numeric','Position',[50 341 30 20],'Value',3);
container.Ptogy = uieditfield(t1,'numeric','Position',[160 341 30 20],'Value',3);
container.Ptogz = uieditfield(t1,'numeric','Position',[270 341 30 20],'Value',3);


Depvectl = uilabel(t1,'Text','Set Dependent variables (put 1 in actuated var, 0 in unactuated var)','Position',[20 311 500 20])

xl = uilabel(t1,'Text','x','Position',[20 291 30 20]);
yl = uilabel(t1,'Text','y','Position',[70 291 30 20]);
zl = uilabel(t1,'Text','z','Position',[120 291 30 20]);
xl = uilabel(t1,'Text','Roll','Position',[170 291 30 20]);
yl = uilabel(t1,'Text','Pitch','Position',[220 291 30 20]);
zl = uilabel(t1,'Text','Yaw','Position',[270 291 30 20]);

container.dp(1) = uieditfield(t1,'numeric','Position',[20 271 30 20]);
container.dp(2) = uieditfield(t1,'numeric','Position',[70 271 30 20]);
container.dp(3) = uieditfield(t1,'numeric','Position',[120 271 30 20]);
container.dp(4) = uieditfield(t1,'numeric','Position',[170 271 30 20]);
container.dp(5) = uieditfield(t1,'numeric','Position',[220 271 30 20]);
container.dp(6) = uieditfield(t1,'numeric','Position',[270 271 30 20]);


Inertial = uilabel(t1,'Text','Barycentric Inertia Matrix','Position',[20 231 300 20]);
xl = uilabel(t1,'Text','Ixx','Position',[20 211 300 20]);
yl = uilabel(t1,'Text','Ixy','Position',[140 211 300 20]);
zl = uilabel(t1,'Text','Ixz','Position',[260 211 300 20]);

container.Ixx = uieditfield(t1,'numeric','Position',[20 180 80 20],'Value',3);
container.Ixy = uieditfield(t1,'numeric','Position',[140 180 80 20],'Value',3);
container.Ixz = uieditfield(t1,'numeric','Position',[260 180 80 20],'Value',3);

yl = uilabel(t1,'Text','Iyy','Position',[140 151 300 20]);
zl = uilabel(t1,'Text','Iyz','Position',[260 151 300 20]);

container.Iyy = uieditfield(t1,'numeric','Position',[140 120 80 20],'Value',3);
container.Iyz = uieditfield(t1,'numeric','Position',[260 120 80 20],'Value',3);

zl = uilabel(t1,'Text','Izz','Position',[260 91 300 20]);
container.Izz = uieditfield(t1,'numeric','Position',[260 60 80 20],'Value',3);

pulley_tab=uitab(tg,'Title','Pulleys');
platform_tab=uitab(tg,'Title','Platform');
s.a=0;
% p = uibutton(t1,'Text','Create Platform','ButtonPushedFcn', @(btn,event) createplatform_ui(btn,pointnumber.Value,tg,platform_tab),'Position',[130 440 100 20]);
% b = uibutton(t1,'Text','Create Pulleys','ButtonPushedFcn', @(btn,event) createpulleys(btn,pointnumber.Value,tg,pulley_tab),'Position',[250 440 100 20]);
p = uibutton(t1,'Text','Create Platform and Pulleys','ButtonPushedFcn', @(btn,event) create_pulleysplatform(btn,pointnumber.Value,tg,platform_tab,pulley_tab),'Position',[130 440 200 20]);
add_point=uibutton(t1,'Text','+ 1','ButtonPushedFcn', @(btn,event) add_pulleysplatform(btn,tg,platform_tab,pulley_tab),'Position',[350 440 50 20]);
remove_point=uibutton(t1,'Text','- 1','ButtonPushedFcn', @(btn,event) remove_pulleysplatform(btn,tg,platform_tab,pulley_tab),'Position',[420 440 50 20]);
l = uibutton(t1,'Text','Build structure','ButtonPushedFcn', @(btn,event) buildstructure(btn,tg),'Position',[550 300 100 80]);
q = uibutton(t1,'Text','Load structure','ButtonPushedFcn', @(btn,event) loadstructure(btn,tg),'Position',[430 300 100 80]);


