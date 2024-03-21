function createplatform_ui(btn,number,tg,PointTab)

delete(get(PointTab, 'Children'))
for i=1:number
    panel(i)=uipanel('Parent',PointTab,'Title',['Attach Point n. ',num2str(i)],'Position',[PointTab.Position(1)+(i-1-3*floor((i-.1)/3))/3*PointTab.Position(3) PointTab.Position(2)+(3-ceil(i/3))/3*PointTab.Position(4) PointTab.Position(3)/3 PointTab.Position(4)/3]);
 
    dx=(panel(i).Position(3)-10)/6;
    dy=(panel(i).Position(4)-10)/5;
    
    pointlab(i).text = uilabel(panel(i),'Text','Enter attach point coordinates w.r.t. P','Position',[5 5+4*dy 200 20]);        

    
    pointlab(i).P(1)=uilabel(panel(i),'Text','X:','Position',[5 5+3*dy 40 20]);
    pointlab(i).P(2)=uilabel(panel(i),'Text','Y:','Position',[15+2*dx 5+3*dy 40 20]);
    pointlab(i).P(3)=uilabel(panel(i),'Text','Z:','Position',[15+2*2*dx 5+3*dy 40 20]);
    
    
    point(i).P(1) = uieditfield(panel(i),'numeric','Position',[5+dx 5+3*dy 40 20],'Value',3);
    point(i).P(2) = uieditfield(panel(i),'numeric','Position',[5+3*dx 5+3*dy 40 20],'Value',3);
    point(i).P(3) = uieditfield(panel(i),'numeric','Position',[5+5*dx 5+3*dy 40 20],'Value',3);
%     
%     pointlab(i).i(1)=uilabel(panel(i),'Text','ix:','Position',[5 5+2*dy 40 20]);
%     pointlab(i).i(2)=uilabel(panel(i),'Text','iy:','Position',[15+2*dx 5+2*dy 40 20]);
%     pointlab(i).i(3)=uilabel(panel(i),'Text','iz:','Position',[15+2*2*dx 5+2*dy 40 20]);
%     
%     point(i).i(1) = uieditfield(panel(i),'numeric','Position',[5+dx 5+3*dy 40 20],'Value',3);
%     point(i).i(2) = uieditfield(panel(i),'numeric','Position',[5+3*dx 5+3*dy 40 20],'Value',3);
%     point(i).i(3) = uieditfield(panel(i),'numeric','Position',[5+5*dx 5+3*dy 40 20],'Value',3);
%     
% %     Ixx = uieditfield(t1,'numeric','Position',[20 180 80 20],'Value',3);
% %     Ixy = uieditfield(t1,'numeric','Position',[140 180 80 20],'Value',3);
% %     Ixz = uieditfield(t1,'numeric','Position',[260 180 80 20],'Value',3);
%    
%     pointlab(i).j(1)=uilabel(panel(i),'Text','jx:','Position',[5 5+dy 40 20]);
%     pointlab(i).j(2)=uilabel(panel(i),'Text','jy:','Position',[15+2*dx 5+dy 40 20]);
%     pointlab(i).j(3)=uilabel(panel(i),'Text','jz:','Position',[15+2*2*dx 5+dy 40 20]);
%     
%     point(i).j(1) = uieditfield(panel(i),'numeric','Position',[5+dx 5+3*dy 40 20],'Value',3);
%     point(i).j(2) = uieditfield(panel(i),'numeric','Position',[5+3*dx 5+3*dy 40 20],'Value',3);
%     point(i).j(3) = uieditfield(panel(i),'numeric','Position',[5+5*dx 5+3*dy 40 20],'Value',3);
%     
%     pointlab(i).k(1)=uilabel(panel(i),'Text','kx:','Position',[5 5 40 20]);
%     pointlab(i).k(2)=uilabel(panel(i),'Text','ky:','Position',[15+2*dx 5 40 20]);
%     pointlab(i).k(3)=uilabel(panel(i),'Text','kz:','Position',[15+2*2*dx 5 40 20]);
%     
%     point(i).k(1) = uieditfield(panel(i),'numeric','Position',[5+dx 5+3*dy 40 20],'Value',3);
%     point(i).k(2) = uieditfield(panel(i),'numeric','Position',[5+3*dx 5+3*dy 40 20],'Value',3);
%     point(i).k(3) = uieditfield(panel(i),'numeric','Position',[5+5*dx 5+3*dy 40 20],'Value',3);
%     

end
end