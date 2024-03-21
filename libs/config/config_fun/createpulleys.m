function createpulleys(btn,number,tg,PulleyTab)

delete(get(PulleyTab, 'Children'))
for i=1:number
    panel(i)=uipanel('Parent',PulleyTab,'Title',['Pulley n. ',num2str(i)],'Position',[PulleyTab.Position(1)+(i-1-3*floor((i-.1)/3))/3*PulleyTab.Position(3) PulleyTab.Position(2)+(3-ceil(i/3))/3*PulleyTab.Position(4) PulleyTab.Position(3)/3 PulleyTab.Position(4)/3]);
            
    dx=(panel(i).Position(3)-10)/6;
    dy=(panel(i).Position(4)-10)/5;
    
    pulleylab(i).r = uilabel(panel(i),'Text','Radius:','Position',[5 5+4*dy 100 20]);
    pulley(i).r = uieditfield(panel(i),'numeric','Position',[5+3*dx 5+4*dy 100 20],'Value',3);

    
    pulleylab(i).D(1)=uilabel(panel(i),'Text','Dx:','Position',[5 5+3*dy 40 20]);
    pulleylab(i).D(2)=uilabel(panel(i),'Text','Dy:','Position',[15+2*dx 5+3*dy 40 20]);
    pulleylab(i).D(3)=uilabel(panel(i),'Text','Dz:','Position',[15+2*2*dx 5+3*dy 40 20]);
    
    
    pulley(i).D(1) = uieditfield(panel(i),'numeric','Position',[5+dx 5+3*dy 40 20],'Value',3);
    pulley(i).D(2) = uieditfield(panel(i),'numeric','Position',[5+3*dx 5+3*dy 40 20],'Value',3);
    pulley(i).D(3) = uieditfield(panel(i),'numeric','Position',[5+5*dx 5+3*dy 40 20],'Value',3);
    
    pulleylab(i).i(1)=uilabel(panel(i),'Text','ix:','Position',[5 5+2*dy 40 20]);
    pulleylab(i).i(2)=uilabel(panel(i),'Text','iy:','Position',[15+2*dx 5+2*dy 40 20]);
    pulleylab(i).i(3)=uilabel(panel(i),'Text','iz:','Position',[15+2*2*dx 5+2*dy 40 20]);
    
    pulley(i).i(1) = uieditfield(panel(i),'numeric','Position',[5+dx 5+2*dy 40 20],'Value',3);
    pulley(i).i(2) = uieditfield(panel(i),'numeric','Position',[5+3*dx 5+2*dy 40 20],'Value',3);
    pulley(i).i(3) = uieditfield(panel(i),'numeric','Position',[5+5*dx 5+2*dy 40 20],'Value',3);
    
   
    pulleylab(i).j(1)=uilabel(panel(i),'Text','jx:','Position',[5 5+dy 40 20]);
    pulleylab(i).j(2)=uilabel(panel(i),'Text','jy:','Position',[15+2*dx 5+dy 40 20]);
    pulleylab(i).j(3)=uilabel(panel(i),'Text','jz:','Position',[15+2*2*dx 5+dy 40 20]);
    
    pulley(i).j(1) = uieditfield(panel(i),'numeric','Position',[5+dx 5+dy 40 20],'Value',3);
    pulley(i).j(2) = uieditfield(panel(i),'numeric','Position',[5+3*dx 5+dy 40 20],'Value',3);
    pulley(i).j(3) = uieditfield(panel(i),'numeric','Position',[5+5*dx 5+dy 40 20],'Value',3);
    
    pulleylab(i).k(1)=uilabel(panel(i),'Text','kx:','Position',[5 5 40 20]);
    pulleylab(i).k(2)=uilabel(panel(i),'Text','ky:','Position',[15+2*dx 5 40 20]);
    pulleylab(i).k(3)=uilabel(panel(i),'Text','kz:','Position',[15+2*2*dx 5 40 20]);
    
    pulley(i).k(1) = uieditfield(panel(i),'numeric','Position',[5+dx 5 40 20],'Value',3);
    pulley(i).k(2) = uieditfield(panel(i),'numeric','Position',[5+3*dx 5 40 20],'Value',3);
    pulley(i).k(3) = uieditfield(panel(i),'numeric','Position',[5+5*dx 5 40 20],'Value',3);
    

end
end