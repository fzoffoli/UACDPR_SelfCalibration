% metti lista delle pose assunte dal cdpr e un'istanza del cdpr. 

% Il video mostra cosa succede al CDPR, il pallino verde è il baricentro,
% quello viola il punto P. Vengono disegnati anche i sistemi di riferimento
% delle pulegge

% Le linee sono disegnate come linee animate (spezzate) di massimo due punti (i.e.
% sono tratti di retta) a cui a ogni istante vengono aggiunti altri due
% punti (i.e. vengono sostituiti i due punti precedenti visto che raggiunto il numero massimo di punti questi vengono sovrascritti)

% Se si vuole modificare il video togliendo/aggiungendo elementi
% creare nuove linee al di fuori del ciclo for j= e aggiungerci punti
% all'interno del ciclo

% la funzione drawline_animated(pointA,pointB,line) aggiunge 
%  i punti "pointA" e "pointB" alla linea  "line" (creata con il comando animatedline di matlab)

% Decommentare la linea FF(j)=getframe(gcf); e commentare  per avere in
% output dalla funzione i frame

% Decommentare "ATTACH POINTS" e ripristinare la cancellazione dei punti per creare pallini sui punti di attacco dei
% cavi sulla piattaforma
function [FF] = FilmDrawRobot(PoseList,obj)
platform_width = 0.1;
FF=1;
fig=figure
hold on
axis('equal')
fig.WindowState = 'maximized';

for i=1:obj.CablesNumber
    D(:,i) = obj.Trasmission.Pulley{i}.CableEnterPosition;
end
zoom = 0.3;
x_min = min(D(1,:)); x_max = max(D(1,:));   dx = (x_max-x_min)*zoom;
y_min = min(D(2,:)); y_max = max(D(2,:));   dy = (y_max-y_min)*zoom;
z_min = min(PoseList(3,:)); z_max = max(D(3,:));   dz = (z_max-z_min)*zoom;
xlim([(x_min-dx) (x_max+dx)]);
ylim([(y_min-dy) (y_max+dy)]);
zlim([(z_min-dz) (z_max+dz)]);

grid on
%view(-1.555548826490898e+02,55.669688102783475)
% view(-2.004338320341311e+02,16.416591251121574);
% view(69.795459156773745,-19.871790314297129);
% view(18.400342335120939,29.194425901919132);
% view(31.595946501202949,30.684713304165026);
view(31.595946501202949,30.684713304165026);
%%%%Creazione degli elementi base per il disegno
for i=1:obj.CablesNumber
    a{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','k');
    e{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','b');
    q{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','b');
    p{i}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','b');
    pulley{i,1}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','c');
    pulley{i,2}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','g');
    pulley{i,3}=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','y');
end

%%%ORIGIN
line([0,.5],[0,0],[0,0], 'LineWidth', 1, 'Color', 'c');
line([0,0],[0,.5],[0,0], 'LineWidth', 1, 'Color', 'g');
line([0,0],[0,0],[0,.5], 'LineWidth', 1, 'Color', 'y');
plot3(0, 0, 0, 'ro', 'MarkerSize', 2);



for j=1:size(PoseList,2)
    obj=obj.SetPoseAndUpdate0KIN(PoseList(:,j));
    
    
    %%%%%%%%%%%PULLEY FRAMES
    for i=1:obj.CablesNumber

        vect1=[.5 0 0];
        vect2=[0 .5 0];
        vect3=[0 0 .5];
        
        Rotmatrix=[obj.Trasmission.Pulley{i}.PulleyFrame{1} obj.Trasmission.Pulley{i}.PulleyFrame{2} obj.Trasmission.Pulley{i}.PulleyFrame{3}].';
        vectpulley1=(vect1*Rotmatrix).'+obj.Trasmission.Pulley{i}.CableEnterPosition;
        vectpulley2=(vect2*Rotmatrix).'+obj.Trasmission.Pulley{i}.CableEnterPosition;
        vectpulley3=(vect3*Rotmatrix).'+obj.Trasmission.Pulley{i}.CableEnterPosition;
        
        pulley{i,1}=drawline_animated(obj.Trasmission.Pulley{i}.CableEnterPosition,vectpulley1,pulley{i,1});
        pulley{i,2}=drawline_animated(obj.Trasmission.Pulley{i}.CableEnterPosition,vectpulley2,pulley{i,2});
        pulley{i,3}=drawline_animated(obj.Trasmission.Pulley{i}.CableEnterPosition,vectpulley3,pulley{i,3});
        
    end
    
    
    %%%%%%%%%%%%%%PLATFORM
    for i=1:obj.CablesNumber
            WidthPoint{i}=obj.EndEffector.GlobalAttachPoints{i}-obj.EndEffector.RotMatrix*[0;0;platform_width]; %%creo un'immagine dei punti di attacco dei cavi 0.3 metri al di sotto (nel sis di rif locale della piattaforma) di questi
    end
    
    %%%Disegno la piattaforma
    for i=1:obj.CablesNumber
        if i==obj.CablesNumber           
            e{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},obj.EndEffector.GlobalAttachPoints{1},e{i}); 
            q{i}=drawline_animated(WidthPoint{i},WidthPoint{1},q{i});          
        else                            
            e{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},obj.EndEffector.GlobalAttachPoints{i+1},e{i});
            q{i}=drawline_animated(WidthPoint{i},WidthPoint{i+1},q{i});            
        end
        p{i}=drawline_animated(obj.EndEffector.GlobalAttachPoints{i},WidthPoint{i},p{i});
    end
    
    %%%%%%%%%CABLES
    for i=1:obj.CablesNumber
        a{i}=drawline_animated(obj.Trasmission.Pulley{i}.CableExitPosition,obj.EndEffector.GlobalAttachPoints{i},a{i});
    end
    %%%%%BARYCENTER AND POINT P
    ptg=obj.RotMatrix_*obj.EndEffector.Platform_Param.PtoG;
    pointP=plot3(obj.EndEffector.Pose(1), obj.EndEffector.Pose(2), obj.EndEffector.Pose(3), 'ro', 'MarkerSize', 5,'Color','r');
    pointG=plot3(obj.EndEffector.Pose(1)+ptg(1), obj.EndEffector.Pose(2)+ptg(2), obj.EndEffector.Pose(3)+ptg(3), 'ro', 'MarkerSize', 5,'Color','g');
    
    
%     %%%%%ATTACH POINTS
%     for i=1:obj.CablesNumber
%         pointA{i}=plot3(obj.EndEffector.GlobalAttachPoints{i}(1), obj.EndEffector.GlobalAttachPoints{i}(2), obj.EndEffector.GlobalAttachPoints{i}(3), 'ro', 'MarkerSize', 5,'Color','m');
%     end
    
    
    
    % FF(j)=getframe(gcf) ;
    
    drawnow
    
    %cancellazione dei pallini creati in precedenza
%     for i=1:obj.CablesNumber
%     delete(pointA{i});
%     end
    delete(pointG);
    %delete(pointP);
   
%     pause(0.1);
end
end

