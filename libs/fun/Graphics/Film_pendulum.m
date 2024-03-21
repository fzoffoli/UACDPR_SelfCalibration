function F=Film_pendulum(pointO,pointP,pointG)

fig=figure
 hold on
 grid on
 axis('equal')
    fig.WindowState = 'maximized';
    
%     view(-73.681625749277671,13.632611464968154)
view([0 0])
xlim([pointO(1)-3,pointO(1)+3])
ylim([pointO(2)-3,pointO(2)+3])
zlim([-5,pointO(3)+5])


    cable=animatedline('MaximumNumPoints',2,'LineWidth',2,'Color','k');
    
    for j=1:length(pointO)
    pointO_film=plot3(pointO(1,j), pointO(2,j), pointO(3,j), 'ro', 'MarkerSize', 10,'Color','k');
    pointP_film=plot3(pointP(1,j), pointP(2,j), pointP(3,j), 'ro', 'MarkerSize', 10,'Color','b');
    pointG_film=plot3(pointG(1,j), pointG(2,j), pointG(3,j), 'ro', 'MarkerSize', 10,'Color','g');
    cable=  drawline_animated(pointO(1:3,j),pointP(1:3,j),cable);
    
     F(j)=getframe(gcf);
     
     drawnow
     
     delete(pointO_film)
     delete(pointP_film)
     delete(pointG_film)
    end
    
end