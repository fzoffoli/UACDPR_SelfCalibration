function linein = replaceline(pointA,pointB,Colorin)

linein=line([pointA(1),pointB(1)],[pointA(2),pointB(2)],[pointA(3),pointB(3)], 'LineWidth', 2, 'Color', Colorin);
end

