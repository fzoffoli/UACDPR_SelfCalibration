function [m,q]=find_segment(x1,x2,y1,y2)
m=(y2-y1)/(x2-x1);
q=(x2*y1-y2*x1)/(x2-x1);
end