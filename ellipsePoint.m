function out=ellipsePoint(x,y,a,b)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:0.07:2*pi; 
xp=a*cos(ang);
yp=b*sin(ang);
out=[x+xp;y+yp];
end