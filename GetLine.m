function out=GetLine(start,Goal) 
hDen=0;
r=0.2;
if start(1)>Goal(1)
   r=-r;
end
x=start(1):r:Goal(1);
d=size(x,2);

%second point
r=abs(start(2)-Goal(2))/(d-1);
if start(2)>Goal(2)
   r=-r;
end
y=start(2):r:Goal(2);
% y=round(y);
%line(x,y);
if abs(start(2)-Goal(2))~=0
    disp('');
else
   y=start(2);
   y=ones(1,d)*y;
end
out=[x;y];
end