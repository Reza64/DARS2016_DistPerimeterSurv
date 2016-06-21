function    Lines=ComputeLines(n,x0,y0,x,y)

for i=1:n
    Lines(i).l=GetLine([x0,y0],[x(i),y(i)]);
end

