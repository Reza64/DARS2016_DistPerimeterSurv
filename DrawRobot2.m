function   DrawRobot2(x,y)
global mycolor;
n=size(x,1);
for i=1:n
    hold on;
    plot(x(i),y(i),'--rs','MarkerEdgeColor','k',...
                'MarkerFaceColor',mycolor(i,:),...
                'MarkerSize',10); 
end