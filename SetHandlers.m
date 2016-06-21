function    handler=SetHandlers(n,x,y)
color = hsv(n);
   for i =1: n
        handler(i) = plot(nan, '.');
        set(handler(i),'Color',color(i,:), 'MarkerSize', 30);
        set(handler(i),'XData',x(i),'YData',y(i));
    end
    % ellipse handler
    handler(n+1) = plot(nan);
    set(handler(n+1),'Color','b','LineWidth',2);
    set(handler(n+1),'XData',[],'YData',[]);    
    % trajectory handler
    handler(n+2) = plot(nan);
    set(handler(n+2),'Color','k','LineWidth',1);
    set(handler(n+2),'XData',[],'YData',[]);    
    % shape center handler
    handler(n+3) = plot(nan,'-.r');
    set(handler(n+3),'XData',[],'YData',[]);    
    handler(n+4) = plot(nan,'--rs');
    set(handler(n+4),'Color','k','MarkerFaceColor','k','MarkerSize',10);
    set(handler(n+4),'XData',[],'YData',[]); 
    %lines for each robot
    for i=1:n
        handler(n+4+i) = plot(nan);
        set(handler(n+4+i),'Color',color(i,:),'LineWidth',1);
        set(handler(n+4+i),'XData',[],'YData',[]);     
    end