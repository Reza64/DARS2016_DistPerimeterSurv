function     DrawEllipse(xc,yc,a,b)
    p=ellipsePoint(xc,yc,a,b);
%     color1=rand(1,3);
    hold on;
    plot(p(1,:),p(2,:),'color','b','LineWidth',2);
    plot(xc,yc,'+');
    drawnow;
end