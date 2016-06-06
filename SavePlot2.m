function SavePlot2(n,h,myaxis,saveflag,Showflag,writerObj,iter,Verr,phierr,trajRobxy,traj_PhiRho,r0traj)
i=1;
% if Showflag==0
% %   h1=figure;
% else
    h1=h;
% end
% plot(trajRobxy(:,i),trajRobxy(:,n+i),'k');
% grid on;
% axis(myaxis);
% set(h1,'Position',[100 100  myaxis(2)*5 myaxis(4)*4])
len=50;
if saveflag
%     frame = getframe(h1,[0 0 myaxis(2)*5 myaxis(4)*4]);
%     for i=1:len
%        writeVideo(writerObj,frame);
%     end
end
size=get(h1,'Position');
h1=figure, plot(1:iter-1,Verr);
set(h1,'Position',[100 100  myaxis(2)*5 myaxis(4)*4])
title('Error of Rho');
if saveflag
%     frame = getframe(h1,[0 0 myaxis(2)*5 myaxis(4)*4]);
% %     frame = getframe(h1,[0 0 size(3) size(4)]);
%     for i=1:len
%        writeVideo(writerObj,frame);
%     end
end
h1=figure, plot(1:iter-1,phierr);
set(h1,'Position',[100 100  myaxis(2)*5 myaxis(4)*4])
title('Error of Phi');
if saveflag
%         frame = getframe(h1,[0 0 myaxis(2)*5 myaxis(4)*4]);
%     for i=1:len
%        writeVideo(writerObj,frame);
%     end
end
h1=figure,polar(traj_PhiRho(1:iter-1,1)',r0traj(1:n:end));
set(h1,'Position',[0 0  size(3) size(4)])
if saveflag
    set(h1,'Position',[100 100  myaxis(2)*5 myaxis(4)*4]);
%     for i=1:len
%        writeVideo(writerObj,frame);
%     end
    close(writerObj);   
end
