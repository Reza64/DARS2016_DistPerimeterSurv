close all
clear all
clc;
%% initialization 
n =3;
a=350;b=200;
A=1000; %the area of ellipse
% rho = 2.3218;
% phi=-0.3;
% rho = 30+ones(n,1)*20;
% temp=pi * rand(n,1);
% phi = sort(temp);
x =[60;50;40];% rho.* cos(phi);
y = [40;60;59];% rho.* sin(phi);
[rho phi] = Convert2Polar(x,y,30,39,n); 
rho=rho';phi=phi';
% defining time 
dt=0.006;
t=0:dt:8*pi;
% parameters ===================
kp = 5.05;
kphi=0.75;
Omeg = 0.84; % constant 
myaxis=[0 150 0 150];
trajxy=[];
trajAbsxy=[];
traj_PhiRho=[];
trajRobxy=[];
r0traj=[];
global mycolor;
 mycolor=readcolor(); 
 Showflag=1;
 saveflag=0;
 h=0;
 if Showflag
      h=figure;
      axis(myaxis);
%       set(h,'Position',[10 10  myaxis(2) myaxis(4)])
      DrawRobot2(x,y); %draw robot
 end
writerObj = VideoWriter('MultiRobStarShape2.avi');
if saveflag
     writerObj = VideoWriter('MultiRobStarShape2.avi');
    writerObj.FrameRate = 30;  % Default 30
    writerObj.Quality = 100;    % Default 75
    open(writerObj);
end  
%  syms a b ph
%  f=a*b/(sqrt(b^2*cos(ph)^2+a^2*sin(ph)^2)); 
%  diff(f,a)
%  diff(f,b)
% # Main loop #####################
%%
load traj_data.mat;
Tlen=size(trajectory_data,1);
trajectory_data; %  x,y, and width, the  constant area is 1000
iter=1;
t=0:0.01:0.1;
for k=1:Tlen
    % tu_i=a_i.t^2/2+b_i.t+c_1
    % dot{tu_i}=a_i.t+b_i
    X=trajectory_data{k,1}; 
    abc=trajectory_data{k,3};
    for ti=t    
        r=180;
        x0 =abc(1,1)*ti^2/2+abc(1,2)*ti+abc(1,3); % r*cos(ti);
        x0_dot = abc(1,1)*ti+abc(1,2);%-r*sin(ti);
        y0 = abc(2,1)*ti^2/2+abc(2,2)*ti+abc(2,3);% r*sin(ti);
        y0_dot = abc(2,1)*ti+abc(2,2);
        a= abc(3,1)*ti^2/2+abc(3,2)*ti+abc(3,3); % need to correct the differenciate
        a_dot= abc(3,1)*ti+abc(3,2);
        b=A/(pi*a);
        for i=1:n
            ph=phi(i);
            r0(i) = a*b/(sqrt(b^2*cos(ph)^2+a^2*sin(ph)^2)); 
        end
        r0traj=[r0traj r0];   
        %=============================================
        phi_av = ComputeAllAvg(n,phi);
        dot_phi = Omeg + kphi .* (phi_av - phi');
        for i=1:n
             ph=phi(i);
             r0_dot(i)=-(a*b*(a^2-b^2)*sin(ph)*cos(ph)*dot_phi(i))/(((b*cos(ph))^2+(a*sin(ph))^2)^(3/2))+...
                (b/(a^2*sin(ph)^2 + b^2*cos(ph)^2)^(1/2) - (a^2*b*sin(ph)^2)/(a^2*sin(ph)^2 + b^2*cos(ph)^2)^(3/2))*a_dot;
        end
        dot_rho = r0_dot + kp * (r0 - rho');
        % Robot Dynamics
        for i=1:n
            dot_x(i) = dot_rho(i) * cos(phi(i)) - rho(i) * dot_phi(i) * sin(phi(i)) + x0_dot;
            dot_y(i) = dot_rho(i) * sin(phi(i)) + rho(i) * dot_phi(i) * cos(phi(i)) + y0_dot;
        end
        % Integrtion=============
        x = x + dot_x'.* dt; 
        y = y + dot_y'.* dt;    
        % =================================
        trajAbsxy = [trajAbsxy;[x0,y0]];
        trajRobxy=[trajRobxy;[x',y']];
     if Showflag
         figure(h);
        axis(myaxis);   
%         set(h,'Position',[10 10  myaxis(2) myaxis(4)])
        for i=1:1
            hold on;
            plot(trajRobxy(:,i),trajRobxy(:,n+i),'k');
        end
        hold on;
        DrawEllipse(x0,y0,a,b);
        DrawRobot2(x,y); %draw robot
        hold on;
        plot(trajAbsxy(:,1),trajAbsxy(:,2),'-.r');    
        drawnow;
        pause(0.02);
        if saveflag
            frame = getframe(h,[0 0 myaxis(2) myaxis(4)]);
            writeVideo(writerObj,frame);
        end         
     end
        % convert to polar coordinates
        oldphi=phi;
        [rho phi] = Convert2Polar(x,y,x0,y0,n); 
        phi = correctphi(oldphi,phi,n)';
        rho=rho';
        %--------------------------
        Verr(iter,:) = 1/2 .* (rho' - r0).^2;
        trajxy = [trajxy; [x' y']];
        traj_PhiRho = [traj_PhiRho; [phi' rho']];    
        for i=1:n-1
            phierr(iter,i)=abs(phi(i)-phi(i+1));
        end
    %     phierr(iter,i+1)=abs(phi(1)-phi(n));
        iter = iter+1;
         if Showflag
              if (ti~=t(end))
                 clf('reset');
              end    
         end
    end
end %  Iter
%% ====================================
if Showflag
hold on;
DrawEllipse(x0,y0,a,b);
DrawRobot2(x,y); %draw robot
end
SavePlot(n,h,myaxis,saveflag,Showflag,writerObj,iter,Verr,phierr,trajRobxy,traj_PhiRho,r0traj);
