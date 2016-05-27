close all
clear all
clc;
%% initialization 
n = 1;
a=350;b=200;
rho = 2.3218;
phi=-0.3;
x = rho.* cos(phi);
y = rho.* sin(phi);
% defining time 
dt=0.05;
t=0:dt:5*pi;
% parameters ===================
kp = 10.0;
Omeg = 0.5; % constant 
myaxis=[-400 400 -400 400];
axis(myaxis);
trajxy=[];
trajAbsxy=[x,y];
traj_PhiRho=[];
trajRobxy=[];
r0traj=[];
global mycolor;
% # Main loop #####################
%%
 iter=1;
for ti=t    
    x0 = 0;
    x0_dot = 0;
    y0 = 0;
    y0_dot = 0;   
    % convert to polar coordinates
    % [rho phi] = Convert2Polar(x,y,x0,y0,n); 
    r0 = a*b/(sqrt(b^2*cos(phi)^2+a^2*sin(phi)^2)); 
    r0traj=[r0traj r0];   
    %=============================================
    dot_phi = Omeg;
%     phi = phi + dot_phi*dt;
    r0_dot=-(a*b*(a^2-b^2)*sin(phi)*cos(phi)*dot_phi)/(((b*cos(phi))^2+(a*sin(phi))^2)^(3/2));
%     -(a*b*(-2*cos(phi)*sin(phi)*b*dot_phi + 2*cos(phi)*sin(phi)*a*dot_phi))/(2*(a^2*sin(phi)^2 + b^2*cos(phi)^2)^(3/2));
    dot_rho = r0_dot + kp * (r0 - rho);
%     rho = rho + dot_rho*dt;
    % Robot Dynamics
    dot_x = dot_rho * cos(phi) - rho * dot_phi * sin(phi) + x0_dot;
    dot_y = dot_rho * sin(phi) + rho * dot_phi * cos(phi) + y0_dot;
    % Integrtion=============
    x = x + dot_x * dt; 
    y = y + dot_y * dt;    
    % =================================
    trajAbsxy = [trajAbsxy;[x0,y0]];
    trajRobxy=[trajRobxy;[x',y']];
    [rho phi] = Convert2Polar(x,y,x0,y0,n); 
    %--------------------------
    Verr(iter) = 1/2 * (rho - r0)^2;
    trajxy = [trajxy; [x,y]];
    traj_PhiRho = [traj_PhiRho; [phi rho]];
    iter = iter+1;
end
%% ====================================
hold on;
DrawEllipse(x0,y0,a,b);
i=1;
plot(trajRobxy(:,i),trajRobxy(:,n+i),'k');
grid on;
axis(myaxis);
figure, plot(1:iter-1,Verr);
figure,polar(traj_PhiRho(1:iter-1,1)',r0traj);