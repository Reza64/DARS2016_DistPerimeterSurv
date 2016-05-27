function     [rho phi]=Convert2Polar(x,y,xc,yc,n)
for i=1:n
    rho(i) =sqrt((x(i)-xc)^2+(y(i)-yc)^2);
    tphi =atan2((y(i)-yc),(x(i)-xc));
    %if (tphi<0)
     %  tphi=tphi+2*pi;
    %end
    phi(i)=tphi;
end