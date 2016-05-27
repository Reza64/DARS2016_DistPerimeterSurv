function  phi_av=ComputeAvg(i,n,phi)
    if (i == 1) 
        phi_av =(phi(2) + phi(n)- 2*pi) / 2;
        if (phi(n)>0 && phi(n)<pi && phi(i)>=3.11 && phi(i)<(2*pi) && phi(2)>pi && phi(2)<(2*pi)) || ...
           (phi(n)>pi && phi(n)<(2*pi) && phi(i)>pi && phi(i)<(2*pi) && phi(2)>pi && phi(2)<(2*pi))
            phi_av =(phi(2) + phi(n)) / 2;
        end
        if (phi(n)>pi && phi(n)<(2*pi) && phi(i)>pi && phi(i)<(2*pi) && phi(2)>=0 && phi(2)<pi)
           phi_av =(phi(2) + phi(n)+ 2*pi) / 2;
        end
    elseif (i == n)
         phi_av = (phi(1) + 2*pi + phi(n-1)) / 2;
        if (phi(1)>0 && phi(1)<pi && phi(i)>0 && phi(i)<=pi && phi(n-1)>0 && phi(n-1)<pi) || ...
            (phi(1)>pi && phi(1)<(2*pi) && phi(i)>0 && phi(i)<=pi && phi(n-1)>0 && phi(n-1)<pi) || ...
            (phi(1)>pi && phi(1)<(2*pi) && phi(i)>pi && phi(i)<=(2*pi) && phi(n-1)>0 && phi(n-1)<pi) || ...
            (phi(1)>pi && phi(1)<(2*pi) && phi(i)>pi && phi(i)<=(2*pi) && phi(n-1)>pi && phi(n-1)<(2*pi))
            phi_av =(phi(1)+ phi(n-1)) / 2;
        end               
    else % phi i
        phi_av = (phi(i+1) + phi(i-1)) / 2;
        if (phi(i+1)>0 && phi(i+1)<pi && phi(i)<(2*pi)&& phi(i)>=pi && phi(i-1)<(2*pi)&& phi(i-1)>pi) || ...
            (phi(i+1)>0 && phi(i+1)<pi && phi(i)<pi && phi(i)>0 && phi(i-1)<(2*pi)&& phi(i-1)>pi)
          phi_av=(phi(i+1) + phi(i-1)+2*pi) / 2;
        end            
    end   

    %====================
    if (phi_av<0)
         if (i==1)
            phi_av=(phi(i+1) + phi(n)) / 2;
         end
     end 
     if (phi_av>6.2832)
         if( i==n)
            phi_av=(phi(1) + phi(n-1)- 2*pi) / 2;
         elseif (i==1)
           phi_av=(phi(i+1) + phi(n)- 2*pi) / 2;
         else
           phi_av=(phi(i+1) + phi(i-1)- 2*pi) / 2;
         end
     end 