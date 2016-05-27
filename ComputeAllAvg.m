function phi_av=ComputeAllAvg(n,phi)
for i=1:n
        if (i == 1) 
             phi_av(i) =(phi(2) + phi(n)- 2*pi) / 2;
        elseif (i == n)
             phi_av(i) = (phi(1) + 2*pi + phi(n-1)) / 2;
        else % phi i
             phi_av(i) = (phi(i+1) + phi(i-1)) / 2;             
        end
 end