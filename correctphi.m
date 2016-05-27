function   phi=correctphi(oldphi,phi,n)
for i=1:n
    deltaphi=abs(phi(i)-oldphi(i));
    c=ceil(deltaphi/(2*pi));
    if deltaphi>pi
        phi(i)=(2*pi*c)+phi(i);
    end
end
