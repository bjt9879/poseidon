function arg=argument(theta,thetai)

%calcule de thetai qui est la plus proche � theta
arg=thetai+2*pi*round((theta-thetai)/(2*pi)); 
 
