function arg=argument(theta,thetai)

%calcule de thetai qui est la plus proche à theta
arg=thetai+2*pi*round((theta-thetai)/(2*pi)); 
 
