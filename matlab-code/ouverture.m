%choix de l'ouverture
function deltavc=ouverture(u)
theta=u(1);
if (theta>pi/6)&(theta<pi/2)
    deltavc=pi/2;
end;
if (theta>pi/2)&(theta<5*pi/6)
    deltavc=pi/2;
end;
deltavc=pi*floor(theta/(2*pi)+1/4)+pi/4-theta/2;
end