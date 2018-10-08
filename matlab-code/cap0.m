function sortie=cap0(u)
%choix du cap
i=u(1);xd=u(2);yd=u(3);x=u(4);y=u(5);theta=u(6);
a=30;
if (i==3)&((x-xd)>a)
    i=4;
end;
if (i==4)&((xd-x)>a)
    i=3;
end;
if (i==3)&((y-yd)>a)
    i=1;
end;
if (i==1)&((yd-y)>a)
    i=3;
end;
if (i==1)&((x-xd)>a)
    i=2;
end;
if (i==2)&((xd-x)>a)
    i=1;
end;
if(i==4)&((y-yd)>a)
    i=2;  
end;
if(i==2)&((yd-y)>a)
    i=4;
end;
switch i;
    case 1
        thetach=argument(theta,7*pi/4);
    case 2
        thetach=argument(theta,5*pi/4);
    case 3
        thetach=argument(theta,pi/6);
    case 4
        thetach=argument(theta,5*pi/6);
end;
sortie=[i thetach];