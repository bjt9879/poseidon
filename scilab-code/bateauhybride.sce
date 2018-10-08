function my_eventhandler(win,x,y,ibut)   //(handler = baggagiste)
       global xd yd V sortie ech;
      [xf,yf]=xchange(x,y,'i2f')
       xinfo(' Réglage Vent : v,V; Réglage Zoom : z,Z; Fin : F; ' + 'x='+string(xf)+', y='+string(yf)+', ibut='+string(ibut))
       //xd=xf;yd=yf;
       if ascii(ibut)=='w' then xd=xd-3; end   
       if ascii(ibut)=='x' then xd=xd+3; end   
       if ascii(ibut)=='l' then yd=yd-3; end   
       if ascii(ibut)=='p' then yd=yd+3; end   
       if ascii(ibut)=='v' then V=V+10; end   
       if ascii(ibut)=='V' then V=V-10; end   
       if ascii(ibut)=='z' then ech=ech*0.95; end   
       if ascii(ibut)=='Z' then ech=ech/0.95; end   
       if ascii(ibut)=='F' then sortie=%T;    end   
endfunction


function draw(x)  //x=(x,y,theta,deltav,deltag,v,w)
   rouge=5;bleu=2;blanc=8;vert=3;noir=0;magenta=6;
   theta=x(3);   deltav=x(4); deltag=x(5);
   coque=[-1  4 5  5 4 -1 -1 -1; -2 -2 -1 1 2 2  -2 -2 ; ones(1:8)] ;
   voile=[-5*L 0;0 0;1 1];  
   gouvernail=[-1 1;0 0;1 1];
   [fv,fg]=forces(x);
   Mfv=[-L -L;0 -fv/100;1 1];
   Mfg=[0 0;0 fg/100;1 1];
   R=[cos(theta),-sin(theta),x(1);sin(theta),cos(theta),x(2);0 0 1];
   coque=R*coque;    
   Rdeltav=[cos(deltav),-sin(deltav),3;sin(deltav),cos(deltav),0;0 0 1];
   Rdeltag=[cos(deltag),-sin(deltag),-1;sin(deltag),cos(deltag),0;0 0 1];
   voile=R*Rdeltav*voile;
   Mfv=R*Rdeltav*Mfv;
   gouvernail=R*Rdeltag*gouvernail;
   Mfg=R*Rdeltag*Mfg;
   xbasc();
   xset('thickness',2);
   isoview(-ech,ech,-ech,ech)
   xset('color',magenta);
   r1=10;xarc(-r1+xd,r1+yd,2*r1,2*r1,0,0);
   xset('color',noir);
   xset('font',2,12);
   xstring(-ech,ech,["Vent = "+string(V)+"m/s"]) ;
   xarrows([-ech+5,-ech+5],[ech,ech-V],2,bleu)
   xpoly(coque(1,:),coque(2,:))
   xset('color',5)
   xpoly(voile(1,:),voile(2,:))
   xpoly(gouvernail(1,:),gouvernail(2,:))
   xset('color',3)
   xset('thickness',1);
   xarrows(Mfv(1,:),Mfv(2,:),1,vert)
   xarrows(Mfg(1,:),Mfg(2,:),1,vert)
endfunction;

//---------------------------------------------
function [fv,fg]=forces(x)  //x=(x,y,theta,deltav,deltag,v,w)
     theta=x(3);deltav=x(4);deltag=x(5);vitesse=x(6);omega=x(7);
     fv=alphav*V*cos(theta+deltav)-alphav*vitesse*sin(deltav)
     fg=alphag*vitesse*sin(deltag);
endfunction;

//---------------------------------------------
function Dx=f(x,u)  //x=(x,y,theta,deltav,deltag,v,w)
     theta=x(3);deltav=x(4);deltag=x(5);vitesse=x(6);omega=x(7);
     [fv,fg]=forces(x);
     Dx=[vitesse*cos(theta);
        vitesse*sin(theta)-beta*V;
        omega;    
       	u(1);u(2);
       	(1/m)*( fv*sin(deltav)-fg*sin(deltag)-vitesse*alphaf);
        (1/J)*((L-rv*cos(deltav))*fv-rg*fg*cos(deltag)-alphatheta*omega);
       ];
endfunction;

//---------------------------------------------
function theta1=arg(a,theta)  //calcule theta1, le + proche de a, tq theta1=theta+2k*pi
  k=round((a-theta)/(2*%pi)); 
  theta1=theta+2*k*%pi;
endfunction  

//---------------------------------------------
function u=r(x,w)
     theta=x(3);deltav=x(4);deltag=x(5);vitesse=x(6);omega=x(7);
     [fv,fg]=forces(x);
     D1theta=omega;
     D1vitesse=(1/m)*( fv*sin(deltav)-fg*sin(deltag)-vitesse*alphaf);
     D1omega= (1/J)*((L-rv*cos(deltav))*fv-rg*fg*cos(deltag)-alphatheta*omega);
     A1=[1,0;(rv/J)*fv*sin(deltav),(rg/J)*fg*sin(deltag)];
     A2=[0,0;L/J-rv/J*cos(deltav),-rg/J*cos(deltag)];
     A3=[-alphav*(V*sin(theta+deltav)+vitesse*cos(deltav)),0;0,alphag*vitesse*cos(deltag)];
     b1=[0;-alphatheta/J*D1omega];
	   b2=[-alphav*(V*omega*sin(theta+deltav)+D1vitesse*sin(deltav));alphag*D1vitesse*sin(deltag)];
	   A=A1+A2*A3;
	   b=A2*b2+b1;
	   u=inv(A)*((w-[deltav;theta+3*omega+3*D1omega])-b);
endfunction;

//---------------------------------------------
function deltav=ouverture(cap)
   deltav=%pi*(floor((cap+%pi/2)/(2*%pi)))+%pi/4-cap/2
endfunction;

//---------------------------------------------
function [cap,q]=automate(cap,q);
   global tq;
	 a=12; 
	 tq1=5;
	 tq=tq+dt;
 	 if (tq>tq1)&(q==3)&(x(1)-xd> a) then q=4; tq=0; end;  //3->4
 	 if (tq>tq1)&(q==4)&(x(1)-xd<-a) then q=3; tq=0; end;  //4->3
   if (tq>tq1)&(q==3)&(x(2)-yd> a) then q=1; tq=0; end;  //3->1
   if (tq>tq1)&(q==1)&(x(2)-yd<-a) then q=3; tq=0; end;  //1->3
   if (tq>tq1)&(q==1)&(x(1)-xd> a) then q=2; tq=0; end;  //1->2
   if (tq>tq1)&(q==2)&(x(1)-xd<-a) then q=1; tq=0; end;  //2->1
   if (tq>tq1)&(q==4)&(x(2)-yd> a) then q=2; tq=0; end;  //4->2
   if (tq>tq1)&(q==2)&(x(2)-yd<-a) then q=4; tq=0; end;  //2->4
   select q
      case 1 then cap1=arg(cap,7*%pi/4),
      case 2 then cap1=arg(cap,5*%pi/4),
      case 3 then cap1=arg(cap,%pi/6),
      case 4 then cap1=arg(cap,5*%pi/6),
   end
   cap=cap+dt*(cap1-cap);
endfunction;

//---------------------------------------------
global xd yd V sortie ech tq;
xd=0;yd=0;tq=0;
sortie=%F;
ech=60;
q=1;
L=1;											//	distance mat-centre de poussée de la voile
alphaf=60; 							  	//	frottement de l'eau
alphatheta=500;      //	frottement angulaire de l'eau
alphav=500; 	        //	portance de la voile
alphag=15*20; 	      //	portance du gouvernail
beta=0.05;           // dérive du bateau
rv=1; 		             //	distance de G au mat
rg=2; 		             //	distance de G au gouvernail 
V=10;	               //	vent (m/sec)
m=1000;		            //	masse du bateau en kg
J=2000;		            //	moment d'inertie
x0=[0;-50;-0.5;0;0.0;2;0];   //x=(x,y,theta,deltav,deltag,v,w)
x=x0;
cap=x(3);
t=0;dt=0.01;
seteventhandler('my_eventhandler');
while sortie==%F, 
        pvm_set_timer();
        t=t+dt;
        [cap,q]=automate(cap,q);
        w=[ouverture(cap);cap];
        u=r(x,w);
        x=x+f(x,u)*dt;
        draw(x);
        dt=8*pvm_get_timer()/1000000;
end;
seteventhandler('') //suppress the event handler
xdel();

