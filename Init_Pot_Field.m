clear; clc;

% zone étude
[x,y] = meshgrid(1:0.1:10,1:0.1:10);


% Target

T=[7 9];
xT=T(1); yT=T(2);

% Obstacles
O1=[5 6];xO1=O1(1); yO1=O1(2);    %<---------------


% Potential field
% Attractive Ka
Ka=1;
ra=sqrt((xT-x).^2+(yT-y).^2);
Fax=Ka*(xT-x)./(ra);Vax=Ka*ra; %les V sont les U dans l'énoncé cad le champs potentiel
Fay=Ka*(yT-y)./(ra);Vay=Ka*ra;

% Obstacle
Krep=1;

G=2; R0=1;

rO1=sqrt((xO1-x).^2+(yO1-y).^2);
Vrep1=Krep./rO1;
Frep1x=-Krep*(xO1-x)./(rO1)^3;
Frep1y=-Krep*(yO1-y)./(rO1)^3;      %<---------------


mesh(x,y,Vax+Vrep1);  
% mesh(x,y,Vrep2);
set(gca,'visible','off')
% colormap(bone)
% grid off

% déplacement
clear x y Vax Vay Vrep1 Vrep2
x(1)=1; y(1)=1;
%on commence à la position (1,1)
Va(1)=0;
Vrep1(1)=0;
i=1;
rT=sqrt((xT-x(1))^2+(yT-y(1))^2);
rO1=sqrt((xO1-x(1))^2+(yO1-y(1))^2);   %<---------------

 
Te=1e-2;
while rT>0.1
   Va(i+1)=Ka*rT;
   Fax=Ka*(xT-x(i))/(rT);
   Fay=Ka*(yT-y(i))/(rT);
   
   Vrep1(i+1)=Krep/rO1;
   Frep1x=-Krep*(xO1-x(i))/(rO1)^3;
   Frep1y=-Krep*(yO1-y(i))/(rO1)^3;   %<---------------
   
   Fx=Fax+Frep1x;
   Fy=Fay+Frep1y;             %<---------------
   Theta=atan2(Fy,Fx);
   %-------------------------------
   %partie modifiée----------------
   [beta]=champs_sonar(rO1);
   Th=Theta+beta;
   vx=cos(Th);                              %<---------------
   vy=sin(Th);
   %-------------------------------
   x(i+1)=x(i)+vx*Te;
   y(i+1)=y(i)+vy*Te;
   
   i=i+1;
   rT=sqrt((xT-x(i))^2+(yT-y(i))^2);
   R(i)=rT;
   rO1=sqrt((xO1-x(i))^2+(yO1-y(i))^2);
% gestion cas bloqué
   if i>10 & abs(R(i)-R(i-9))<0.02  
       %ou le robot risque d'aller a l'infini ??!! 
break
   end
end
% figure
hold;plot3(x(2:end),y(2:end),Va(2:end)+Vrep1(2:end)+0.2,'Or'); %add +Vrep2(2:end)   %<---------------



