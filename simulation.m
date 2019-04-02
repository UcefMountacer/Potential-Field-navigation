clear; clc;
% S1=75;
% S2=165;
% Sensors=[S1,S2];

% zone étude
[x,y] = meshgrid(1:0.1:30,1:0.1:30);


% Target

T=[28 28];
xT=T(1); yT=T(2);

% Obstacles
O1=[25 5];xO1=O1(1); yO1=O1(2);
O2=[5 8];xO2=O2(1); yO2=O2(2);
O3=[20 20];xO3=O3(1); yO3=O3(2);
O4=[10 10];xO4=O4(1); yO4=O4(2);

Obstacls = [O1,O2,O3,O4];        %maybe this array willl help in the programming

% Potential field calculation
% Attractive Ka
Ka=10;
ra=sqrt((xT-x).^2+(yT-y).^2);
Fax=Ka*(xT-x)./(ra);Vax=Ka*ra; %les V sont les U dans l'énoncé cad le champs potentiel
Fay=Ka*(yT-y)./(ra);Vay=Ka*ra;

% Obstacle
Krep=10;

% rO1=sqrt((xO1-x).^2+(yO1-y).^2);
% rO2=sqrt((xO2-x).^2+(yO2-y).^2);
% rO3=sqrt((xO3-x).^2+(yO3-y).^2);
% rO4=sqrt((xO4-x).^2+(yO4-y).^2);

% Vrep1=Krep./rO1;
% Frep1x=-Krep*(xO1-x)./(rO1)^3;
% Frep1y=-Krep*(yO1-y)./(rO1)^3;  
% 
% Vrep2=Krep./rO2;
% Frep2x=-Krep*(xO2-x)./(rO2)^3;
% Frep2y=-Krep*(yO2-y)./(rO2)^3; 
% 
% Vrep3=Krep./rO3;                       %<----necessary?
% Frep3x=-Krep*(xO3-x)./(rO3)^3;
% Frep3y=-Krep*(yO3-y)./(rO3)^3; 
% 
% Vrep4=Krep./rO4;
% Frep4x=-Krep*(xO4-x)./(rO4)^3;
% Frep4y=-Krep*(yO4-y)./(rO4)^3; 
% 
% mesh(x,y,Vax+Vrep1+Vrep2+Vrep3+Vrep4);  
% set(gca,'visible','off')

% déplacement
clear x y Vax Vay Vrep1 Vrep2 Vrep3 Vrep4
x(1)=1; y(1)=1;
%on commence à la position (1,1)
Va(1)=0;
Vrep1(1)=0;
i=1;
rT=sqrt((xT-x(1))^2+(yT-y(1))^2);
% rO1=sqrt((xO1-x(1))^2+(yO1-y(1))^2);
% rO2=sqrt((xO2-x(1)).^2+(yO2-y(1)).^2);
% rO3=sqrt((xO3-x(1)).^2+(yO3-y(1)).^2);
% rO4=sqrt((xO4-x(1)).^2+(yO4-y(1)).^2);

Te=1/100;

% while rT>0.1
% Va(i+1)=Ka*rT;
%    Fax=Ka*(xT-x(i))/(rT);
%    Fay=Ka*(yT-y(i))/(rT);
%    
%    Vrep1(i+1)=Krep/rO1;
%    Frep1x=-Krep*(xO1-x(i))/(rO1)^3;
%    Frep1y=-Krep*(yO1-y(i))/(rO1)^3;   %<---------------
%    
%    Fx=Fax+Frep1x;
%    Fy=Fay+Frep1y;             %<---------------
%    Theta=atan2(Fy,Fx);
%    %-------------------------------
%    %partie modifiée----------------
%    [beta]=champs_sonar(rO1);
%    Th=Theta+beta;
%    vx=cos(Th);                              
%    vy=sin(Th);
%    %-------------------------------
%    x(i+1)=x(i)+vx*Te;
%    y(i+1)=y(i)+vy*Te;
%    patch(a+x(i+1),b+y(i+1)+30);            %<-patch here + 30 cm marge 
%    i=i+1;
%    rT=sqrt((xT-x(i))^2+(yT-y(i))^2);
%    R(i)=rT;
%    rO1=sqrt((xO1-x(i))^2+(yO1-y(i))^2);
% end

while rT>0.1
    Va(i+1)=Ka*rT;
    Fax=Ka*(xT-x(i))/(rT);
    Fay=Ka*(yT-y(i))/(rT);
    %part where to code moving rectangle (1 for the time)
    a=[-10,10,10,-10];
    b=[20,20,40,40];
    %dimensions of the rectangle
    g = hgtransform;
    %----------------------------------------------------
    %[in] = inside_1( Obstacls , a , b );
    in=[];
    O1=Obstacls(1);
    O2=Obstacls(2);
    O3=Obstacls(3);
    O4=Obstacls(4);
    in1=inpolygon(xO1,yO1,a,b);
    in(1)=in1;
    in2=inpolygon(xO2,yO2,a,b);
    in(1)=in2;
    in3=inpolygon(xO3,yO3,a,b);
    in(3)=in3;
    in4=inpolygon(xO4,yO4,a,b);
    in(4)=in4;
    if(in(1)==1 || in(2)==1 || in(3)==1 || in(4)==1);
        I = find(in==1); % index of the obstacl inside the rectangle
        Od = O(I); % obstacle detected Od
        r=sqrt((xOd-x).^2+(yOd-y).^2);
        Vrep=Krep./rOd;
        Frepx=-Krep*(xOd-x)./(rOd)^3;
        Frepy=-Krep*(yOd-y)./(rOd)^3;
        Fx=Fax+Frepx;
        Fy=Fay+Frepy;
        Theta=atan2(Fy,Fx);
        x(i+1)=x(i)+vx*Te;
        y(i+1)=y(i)+vy*Te;
        patch(a+x(i+1),b+y(i+1)+30);            %<-patch here + 30 cm marge 
        i=i+1;
        rT=sqrt((xT-x(i))^2+(yT-y(i))^2);
        R(i)=rT;
    end    
end


% figure
hold;plot3(x(2:end),y(2:end),Va(2:end)+Vrep1(2:end)+0.2,'Or');