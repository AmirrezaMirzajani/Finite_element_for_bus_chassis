%% Amirreza mirzajani
% 975241045



%%
clc
close all

%% 
%  First we determine the  ...
 ... coordinates of the nodes relative to the global coordinates.
 % The global device is at point c.
 % The choice of global device is optional but this point is easier.
 % coordition matrix
coord_matrix=[0 0 ; 1 0 ; 0 -0.4 ; 0.4284 -0.2286 ; 1 -0.5333 ; 2 0 ; ...
    2 -0.6667 ; 3 0 ; 3 -0.8 ; 3.3333 -0.2667 ; 3.5 0 ; 3.5 -0.4];
 %   Then we write a matrix of nodes that ...
 ... together form the truss elements in pairs.
     % connection matrix
 con_matrix=[1 3 ; 1 2 ; 1 4 ; 2 4 ; 3 4; 4 5; 3 5 ; 2 5; 2 6 ;2 7; 5 7 ...
     ; 6 7 ; 6 8 ; 6 9 ; 7 9 ; 8 9 ; 8 11 ; 8 10 ; 10 11 ; 9 10 ; 10 12 ...
     ; 9 12 ; 11 12];
 
%%
% The modulus of elasticity and the cross-sectional area of ...
... the diagonal, horizontal and vertical elements are ...
    ... different from each other.
% Therefore, we calculate the modulus and area ...
... for different types of elements separately

% First we find the modulus and area for the horizontal elements
% The dimensions of the horizontal elements along with their thickness ...
... are known to us.  
% Horizontal element information:
% [the length , Width , Thickness]
H=[100 60 3 ];  % Units in millimeters
% Horizontal element length(HEL) :
HEL=H(1)*10^(-3); % Unit in meters
% Horizontal element width(HEW) :
HEW=H(2)*10^(-3);  % Unit in meters
% Thickness of horizontal element(HET) :
HET=H(3)*10^(-3);  % Unit in meters
% Calculate the cross-sectional area of a horizontal element:
HA=(HEW*HET*2+(HEL-2*HET)*HET*2);

disp(['cross-sectional area of a horizontal element: ((' ...
    , num2str(HA),'))in mm^2']);

 fprintf('\n');
%The modulus of elasticity corresponding ...
... to the horizontal elements is as follows
% The modulus of elasticity of the horizontal element(HEM) :
HEM=210*10^9;  %Units in Pascals

% In calculating the stiffness matrix,...
... we will directly use the coefficient of modulus of elasticity... 
    ... in the cross-sectional area.
 % So at the beginning we do the coefficient ...
... of these two to make our work easier
 EA_H=HEM*HA;
disp(['Modulus*area: ((', num2str(EA_H),')) in m^2.pa(N) or '...
    , num2str(EA_H*10^(-6)),' in mm^2.pa(N)']);

 fprintf('\n');
%After the horizontal elements, ...
... it is the turn of the vertical elements
     
% Vertical element information :
% [the length , Width , Thickness]
V=[60 60 3 ];  % Units in millimeters
% Vertical element length(HEL) :
VEL=V(1)*10^(-3); % Unit in meters
% Vertical element width(HEW) :
VEW=V(2)*10^(-3);  % Unit in meters
% Thickness of Vertical element(HET) :
VET=V(3)*10^(-3);  % Unit in meters
% Calculate the cross-sectional area of a Vertical element:
VA=(VEW*VET*2+(VEL-2*VET)*VET*2);

disp(['cross-sectional area of a Vertical element: ((' ...
    , num2str(VA),'))in mm^2']);

 fprintf('\n');
% Multiply the modulus of elasticity ...
... by the cross section for vertical elements
    
VEM=200*10^9;  %Units in Pascals
 EA_V=VEM*VA;
disp(['Modulus*area: ((', num2str(EA_V),')) in m^2.pa(N)m or ' ,...
     num2str(EA_V*10^(-6)), ' in mm^2.pa(N)']);


 fprintf('\n');
% Finally, for diagonal elements, we do the following 
     
% diagonal element information :
% [the length , Width , Thickness]
D=[60 40 2 ];  % Units in millimeters
% diagonal element length(HEL) :
DEL=D(1)*10^(-3); % Unit in meters
% diagonal element width(HEW) :
DEW=D(2)*10^(-3);  % Unit in meters
% Thickness of diagonal element(HET) :
DET=V(3)*10^(-3);  % Unit in meters
% Calculate the cross-sectional area of a diagonal element:
DA=(DEW*DET*2+(DEL-2*DET)*DET*2);

disp(['cross-sectional area of a diagonal element: ((' ...
    , num2str(DA),'))in mm^2']);
fprintf('\n');
% Multiply the modulus of elasticity by ...
... the cross section for diagonal elements
DEM=195*10^9;  %Units in Pascals
 EA_D=DEM*DA;
disp(['Modulus*area: ((', num2str(EA_D),')) in m^2.pa(N) or ',...
     num2str(EA_D*10^(-6)), ' in mm^2.pa(N)']);
fprintf('\n');
% number of elements
NE=numel( con_matrix(:,1));


% We now specify the EA value for each element
%By first specifying the type of element ...
... horizontally, vertically or diagonally
% We have 23 elements so we first create a matrix with zero values.
Area=zeros(NE,1);

% Horizontal elements :
... [2] , [9] , [13] , [17]
Area(2)=HA;
Area(9)=HA;
Area(13)=HA;
Area(17)=HA;

% Vertical elements :
... [1] , [8] , [12] , [16] , [23]
Area(1)=VA;
Area(8)=VA;
Area(12)=VA;
Area(16)=VA;
Area(23)=VA;

% Diagonal elements :
... [3],[5],[4],[6],[7],[10],[11],[14],[15],[18],[19],[20],[21],[22]
Area(3)=DA;
Area(5)=DA;
Area(4)=DA;
Area(6)=DA;
Area(7)=DA;
Area(10)=DA;
Area(11)=DA;
Area(14)=DA;
Area(15)=DA;
Area(18)=DA;
Area(19)=DA;
Area(20)=DA;
Area(21)=DA;
Area(22)=DA;
fprintf('\n\n\n');
disp('The cross-sectional area of the elements:');

for i=1:NE
disp(['In element  [' ,num2str(i) ...
    ,']  Area is: (', num2str(Area(i)*10^(6)), ') in mm^2']);
end
%%
A=vpa(Area);
digits(6);
Areaa=sym('Area_%d' ,[23,1])==A
%%
% Modulus of elasticity : (E)
E=zeros(NE,1);
% Horizontal elements :
... [2] , [9] , [13] , [17]
E(2)=HEM;
E(9)=HEM;
E(13)=HEM;
E(17)=HEM;

% Vertical elements :
... [1] , [8] , [12] , [16] , [23]
E(1)=VEM;
E(8)=VEM;
E(12)=VEM;
E(16)=VEM;
E(23)=VEM;

% Diagonal elements :
... [3],[5],[4],[6],[7],[10],[11],[14],[15],[18],[19],[20],[21],[22]
E(3)=DEM;
E(5)=DEM;
E(4)=DEM;
E(6)=DEM;
E(7)=DEM;
E(10)=DEM;
E(11)=DEM;
E(14)=DEM;
E(15)=DEM;
E(18)=DEM;
E(19)=DEM;
E(20)=DEM;
E(21)=DEM;
E(22)=DEM;
%%
Modulus=vpa(E);
% Modulus_of_elasticity=sym('E_%d' ,[23,1])==Modulus
EE=E*10^(-9);
for cnt=1:23
    fprintf('E%d = %7.3f\n' , cnt,EE(cnt));
  
end
%%

% For ease of work in calculating the stiffness matrix...
... , we calculate the cross-sectional area coefficient...
    ...and the modulus of elasticity for each element.
       ...And we place it in the matrix called EA :
EA=zeros(NE,1);
% Horizontal elements :
... [2] , [9] , [13] , [17]
EA(2)=HEM*HA;
EA(9)=HEM*HA;
EA(13)=HEM*HA;
EA(17)=HEM*HA;

% Vertical elements :
... [1] , [8] , [12] , [16] , [23]
EA(1)=VEM*VA;
EA(8)=VEM*VA;
EA(12)=VEM*VA;
EA(16)=VEM*VA;
EA(23)=VEM*VA;

% Diagonal elements :
... [3],[5],[4],[6],[7],[10],[11],[14],[15],[18],[19],[20],[21],[22]
EA(3)=DEM*DA;
EA(5)=DEM*DA;
EA(4)=DEM*DA;
EA(6)=DEM*DA;
EA(7)=DEM*DA;
EA(10)=DEM*DA;
EA(11)=DEM*DA;
EA(14)=DEM*DA;
EA(15)=DEM*DA;
EA(18)=DEM*DA;
EA(19)=DEM*DA;
EA(20)=DEM*DA;
EA(21)=DEM*DA;
EA(22)=DEM*DA;
disp('*********************************************');
disp('*********************************************');
disp('*********************************************');
disp('EA:');
for i=1:NE
disp(['In element  [' ,num2str(i) ...
    ,']  EA is: (', num2str(EA(i)), ') in N']);
end
disp('*********************************************');
disp('*********************************************');
disp('*********************************************');
%%
EAA=vpa(EA);
digits(6);
E_A=sym('EA_%d' ,[23,1])==EAA
%%

% After calculating the cross-sectional area ...
... and the modulus of elasticity of the whole truss,...
    ...we calculate the length of the truss sides.

% To do this, we calculate the change ...
...values of x and y for each side.\

Length=zeros(NE,1);
for k=1:NE;
   i=con_matrix(k,1);
   j=con_matrix(k,2);
   delta_X=coord_matrix(j,1)-coord_matrix(i,1);
   delta_y=coord_matrix(j,2)-coord_matrix(i,2);
   Length(k)=sqrt(delta_X^2+(delta_y)^2);  
   
   
end
for i=1:NE
    disp(['Length of Element [' ...
        , num2str(i),']is: "', ...
        num2str(Length(i)),'" m']);
end
% Each of our elements consists of two nodes, ...
... each of which has two degrees of freedom
   ...As a result, we have a total of 4 degrees of...
      ...freedom for each element.
 
% To understand which element each displacement value belongs to ... 
... We need to create a matrix called the identification matrix
% Each of the nodes has a displacement in two directions,...
... which we consider as the following matrix 
% In fact, the first column is for the x direction...
... and the second column is for the y direction
 EQ=[1 2;3 4; 5 6 ; 7 8; 9 10; 11 12; 13 14; 15 16; 17 18; 19 20 ...
 ;21 22 ;23 24];
detection_matrix=zeros(NE,4);
 for e=1:NE
 
 i = con_matrix(e,1);
 j = con_matrix(e,2);
 detection_matrix(e,1:2)=EQ(i,1:2);
 detection_matrix(e,3:4)=EQ(j,1:2);
 end
 % Calculate the number of truss nodes :
 NN=numel( coord_matrix(:,1));
 % The number of degrees of truss freedom ...
 ... is obtained from the following equation :
 NDOF=2*NN;
K=zeros(NDOF);
for k=1:NE
     i=con_matrix(k,1);
    j=con_matrix(k,2);
    delta_X=coord_matrix(j,1)-coord_matrix(i,1);
    delta_y=coord_matrix(j,2)-coord_matrix(i,2);
    
    landa=[ -delta_X/Length(k) -delta_y/Length(k) ...
        delta_X/Length(k) delta_y/Length(k)];
    STIFFENESS_matrix=landa' .*EA(k)/Length(k)*landa;
    
    for m=1:4
        for n=1:4
            a=detection_matrix(k,m);
         
            b=detection_matrix(k,n);
            K(a,b)=K(a,b)+STIFFENESS_matrix(m,n);
        end
    end
end

%%
% The support for node 11 is of the...
...roller support type and does not move in the y direction
    ...So the amount of u_10 motion will be zero 
  
% Also, the support related to node ...
...12 is of the hinged straw support type
   ...And it does not move in the X 
       ...direction or in the y direction 
       
       
% Therefore, we form the U matrix in the following form :

   
   % u_10=0 && u_12=0 && v_12=0 
   
   syms u_1 v_1 u_2 v_2 u_3 v_3 ...
   u_4 v_4 u_5 v_5 u_6  ...
    v_6 u_7 v_7 u_8 v_8 u_9 v_9 u_10 v_10 v_11 
   
   
   
   U=[u_1;v_1;u_2;v_2;u_3;v_3; ...
   u_4;v_4;u_5;v_5;u_6; ...
    v_6;u_7;v_7;u_8;v_8; u_9 ;v_9 ;u_10;v_10 ;0 ; v_11 ;0 ; 0  ];
 Force=zeros(24,1);
  Force(2)=vpa(-2500); Force(4)=vpa(-5000); Force(9)=vpa(-15000*cosd(45)); ...
 Force(10)=vpa(-15000*sind(45));
  Force(12)=vpa(-5000); Force(14)=-vpa(4000); Force(16)=-vpa(3750); ...
NDOF=2*NN;

% To calculate the displacement values ...
    ...for each element, we first use the elimination method
...And consider the values that are zero in the U matrix
... Deleting these lines makes ...
...our job easier and we can solve the equation
KK=K;
UU=U;
FF=Force;
 for m=24:-1:1
    if U(m)==0
        mm(m)=m;
        KK(m,:)=[];
        KK(:,m)=[];
        UU(m,:)=[];
        FF(m,:)=[];
    end
 end
 UU=linsolve(KK,FF);
 mm=mm(mm~=0);
 nn=mm(1)-1;
 
 displacement=zeros(NDOF,1);
 for n=1:nn;
     displacement(n)=   UU(n);
     displacement(22)=UU(21);
 end
 fprintf('\n\n\n\n');
% Now we use the relation K * U = F to find the forces :
Force=K*displacement;

for cnt=1:length(Force)/2
    fprintf('F%dx = %7.3f\n' , cnt,Force(2*cnt-1));
    fprintf('F%dy = %7.3f\n' , cnt,Force(2*cnt));
    

end

 fprintf('\n\n\n\n');
% F=sym('F_%d' ,[24,1])==Force
% eq1=U==vpa(displacement)
disp('displacement_matrix in mm');
for cnt=1:length(displacement)/2
    
    fprintf('u%d = %7.3f\n' , cnt,displacement(2*cnt-1)*10^(3));
    fprintf('v%d = %7.3f\n' , cnt,displacement(2*cnt)*10^(3));
    

end

 fprintf('\n\n\n\n');
%To calculate the stress, I calculate the strain start :
% link CD:
 i=con_matrix(3,1);
    j=con_matrix(3,2);
 delta_X=coord_matrix(j,1)-coord_matrix(i,1);
    delta_y=coord_matrix(j,2)-coord_matrix(i,2);
    cos_3=delta_X/Length(3);  
    sin_3=delta_y/Length(3); 
    landa=[cos_3 sin_3 0 0;0 0 cos_3 sin_3];
    q_1=[displacement((detection_matrix(3,1)),1);...
        displacement((detection_matrix(3,2)),1); ...
        displacement((detection_matrix(3,3)),1); ...
        displacement((detection_matrix(3,4)),1)];
    ee_1=landa*q_1;
    strain_CD=(-ee_1(1)+ee_1(2))/length(3);
    stress_CD=E(3)*strain_CD;
    
  %Link AB
    i=con_matrix(20,1);
    j=con_matrix(20,2);
 delta_X=coord_matrix(j,1)-coord_matrix(i,1);
    delta_y=coord_matrix(j,2)-coord_matrix(i,2);
    cos_20=delta_X/Length(20);  
    sin_20=delta_y/Length(20); 
    landa=[cos_20 sin_20 0 0;0 0 cos_20 sin_20];
    q_2=[displacement((detection_matrix(20,1)),1);...
        displacement((detection_matrix(20,2)),1); ...
        displacement((detection_matrix(20,3)),1); ...
        displacement((detection_matrix(20,4)),1)];
    ee_2=landa*q_2;
    strain_AB=(-ee_2(1)+ee_2(2))/length(20);
    stress_AB=E(20)*strain_AB;
   stress=[stress_AB stress_CD];
   
    disp(['stress_AB [in MPa] = ' , num2str(stress(1)*10^(-6))]);
    disp(['stress_CD[in MPa] = ' , num2str(stress(2)*10^(-6))]);
    
fprintf('\n\n\n');
 
 for i=1:numel(stress)
     % if The stress associated with ...
     ... the element is of the tensile type
     if stress(i)>0
     
        if i==1
            disp('stress_AB is tensile');
        elseif i==2
            disp('stress_CD is tensile');
        end
     
     elseif stress(i)<0
         if i==1
            disp('stress_AB is Compressive');
        elseif i==2
            disp('stress_CD is Compressive');
        end
     end
 end
 
 % undeformed truss plot
for e=1:NE
    x=[coord_matrix(con_matrix(e,1),1) coord_matrix(con_matrix(e,2),1)];
    y=[coord_matrix(con_matrix(e,1),2) coord_matrix(con_matrix(e,2),2)];
plot(x,y,'b')
grid on
hold on
end
k=0;
for i=1:12
    for j=1:2
    k=k+1;
    nod_coor_def(i,j)=coord_matrix(i,j)+displacement(k,1);
    end
end
% deformed truss plot
for e=1:NE
    x=[nod_coor_def(con_matrix(e,1),1) nod_coor_def(con_matrix(e,2),1)];
    y=[nod_coor_def(con_matrix(e,1),2) nod_coor_def(con_matrix(e,2),2)];
    
plot(x,y,'r--')
xlabel('X(m)');
ylabel('Y(m)');
title('undeformed truss plot & deformed truss plot');
hold on
end




