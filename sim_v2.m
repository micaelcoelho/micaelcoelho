clear all
close all


%lidarVec=[0 -1 -0.1];
axisrot=[0 0 1]; % eixo sobre o qual se rotaciona o 1º vetor do lidar
axisrotH=[axisrot 1];
lidarVec=makehgtform('xrotate',deg2rad(90))*axisrotH';


lidarPack=lidarVec(1:3)';
pointLidar=[5,5,5];

quiver3(pointLidar(1),pointLidar(2),pointLidar(3),lidarPack(1),lidarPack(2),lidarPack(3),20); % plot
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on


for i=-16:2:16
    M=makehgtform('axisrotate',axisrot,deg2rad(i));
    %lidarVec(4)=1; % torna homo
    lidarRot=M*lidarVec; % rotação
    lidarRot=lidarRot(1:3); % retira homo
    lidarPack=[lidarPack;lidarRot']; % guarda
    
    q=quiver3(pointLidar(1),pointLidar(2),pointLidar(3),lidarRot(1),lidarRot(2),lidarRot(3),10); % plot
    %H=[H i];
    hold on
end

%%
% definição do plano de refleção
a=[0 0 4];
b=[5 0 0];
p=[10 -5 5];

[xx,yy,zz]=planePoint(p(1),p(2),p(3),a(1),a(2),a(3),b(1),b(2),b(3));
surf(xx,yy,zz);
[nx,ny,nz]=surfnorm(xx,yy,zz);
n=[nx(1,1),ny(1,1),nz(1,1)];

pointIntersec=[];
Check=[];
for i=1:size(lidarPack,1)
    t=50;
    pointFinal=pointLidar+t*lidarPack(i,:);
    [I,check]=plane_line_intersect(n,p,pointLidar,pointFinal);
    Check=[Check check];
    pointIntersec=[pointIntersec; I];
    if check ~=0
        plot3(I(1),I(2),I(3),'*');
        hold on
    end
end

% Preciso de arranjar forma para calcular um vetor perpendicular a outros 2
% na linha 73
%
% preciso corrigir o angulo de reflexão ou o plot, não sei bem onde está o
% problema
%
%


theta=[];
lidarRef=[];
for i=1:size(lidarPack,1)
    % angulo entre a normal do espelho e o feixe incidente em graus
    CosTheta = max(min(dot(lidarPack(i,:),n)/(norm(lidarPack(i,:))*norm(n)),1),-1);
    ThetaInDegrees = real(acosd(CosTheta));
    theta=[theta ThetaInDegrees];
        T=makehgtform('axisrotate',[0 0 1],-deg2rad(180-2*ThetaInDegrees));
        lidarpack=lidarPack(i,:); lidarpack(4)=1;
        lidarref=T*lidarpack';
        lidarref=lidarref(1:3);
        lidarRef=[lidarRef; lidarref'];
    if Check(i)~=0
        quiver3(pointIntersec(i,1),pointIntersec(i,2),pointIntersec(i,3),lidarRef(i,1),lidarRef(i,2),lidarRef(i,3),20);
        hold on
    end
end

% definição do plano que representa o objeto
a2=[0 0 4];
b2=[5 0 5];
p2=[10 10 5];

[xx2,yy2,zz2]=planePoint(p2(1),p2(2),p2(3),a2(1),a2(2),a2(3),b2(1),b2(2),b(3));
surf(xx2,yy2,zz2);
[nx2,ny2,nz2]=surfnorm(xx2,yy2,zz2);
n2=[nx2(1,1),ny2(1,1),nz2(1,1)];

pointIntersec2=[];
Check2=[];
for i=1:size(lidarPack,1)
    t=50;
    pointFinal=pointIntersec(i,:)+t*lidarRef(i,:);
    [I2,check2]=plane_line_intersect(n2,p2,pointIntersec(i,:),pointFinal);
    Check2=[Check2 check2];
    pointIntersec2=[pointIntersec2; I2];
    if check2 ~=0
        plot3(I2(1),I2(2),I2(3),'o');
        hold on
    end
end
