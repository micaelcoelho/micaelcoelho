%% Reflexão de um feixe num plano
clear all
close all

% Definição de um vetor
nfeixes=10; % numero de feixes
lidar(:,:,nfeixes)=zeros(1,3);
lidar(:,:,1)=[-1 -2 -10];
point=[4 4 7];

% rotação dos vetores incidentes
for i=1:nfeixes
    lidar(:,:,i+1)=vecRot(lidar(:,1,1),lidar(:,2,1),lidar(:,3,1),i*5);
end


%% Definição de um plano paralelo a xy
[X, Y]=meshgrid(0:9,0:9);
Z=ones(10)*2;
surf(X,Y,Z); %plot do plano
[nx,ny,nz]=surfnorm(X,Y,Z); %calculo da normal ao plano
n=[nx(1,1) ny(1,1) nz(1,1)];
hold on
for i=1:nfeixes
    quiver3(point(1),point(2),point(3),lidar(:,1,i),lidar(:,2,i),lidar(:,3,i)); 
    hold on 
end

%% Cálculo da intersecção do feixe com o plano
a=[1 2 0];
b=[3 1 0];
p=[2 2 2];
% calculo dos pontos do plano
[px,py,pz]=planePoint(p(1),p(2),p(3),a(1),a(2),a(3),b(1),b(2),b(3));
% calculo dos pontos da reta
R(:,:,nfeixes)=zeros(3,10);
for i=1:nfeixes
    [rx,ry,rz]=linePoint(point(1),point(2),point(3),lidar(:,1,i),lidar(:,2,i),lidar(:,3,i));
    R(:,:,i)=[rx; ry; rz];
end

% calculo do ponto de intersecção
PI(:,:,nfeixes)=zeros(3,1); % Matriz dos pontos de intersecção
for i=1:nfeixes
    [px_intersec,py_intersec,pz_intersec]=planeLineInt(pz,point(1),point(2),point(3),lidar(:,1,i),lidar(:,2,i),lidar(:,3,i));
    PI(:,:,i)=[px_intersec; py_intersec; pz_intersec];
    plot3(PI(1,1,i), PI(2,1,i), PI(3,1,i), 'p','Color','g','MarkerSize',10);
    hold on
end


%% Feixes refletidos
[nx,ny,nz]=surfnorm(X,Y,Z); %calculo da normal ao plano
n=[nx(1,1) ny(1,1) nz(1,1)];

C(:,:,nfeixes)=zeros(1,3);
theta(:,:,nfeixes)=zeros(1,1);
vec_rot(:,:,nfeixes)=zeros(1,3);
for i=1:nfeixes
    C(:,:,i)=cross(n,lidar(:,:,i));
    % angulo entre a normal do espelho e o feixe incidente em graus
    CosTheta = max(min(dot(lidar(:,:,i),n)/(norm(lidar(:,:,i))*norm(n)),1),-1);
    ThetaInDegrees = real(acosd(CosTheta));
    theta(:,:,i)=180-ThetaInDegrees; 
    vec_rot(:,:,i)=vecRot(lidar(1,1,i),lidar(1,2,i),lidar(1,3,i),180-2*theta(1,1,i));
    quiver3(PI(1,1,i), PI(2,1,i), PI(3,1,1), vec_rot(1,1,i), vec_rot(1,2,i), vec_rot(1,3,i));
    hold on 
end

%% Objeto ou obstáculo
a_ob=[2 1 10]; % vetor alpha do plano
b_ob=[1 2 10.1]; % vetor beta do plano
p_ob=[2 1 10.1]; % ponto do plano
% calculo dos pontos do plano
[px_ob,py_ob,pz_ob]=planePoint(p_ob(1),p_ob(2),p_ob(3),a_ob(1),a_ob(2),a_ob(3),b_ob(1),b_ob(2),b_ob(3));
surf(px_ob,py_ob,pz_ob);
hold on
[nx_ob,ny_ob,nz_ob]=surfnorm(px_ob,py_ob,pz_ob); %calculo da normal ao plano
n_ob=[nx_ob(1,1) ny_ob(1,1) nz_ob(1,1)];
%surf(px_ob,py_ob,pz_ob);
% calculo dos pontos da reta
R_ob(:,:,nfeixes)=zeros(3,10);
for i=1:nfeixes
    [rx,ry,rz]=linePoint(PI(1,1,i),PI(2,1,i),PI(3,1,i),vec_rot(:,1,i),vec_rot(:,2,i),vec_rot(:,3,i));
    R_ob(:,:,i)=[rx; ry; rz];
end

% calculo e plot dos pontos de intersecção
PI_ob(:,:,nfeixes)=zeros(3,1); % Matriz dos pontos de intersecção
for i=1:nfeixes
    [I, check]=plane_line_intersect(n_ob,p_ob,R_ob(:,1,i)',R_ob(:,10,i)');
    PI_ob(:,:,i)=[I];
    plot3(PI_ob(1,1,i), PI_ob(2,1,i), PI_ob(3,1,i), '*','Color','r','MarkerSize',10);
    hold on
end


