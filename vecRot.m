function [vecRot] = vecRot(vx,vy,vz,angle)
%Função que rotaciona um vetor em coordenadas esferias mudando 
%o seu angulo phi
%
%   Não elimina o vetor de entrada e guarda as coordenadas dos novos
%   vetores numa matriz 3D
%   O angulo de entrada é em graus
vecRot(:,:,1)=zeros(1,3);
vecRot(:,:,1)=[vx vy vz];
for i=1:1
    [theta,phi,r]=cart2sph(vx,vy,vz);
    vecRot(:,:,i)=[theta,phi+(angle*i*pi)/180,r];
    [xx,yy,zz]=sph2cart(vecRot(:,1,i),vecRot(:,2,i),vecRot(:,3,i));
    vecRot(:,:,i)=[xx yy zz];
    

end

