function [xx,yy,zz] = planePoint(px,py,pz,ax,ay,az,bx,by,bz)
%Função que calcula pontos do plano
%   Dados de entrada:
%   - ponto que pertence ao plano (px py pz)
%   - vetores que definem o plano (alpha e beta, ax ay az e bx by bz)
%   

[X, Y]=meshgrid(-2:2,-2:2);
xx=px+X.*ax+Y.*bx;
yy=py+X.*ay+Y.*by;
zz=pz+X.*az+Y.*bz;





end

