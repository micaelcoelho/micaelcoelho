function [rx,ry,rz] = linePoint(px,py,pz,ax,ay,az)
%Função que calcula pontos da reta
%   Dados de entrada:
%   - ponto que pertence à reta (px py pz)
%   - vetor que define a reta (alpha, ax ay az)
%   

t=linspace(0,1,10);
rx=px+t.*ax;
ry=py+t.*ay;
rz=pz+t.*az;


end

