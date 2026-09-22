%% quick numeric test: BEARING-ONLY radar tracking, EKF vs UKF
clear; clc; rng(7);
T=1.0; N=60; sig_th=deg2rad(1.5); R=sig_th^2;
q_v=0.25^2; q_Om=deg2rad(1.2)^2; Q=diag([1e-4,1e-4,q_v,1e-6,q_Om]);
x=zeros(5,N); x(:,1)=[95;60;6.0;deg2rad(120);deg2rad(3.8)];
for k=1:N-1
    if k==16, x(5,k)=deg2rad(-4.2); elseif k==34, x(5,k)=deg2rad(3.6); elseif k==50, x(5,k)=deg2rad(-2.4); end
    x(:,k+1)=f_ct(x(:,k),T)+[0;0;sqrt(q_v)*randn;0;sqrt(q_Om)*randn];
end
y=zeros(1,N); for k=1:N, y(k)=atan2(x(2,k),x(1,k))+sig_th*randn; end
x0=x(:,1)+[15;-15;1.5;deg2rad(15);deg2rad(2)];
P0=diag([25^2,25^2,3^2,deg2rad(28)^2,deg2rad(5)^2]);

% EKF
xe=zeros(5,N); xe(:,1)=x0; P=P0; I5=eye(5);
for k=1:N-1
    A=jac_f(xe(:,k),T); xp=f_ct(xe(:,k),T); Pp=A*P*A'+Q;
    px=xp(1);py=xp(2); r2=px^2+py^2; H=[-py/r2 px/r2 0 0 0];
    yp=atan2(py,px); inn=wrapToPi(y(k+1)-yp); S=H*Pp*H'+R; Kk=Pp*H'/S;
    xe(:,k+1)=xp+Kk*inn; xe(4,k+1)=wrapToPi(xe(4,k+1)); P=(I5-Kk*H)*Pp;
end
% UKF
n=5; kappa=1; c=sqrt(n+kappa); wm=[kappa/(n+kappa) repmat(1/(2*(n+kappa)),1,2*n)]; wc=wm;
xu=zeros(5,N); xu(:,1)=x0; P=P0;
for k=1:N-1
    Ls=chol(P,'lower'); X=[xu(:,k) xu(:,k)+c*Ls xu(:,k)-c*Ls];
    Xp=zeros(5,2*n+1); for i=1:2*n+1, Xp(:,i)=f_ct(X(:,i),T); end
    mp=Xp*wm'; mp(4)=atan2(sum(wm.*sin(Xp(4,:))),sum(wm.*cos(Xp(4,:))));
    dX=Xp-mp; dX(4,:)=wrapToPi(dX(4,:)); Pp=(wc.*dX)*dX'+Q;
    Yp=atan2(Xp(2,:),Xp(1,:)); yh=atan2(sum(wm.*sin(Yp)),sum(wm.*cos(Yp)));
    dY=wrapToPi(Yp-yh); Sy=(wc.*dY)*dY'+R; Pxy=(wc.*dX)*dY';
    Kk=Pxy/Sy; inn=wrapToPi(y(k+1)-yh); xu(:,k+1)=mp+Kk*inn; xu(4,k+1)=wrapToPi(xu(4,k+1));
    P=Pp-Kk*Sy*Kk'; P=(P+P')/2;
end
rmse=@(a,b) sqrt(mean(sum((a-b).^2,1)));
fprintf('BEARING-ONLY  position RMSE:  EKF = %.2f m | UKF = %.2f m\n', rmse(xe(1:2,:),x(1:2,:)), rmse(xu(1:2,:),x(1:2,:)));
fprintf('final pos err:  EKF = %.2f | UKF = %.2f\n', norm(xe(1:2,end)-x(1:2,end)), norm(xu(1:2,end)-x(1:2,end)));

function xn=f_ct(x,T)
    px=x(1);py=x(2);v=x(3);ph=x(4);Om=x(5);
    if abs(Om)<1e-6, px2=px+v*T*cos(ph); py2=py+v*T*sin(ph);
    else, px2=px+(v/Om)*(sin(ph+Om*T)-sin(ph)); py2=py+(v/Om)*(-cos(ph+Om*T)+cos(ph)); end
    xn=[px2;py2;v;ph+Om*T;Om];
end
function A=jac_f(x,T), m=numel(x);A=zeros(m);h=1e-6; for j=1:m, d=zeros(m,1);d(j)=h; A(:,j)=(f_ct(x+d,T)-f_ct(x-d,T))/(2*h); end, end
function a=wrapToPi(a), a=mod(a+pi,2*pi)-pi; end
