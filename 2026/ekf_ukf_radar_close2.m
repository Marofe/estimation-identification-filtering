%% EKF vs UKF — Target Tracking with Radar (hexacopter drone)
%  SEL5917 — Nonlinear Filtering
%  Same model as slides 32-33:
%    state x=[px;py;v;phi;Om]  (nearly-constant-turn dynamics)
%    radar measurement y=[r;theta]=[sqrt(px^2+py^2); atan2(py,px)] + eps
%  Both an EKF and a UKF run on the SAME truth and the SAME measurements.
%  Output: ../images/ekf_ukf_radar.gif , ../images/ekf_ukf_radar.png
%          and prints the RMSE comparison used in the table slide.
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(7);

%% ---- parameters ----
T=1.0; N=40;
sig_r=2.5; sig_th=deg2rad(2.0); R=diag([sig_r^2, sig_th^2]);
q_v=0.25^2; q_Om=deg2rad(1.2)^2; Q=diag([1e-4,1e-4,q_v,1e-6,q_Om]);

%% ---- ground truth (compact curvy path) ----
x=zeros(5,N); x(:,1)=[-150;26;7.5;deg2rad(-6.8);deg2rad(0.1)];   % nearly straight line passing ~8 m from the radar
for k=1:N-1
    % (no manoeuvre: keep the close straight pass)
    x(:,k+1)=f_ct(x(:,k),T)+[0;0;sqrt(q_v)*randn;0;sqrt(q_Om)*randn];
end

%% ---- radar measurements (shared) ----
y=zeros(2,N);
for k=1:N, y(:,k)=h_radar(x(:,k))+[sig_r;sig_th].*randn(2,1); end
mx=y(1,:).*cos(y(2,:)); my=y(1,:).*sin(y(2,:));   % measurements in Cartesian (display)
fprintf('closest approach to radar: %.1f m\n', min(vecnorm(x(1:2,:))));

%% ---- shared initialisation ----
x0=x(:,1)+[12;-12;1.5;deg2rad(15);deg2rad(1.5)];
P0=diag([18^2,18^2,3^2,deg2rad(25)^2,deg2rad(4)^2]);

%% ---- EKF ----
xe=zeros(5,N); Pe=cell(1,N); xe(:,1)=x0; P=P0; Pe{1}=P; I5=eye(5);
for k=1:N-1
    A=jac_f(xe(:,k),T); xp=f_ct(xe(:,k),T); Pp=A*P*A'+Q;
    H=jac_h(xp); yp=h_radar(xp); inn=y(:,k+1)-yp; inn(2)=wrapToPi(inn(2));
    S=H*Pp*H'+R; Kk=Pp*H'/S; xe(:,k+1)=xp+Kk*inn; xe(4,k+1)=wrapToPi(xe(4,k+1));
    P=(I5-Kk*H)*Pp; Pe{k+1}=P;
end

%% ---- UKF ----
n=5; kappa=1; c=sqrt(n+kappa);
wm=[kappa/(n+kappa), repmat(1/(2*(n+kappa)),1,2*n)]; wc=wm;
xu=zeros(5,N); Pu=cell(1,N); xu(:,1)=x0; P=P0; Pu{1}=P;
for k=1:N-1
    % sigma points
    Ls=chol(P,'lower'); X=[xu(:,k), xu(:,k)+c*Ls, xu(:,k)-c*Ls];
    % predict
    Xp=zeros(5,2*n+1); for i=1:2*n+1, Xp(:,i)=f_ct(X(:,i),T); end
    mpred=stateMean(Xp,wm);
    dX=Xp-mpred; dX(4,:)=wrapToPi(dX(4,:));
    Ppred=(wc.*dX)*dX'+Q;
    % measurement sigma points
    Yp=zeros(2,2*n+1); for i=1:2*n+1, Yp(:,i)=h_radar(Xp(:,i)); end
    yhat=measMean(Yp,wm);
    dY=Yp-yhat; dY(2,:)=wrapToPi(dY(2,:));
    Syy=(wc.*dY)*dY'+R;
    Pxy=(wc.*dX)*dY';
    Kk=Pxy/Syy; inn=y(:,k+1)-yhat; inn(2)=wrapToPi(inn(2));
    xu(:,k+1)=mpred+Kk*inn; xu(4,k+1)=wrapToPi(xu(4,k+1));
    P=Ppred-Kk*Syy*Kk'; P=(P+P')/2; Pu{k+1}=P;
end

%% ---- RMSE ----
rmse=@(a,b) sqrt(mean(sum((a-b).^2,1)));
posE=rmse(xe(1:2,:),x(1:2,:));  posU=rmse(xu(1:2,:),x(1:2,:));
posRaw=sqrt(mean((mx-x(1,:)).^2+(my-x(2,:)).^2));
spdE=sqrt(mean((xe(3,:)-x(3,:)).^2)); spdU=sqrt(mean((xu(3,:)-x(3,:)).^2));
hdgE=sqrt(mean(wrapToPi(xe(4,:)-x(4,:)).^2)); hdgU=sqrt(mean(wrapToPi(xu(4,:)-x(4,:)).^2));
fprintf('==== RMSE (over %d steps) ====\n',N);
fprintf('Position [m]     :  raw radar = %.2f | EKF = %.2f | UKF = %.2f\n',posRaw,posE,posU);
fprintf('Speed    [m/s]   :                    EKF = %.2f | UKF = %.2f\n',spdE,spdU);
fprintf('Heading  [deg]   :                    EKF = %.2f | UKF = %.2f\n',rad2deg(hdgE),rad2deg(hdgU));

%% ---- animation ----
here=fileparts(mfilename('fullpath'));
gifOut=fullfile(here,'..','images','ekf_ukf_radar.gif');
pngOut=fullfile(here,'..','images','ekf_ukf_radar.png');
co=get(groot,'defaultAxesColorOrder');
cTrue=[0.12 0.12 0.14]; cEKF=co(2,:); cUKF=co(4,:); cMeas=[0.62 0.62 0.66]; cK=[0.16 0.17 0.20]; cRot=[0.20 0.50 0.90];
allx=[x(1,:) mx 0]; ally=[x(2,:) my 0];
cxc=(min(allx)+max(allx))/2; cyc=(min(ally)+max(ally))/2;
half=max(range(allx),range(ally))/2+25; LIM=[cxc-half cxc+half cyc-half cyc+half]; hs=half*0.05;

fig=figure('Color','w','Position',[100 100 820 760]); first=true;
for k=1:N
    clf; ax=axes(fig); hold(ax,'on'); axis(ax,'equal'); axis(ax,LIM); grid(ax,'on'); box(ax,'on');
    for rr=30:30:2*half, th=linspace(0,2*pi,100); plot(ax,rr*cos(th),rr*sin(th),'-','Color',[0.92 0.92 0.94]); end
    draw_radar(ax,cK,hs);
    plot(ax,[0 x(1,k)],[0 x(2,k)],'-','Color',[0.5 0.75 0.5]);
    plot(ax,mx(1:k),my(1:k),'.','Color',cMeas,'MarkerSize',8);
    plot(ax,x(1,1:k),x(2,1:k),'-','Color',cTrue,'LineWidth',2.0);
    plot(ax,xe(1,1:k),xe(2,1:k),'-','Color',cEKF,'LineWidth',2.0);
    plot(ax,xu(1,1:k),xu(2,1:k),'-','Color',cUKF,'LineWidth',2.0);
    draw_ellipse(ax,xe(1:2,k),Pe{k}(1:2,1:2),2,cEKF);
    draw_ellipse(ax,xu(1:2,k),Pu{k}(1:2,1:2),2,cUKF);
    draw_hexa(ax,x(1:2,k),x(4,k),hs,cK,cRot);
    hT=plot(ax,nan,nan,'-','Color',cTrue,'LineWidth',2);
    hE=plot(ax,nan,nan,'-','Color',cEKF,'LineWidth',2);
    hU=plot(ax,nan,nan,'-','Color',cUKF,'LineWidth',2);
    hM=plot(ax,nan,nan,'.','Color',cMeas,'MarkerSize',12);
    legend(ax,[hT hM hE hU],{'true','radar meas.','EKF','UKF'},'Location','northoutside','Orientation','horizontal','FontSize',11,'Box','off');
    title(ax,sprintf('EKF vs UKF radar tracking   |   t = %2.0f s   |   pos-RMSE:  EKF %.1f m,  UKF %.1f m',(k-1)*T,posE,posU),'FontSize',12);
    xlabel(ax,'p_x  [m]'); ylabel(ax,'p_y  [m]');
    drawnow; fr=getframe(fig); [A,cm]=rgb2ind(frame2im(fr),256);
    if first, imwrite(A,cm,gifOut,'gif','LoopCount',Inf,'DelayTime',0.09); first=false;
    else,     imwrite(A,cm,gifOut,'gif','WriteMode','append','DelayTime',0.09); end
end
try, exportgraphics(fig,pngOut,'Resolution',130); catch, saveas(fig,pngOut); end
fprintf('Saved GIF -> %s\nSaved PNG -> %s\n',gifOut,pngOut);

%% ================= local functions =================
function xn=f_ct(x,T)
    px=x(1);py=x(2);v=x(3);ph=x(4);Om=x(5);
    if abs(Om)<1e-6, px2=px+v*T*cos(ph); py2=py+v*T*sin(ph);
    else, px2=px+(v/Om)*(sin(ph+Om*T)-sin(ph)); py2=py+(v/Om)*(-cos(ph+Om*T)+cos(ph)); end
    xn=[px2;py2;v;ph+Om*T;Om];
end
function A=jac_f(x,T)
    m=numel(x);A=zeros(m);h=1e-6;
    for j=1:m, d=zeros(m,1);d(j)=h; A(:,j)=(f_ct(x+d,T)-f_ct(x-d,T))/(2*h); end
end
function yv=h_radar(x), yv=[hypot(x(1),x(2)); atan2(x(2),x(1))]; end
function H=jac_h(x)
    px=x(1);py=x(2);r2=px^2+py^2;r=sqrt(r2);
    H=[px/r py/r 0 0 0; -py/r2 px/r2 0 0 0];
end
function m=stateMean(X,w)
    m=X*w'; m(4)=atan2(sum(w.*sin(X(4,:))), sum(w.*cos(X(4,:))));
end
function m=measMean(Y,w)
    m=[sum(w.*Y(1,:)); atan2(sum(w.*sin(Y(2,:))), sum(w.*cos(Y(2,:))))];
end
function a=wrapToPi(a), a=mod(a+pi,2*pi)-pi; end
function draw_ellipse(ax,mu,C,ns,col)
    C=(C+C')/2;[V,D]=eig(C);D=max(D,0);t=linspace(0,2*pi,80);e=V*sqrt(D)*[cos(t);sin(t)]*ns;
    plot(ax,mu(1)+e(1,:),mu(2)+e(2,:),'-','Color',col,'LineWidth',1.6);
end
function draw_radar(ax,c,s)
    plot(ax,0,0,'^','MarkerSize',12,'MarkerFaceColor',c,'MarkerEdgeColor',c);
    plot(ax,[0 0],[0 s],'-','Color',c,'LineWidth',1.5);
    th=linspace(-pi/3,pi/3,20); plot(ax,0.85*s*sin(th),s+0.42*s*(cos(th)-1),'-','Color',c,'LineWidth',1.5);
    text(ax,0.5*s,-s,'RADAR','Color',c,'FontSize',9,'FontWeight','bold');
end
function draw_hexa(ax,c,hd,s,cArm,cRot)
    c=c(:);
    for a=0:60:300
        ang=deg2rad(a)+hd; d=c+s*[cos(ang);sin(ang)];
        plot(ax,[c(1) d(1)],[c(2) d(2)],'-','Color',cArm,'LineWidth',2);
        th=linspace(0,2*pi,18); patch(ax,d(1)+0.42*s*cos(th),d(2)+0.42*s*sin(th),cRot,'FaceAlpha',0.35,'EdgeColor',cRot,'LineWidth',1);
    end
    th=linspace(0,2*pi,18); patch(ax,c(1)+0.34*s*cos(th),c(2)+0.34*s*sin(th),cArm,'EdgeColor',cArm);
end
