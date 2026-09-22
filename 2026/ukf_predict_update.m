%% UKF — didactic 2D animation:  the PREDICT / UPDATE cycle repeated (6 steps)
%  SEL5917 — The Unscented Kalman Filter
%  The state x=[x1;x2] is the position of a hexacopter drone that flies a curved
%  path. Each cycle: build sigma points -> propagate through f -> predicted
%  mean/cov (+Q) -> noisy measurement -> update (posterior shown at once).
%  Only the 2n+1 = 5 sigma points are propagated (no Monte-Carlo cloud).
%  Output: ../images/ukf_predict_update.gif
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(11);

%% ---- UKF weights & symmetric set (n=2) ----
n=2; kappa=1; c=sqrt(n+kappa);
S=[0 1 -1 0 0; 0 0 0 1 -1];
wm=[kappa/(n+kappa), repmat(1/(2*(n+kappa)),1,2*n)];  wc=wm;

%% ---- dynamics: swirl about a pivot (curved flight) ----
pv=[0;0]; thStep=0.52; beta=0.32; rref=3.5;
fstep = @(X) pv + rotmap(X-pv, thStep, beta, rref);
fpart = @(X,frac) pv + rotmap(X-pv, frac*thStep, beta, rref);

%% ---- noises ----
Q=0.02*eye(2);  R=0.30*eye(2);

%% ---- pre-compute the whole run ----
K=6;
Xt=zeros(2,K+1); Xt(:,1)=[3.5;0];              % true state (time 0..K)
for k=1:K, Xt(:,k+1)=fstep(Xt(:,k)); end
mest=zeros(2,K+1); Pest=cell(1,K+1);
mest(:,1)=[3.1;0.7]; Pest{1}=[0.8 0.2;0.2 0.6];   % initial estimate (with error)
sigS=cell(1,K); Xpr=cell(1,K); mp=zeros(2,K); Pp=cell(1,K); meas=zeros(2,K);
for k=1:K
    Lk=chol(Pest{k},'lower'); sigS{k}=mest(:,k)+c*Lk*S;
    Xp=fstep(sigS{k}); Xpr{k}=Xp;
    mpk=Xp*wm'; dp=Xp-mpk; Ppk=(wc.*dp)*dp'+Q; mp(:,k)=mpk; Pp{k}=Ppk;
    yk=Xt(:,k+1)+chol(R,'lower')*randn(2,1); meas(:,k)=yk;
    Kk=Ppk/(Ppk+R); mest(:,k+1)=mpk+Kk*(yk-mpk); Pest{k+1}=(eye(2)-Kk)*Ppk;
end

%% ---- limits & colors ----
allP=[Xt, meas, mest, cat(2,sigS{:}), cat(2,Xpr{:})];
cx=mean([min(allP(1,:)) max(allP(1,:))]); cy=mean([min(allP(2,:)) max(allP(2,:))]);
half=max(range(allP(1,:)),range(allP(2,:)))/2 + 1.0; LIM=[cx-half cx+half cy-half cy+half]; hs=half*0.05;
co=get(groot,'defaultAxesColorOrder');
cPri=co(1,:); cSig=co(2,:); cMeas=co(5,:); cPost=co(4,:); edge=[0.4 0.4 0.4];
trailC=[0.96 0.78 0.55]; cArm=[0.15 0.15 0.18]; cRot=[0.20 0.50 0.90]; faint=[0.80 0.80 0.83];

fig=figure('Color','w','Position',[100 100 800 730]); F={};
Nsub=12;

%% ---- initial prior (before cycle 1) ----
for j=1:5
    setupAx(LIM);
    draw_ellipse(mest(:,1),Pest{1},2,cPri,'-'); plot(mest(1,1),mest(2,1),'o','MarkerFaceColor',cPri,'MarkerEdgeColor','k','MarkerSize',8);
    for i=1:5, sigmaPt(sigS{1}(:,i),cSig); end
    draw_hexa(Xt(:,1),0,hs,cArm,cRot); text(Xt(1,1)+1.3*hs,Xt(2,1)-1.4*hs,'true state','FontSize',10,'Color',cArm);
    banner('Initial prior', 'N(x_{0|0}, P_{0|0}) and its sigma points', cPri);
    F{end+1}=snap(fig);
end

%% ================= repeat the cycle K times =================
for k=1:K
    p85=fpart(Xt(:,k),0.85); hdE=atan2(Xt(2,k+1)-p85(2), Xt(1,k+1)-p85(1));
    % ---- (a) PREDICT: propagate sigma points; drone flies this segment ----
    for j=1:Nsub
        frac=j/Nsub; setupAx(LIM);
        history(Xt,mest,Pest,meas,k,k,k-1,k-1,cArm,cPost,cMeas,faint);
        draw_ellipse(mest(:,k),Pest{k},2,cPri,'--');
        sg=fpart(sigS{k},frac);
        for i=1:5, plot([sigS{k}(1,i) sg(1,i)],[sigS{k}(2,i) sg(2,i)],'-','Color',trailC,'LineWidth',1.2); end
        for i=1:5, sigmaPt(sg(:,i),cSig); end
        dp=fpart(Xt(:,k),frac); hd=atan2(dp(2)-Xt(2,k),dp(1)-Xt(1,k));
        plot([Xt(1,k) dp(1)],[Xt(2,k) dp(2)],'-','Color',cArm,'LineWidth',1.6);
        draw_hexa(dp,hd,hs,cArm,cRot);
        banner(sprintf('Cycle %d/%d  —  Predict',k,K), 'propagate the 5 sigma points through f  (drone flies its path)', cSig);
        F{end+1}=snap(fig);
    end
    % ---- (b) predicted mean & covariance ----
    for j=1:3
        setupAx(LIM);
        history(Xt,mest,Pest,meas,k+1,k,k-1,k-1,cArm,cPost,cMeas,faint);
        for i=1:5, sigmaPt(Xpr{k}(:,i),cSig); end
        draw_ellipse(mp(:,k),Pp{k},2,cPri,'-'); plot(mp(1,k),mp(2,k),'o','MarkerFaceColor',cPri,'MarkerEdgeColor','k','MarkerSize',8);
        draw_hexa(Xt(:,k+1),hdE,hs,cArm,cRot);
        banner(sprintf('Cycle %d/%d  —  Prediction',k,K), 'weighted mean & covariance (+Q):  N(x_{k+1|k}, P_{k+1|k})', cPri);
        F{end+1}=snap(fig);
    end
    % ---- (c) measurement ----
    for j=1:3
        setupAx(LIM);
        history(Xt,mest,Pest,meas,k+1,k,k-1,k-1,cArm,cPost,cMeas,faint);
        for i=1:5, sigmaPt(Xpr{k}(:,i),cSig); end
        draw_ellipse(mp(:,k),Pp{k},2,cPri,'-');
        draw_ellipse(meas(:,k),R,2,cMeas,'--'); plot(meas(1,k),meas(2,k),'x','Color',cMeas,'MarkerSize',14,'LineWidth',3);
        draw_hexa(Xt(:,k+1),hdE,hs,cArm,cRot);
        banner(sprintf('Cycle %d/%d  —  Measurement',k,K), 'noisy measurement  y_k  of the drone position  (cov. R)', cMeas);
        F{end+1}=snap(fig);
    end
    % ---- (d) UPDATE: posterior appears at once ----
    for j=1:5
        setupAx(LIM);
        history(Xt,mest,Pest,meas,k+1,k,k,k,cArm,cPost,cMeas,faint);
        draw_ellipse(mp(:,k),Pp{k},2,cPri,'--');
        draw_ellipse(meas(:,k),R,2,cMeas,'--'); plot(meas(1,k),meas(2,k),'x','Color',cMeas,'MarkerSize',14,'LineWidth',3);
        plot([mp(1,k) mest(1,k+1)],[mp(2,k) mest(2,k+1)],'-','Color',edge);
        draw_ellipse(mest(:,k+1),Pest{k+1},2,cPost,'-'); plot(mest(1,k+1),mest(2,k+1),'o','MarkerFaceColor',cPost,'MarkerEdgeColor','k','MarkerSize',9);
        draw_hexa(Xt(:,k+1),hdE,hs,cArm,cRot);
        banner(sprintf('Cycle %d/%d  —  Update',k,K), 'K=P_{xy}S^{-1};  posterior N(x_{k+1|k+1}, P_{k+1|k+1})  becomes the next prior', cPost);
        F{end+1}=snap(fig);
    end
end

%% ---- final hold: whole run ----
for j=1:12
    setupAx(LIM);
    history(Xt,mest,Pest,meas,K+1,K+1,K,K,cArm,cPost,cMeas,faint);
    draw_hexa(Xt(:,K+1),0,hs,cArm,cRot);
    text(LIM(1)+0.2,LIM(4)-0.5, '\color[rgb]{0.15,0.15,0.18}true path   \color[rgb]{0.494,0.184,0.556}estimate   \color[rgb]{0.466,0.674,0.188}measurements','FontSize',11,'FontWeight','bold');
    banner('Recursive UKF  —  6 cycles', 'predict \rightarrow update, repeated:  the estimate tracks the drone', cPost);
    F{end+1}=snap(fig);
end

%% ---- write GIF ----
gifOut=fullfile(fileparts(mfilename('fullpath')),'..','images','ukf_predict_update.gif');
for i=1:numel(F)
    [A,cm]=rgb2ind(F{i},256);
    if i==1, imwrite(A,cm,gifOut,'gif','LoopCount',Inf,'DelayTime',0.075);
    else,    imwrite(A,cm,gifOut,'gif','WriteMode','append','DelayTime',0.075); end
end
fprintf('Wrote %d frames -> %s\n', numel(F), gifOut);

%% ================= local functions =================
function im = snap(fig), drawnow; im = frame2im(getframe(fig)); end
function setupAx(LIM)
    cla; hold on; box on; grid on; axis equal; axis(LIM);
    set(gca,'FontSize',11); xlabel('x_1'); ylabel('x_2');
end
function banner(step,txt,col)
    title({['\bf' step],['\rm\fontsize{12}' txt]},'Color',col,'FontSize',15);
end
function sigmaPt(p,col)
    plot(p(1),p(2),'d','MarkerFaceColor',col,'MarkerEdgeColor','k','MarkerSize',10);
end
function history(Xt,mest,Pest,meas,nTrue,nEst,nMeas,nEll,cTrue,cEst,cMeas,faint)
    if nEll>=1
        for j=1:nEll, draw_ellipse(mest(:,j+1),Pest{j+1},2,faint,'-'); end
    end
    if nTrue>=2, plot(Xt(1,1:nTrue),Xt(2,1:nTrue),'-','Color',cTrue,'LineWidth',1.3); end
    if nEst>=2,  plot(mest(1,1:nEst),mest(2,1:nEst),'-','Color',cEst,'LineWidth',1.8); end
    if nEst>=1,  plot(mest(1,1:nEst),mest(2,1:nEst),'o','MarkerFaceColor',cEst,'MarkerEdgeColor','none','MarkerSize',3); end
    if nMeas>=1, plot(meas(1,1:nMeas),meas(2,1:nMeas),'.','Color',cMeas,'MarkerSize',12); end
end
function Y = rotmap(U, ang, beta, rref)
    r = sqrt(sum(U.^2,1));  a = ang.*(1 + beta*(r-rref));
    Y = [cos(a).*U(1,:) - sin(a).*U(2,:);  sin(a).*U(1,:) + cos(a).*U(2,:)];
end
function draw_hexa(c,hd,s,cArm,cRot)
    c=c(:);
    for a=0:60:300
        ang=deg2rad(a)+hd; d=c+s*[cos(ang);sin(ang)];
        plot([c(1) d(1)],[c(2) d(2)],'-','Color',cArm,'LineWidth',2);
        th=linspace(0,2*pi,18);
        patch(d(1)+0.42*s*cos(th), d(2)+0.42*s*sin(th), cRot,'FaceAlpha',0.35,'EdgeColor',cRot,'LineWidth',1);
    end
    th=linspace(0,2*pi,18);
    patch(c(1)+0.34*s*cos(th), c(2)+0.34*s*sin(th), cArm,'EdgeColor',cArm);
end
function draw_ellipse(mu,C,ns,col,ls)
    C=(C+C')/2; [V,D]=eig(C); D=max(D,0);
    t=linspace(0,2*pi,100); e=V*sqrt(D)*[cos(t);sin(t)]*ns;
    plot(mu(1)+e(1,:), mu(2)+e(2,:), ls,'Color',col,'LineWidth',2.2);
end
