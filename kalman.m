function[x_est]=kalman(Phi,G,H,cov_mod,cov_mes,obs,x_init,P_init,N,lissage)

%PKk denote P_{k/k-1}
%Pkk denote P_{k-1/k-1}
%cov_mod: covariance du bruit de modèle
%cov_mes: covariance du bruit de mesure(dans notre cas scalaire c'est la variance)
% G et H invariantes pendant le trajet


x_kk=x_init;
P_kk=P_init;
for i=1:N
    P_Kk=Phi*P_kk*Phi.'+G*cov_mod*G.';
    %
    P_Kk_save(:,:,i)=P_Kk; %k/k+1
    %
    K=P_Kk*H.'*inv(H*P_Kk*H.'+cov_mes);
    x_Kk=Phi*x_kk; %k+1/k
    x_kk=x_Kk+K*(obs(i+1)-H*x_Kk);
    %
    tmp=K*H;
    %
    P_kk=[eye(size(tmp))-tmp]*P_Kk;
    %
    P_kk_save(:,:,i)=P_kk;
    %
    x_est(i,:)=x_kk;
end

if lissage==1
    P(:,:,N)=P_kk_save(:,:,N);
    for i=2:N-1
        A=P_kk_save(:,:,N-i+1)*Phi.'*inv(P_Kk_save(:,:,N-i+2));
        P(:,:,N-i+1) = P_kk_save(:,:,N-i+1) + A*(P(:,:,N-i+2)-P_Kk_save(:,:,N-i+2))*A.';
        x_est(N-i+1,:)= (x_est(N-i+1,:).' + A*(x_est(N-i+2,:).'-Phi*x_est(N-i+1,:).')).';
    end

end
save("save.mat","P_Kk_save")

end