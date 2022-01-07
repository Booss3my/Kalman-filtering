function [out] =MRU_traj(etat_init,sigma_acc,num_steps)
% etat init matrice [x y;Vx Vy]

T=1;
Trans_mat=[1,T;0,1];
Cov=sigma_acc*[T^4/4,T^3/2;T^3/2,T^2];
X_MRU=zeros([2,num_steps]); 
Y_MRU=zeros([2,num_steps]); 
X_MRU(:,1)=etat_init(:,1);%état initial
Y_MRU(:,1)=etat_init(:,2);%état initial

for i=1:num_steps-1
    %par indépendante on pourra générer les deux coordonnées séparément.
    X_MRU(:,i+1)=next_step(X_MRU(:,i),Trans_mat,Cov);
    Y_MRU(:,i+1)=next_step(Y_MRU(:,i),Trans_mat,Cov); 
    out=[X_MRU(1,:);Y_MRU(1,:)];%sortie
end


end