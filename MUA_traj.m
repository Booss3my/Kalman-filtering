function [out] =MUA_traj(etat_init,sigma_jerk,num_steps)
%etat init matrice [x y;Vx Vy;Ax Ay]

T=1;
Trans_mat=[1,T,T^2/2;0,1,T;0,0,1];
Cov=sigma_jerk*[T^3/6,T^2/2,T]*[T^3/6;T^2/2;T];
X_MUA=zeros([3,num_steps]); 
Y_MUA=zeros([3,num_steps]); 
X_MUA(:,1)=etat_init(:,1);%état initial
Y_MUA(:,1)=etat_init(:,2);%état initial

for i=1:num_steps-1
    %par indépendante on pourra générer les deux coordonnées séparément.
    X_MUA(:,i+1)=next_step(X_MUA(:,i),Trans_mat,Cov);
    Y_MUA(:,i+1)=next_step(Y_MUA(:,i),Trans_mat,Cov);    
end

out=[X_MUA(1,:);Y_MUA(1,:)];%sortie
end