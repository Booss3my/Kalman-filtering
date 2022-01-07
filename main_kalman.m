clear;


%% filtrage/lissage Kalman MRU

fig_MRU=0; %Vitesse MRU 
sigma_acc=0.0001;
sigma_mesure=10;
err_quad_moy=0;
n_real=1;
vit_init=[1,2];
etat_init=zeros(2);
etat_init(2,:)=vit_init;


for i=1:n_real
    traj_theo_MRU=MRU_traj(etat_init,0,500);
    
    
    traj_MRU=MRU_traj(etat_init,sigma_acc,500);
    T=1;
    Trans_mat=[1,T;0,1];
    Cov=sigma_acc*[T^4/4,T^3/2;T^3/2,T^2];

    x_bruite=traj_MRU(1,:)+sigma_mesure*randn(1,500);
    y_bruite=traj_MRU(2,:)+sigma_mesure*randn(1,500);

    est_x=kalman(Trans_mat,eye(2),[1,0],Cov,6,x_bruite,zeros(2,1),zeros(2),500,0);
    est_y=kalman(Trans_mat,eye(2),[1,0],Cov,6,y_bruite,zeros(2,1),zeros(2),500,0);
    
    if i==1
        figure;
        plot_traj(traj_theo_MRU(1,:),traj_theo_MRU(2,:),x_bruite,y_bruite,est_x(2:end,1),est_y(2:end,1))
    end
    err_quad_moy=err_quad_moy+sum(abs(est_x(2:end,1).'-traj_MRU(1,2:end)))/500;
end
err_quad_moy=err_quad_moy/n_real;


%% Robustesse aux variations de parametres // MUA

T=1;
sigma_jerk=1e-10;
sigma_mesure=100;
Trans_mat=[1,T,T^2/2;0,1,T;0,0,1];
Cov=sigma_jerk*[T^3/6,T^2/2,T]*[T^3/6;T^2/2;T];
err_quad_moy=0;

Acc_init=[0.1,0.1];
etat_init=zeros(3,2);
etat_init(3,:)=Acc_init;
n_real=1;
for i=1:n_real
    i
    traj_theo_MUA=MUA_traj(etat_init,0,500);
    traj_MUA=MUA_traj(etat_init,sigma_jerk,500);


    x_bruite=traj_MUA(1,:)+sigma_mesure*randn(1,500);
    y_bruite=traj_MUA(2,:)+sigma_mesure*randn(1,500);

    est_x=kalman(Trans_mat,eye(3),[1,0,0],Cov,100,x_bruite,zeros(3,1),zeros(3),500,0);
    est_y=kalman(Trans_mat,eye(3),[1,0,0],Cov,100,y_bruite,zeros(3,1),zeros(3),500,0);
    
    if i==1
        plot_traj(traj_theo_MUA(1,:),traj_theo_MUA(2,:),x_bruite,y_bruite,est_x(2:end,1),est_y(2:end,1))
    end
    err_quad_moy=err_quad_moy+sum(abs(est_x(2:end,1).'-traj_MUA(1,2:end)))/500;
end
err_quad_moy=err_quad_moy/n_real;



