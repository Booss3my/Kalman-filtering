clear;


%% filtrage/lissage Kalman MRU
%%%%
plot=0;
%%%%


sigma_acc=0.001;
var_mesure=100;
err_quad_moy=0;
n_real=50;
vit_init=[1,2];
etat_init=zeros(2);
etat_init(2,:)=vit_init;
t=700;
T=1;
Trans_mat_MRU=[1,T;0,1];

Cov=sigma_acc*[T^4/4,T^3/2;T^3/2,T^2];


for i=1:n_real
    i
    traj_theo_MRU=MRU_traj(etat_init,0,t);
    
    
    traj_MRU=MRU_traj(etat_init,sigma_acc,t);


    x_bruite_MRU=traj_MRU(1,:)+sqrt(var_mesure)*randn(1,t);
    y_bruite_MRU=traj_MRU(2,:)+sqrt(var_mesure)*randn(1,t);

    est_x=kalman(Trans_mat_MRU,eye(2),[1,0],Cov,10,x_bruite_MRU,zeros(2,1),zeros(2),t-1,1);
    est_y=kalman(Trans_mat_MRU,eye(2),[1,0],Cov,10,y_bruite_MRU,zeros(2,1),zeros(2),t-1,1);
    
    if i==1
        figure;
        plot_traj(traj_theo_MRU(1,:),traj_theo_MRU(2,:),x_bruite_MRU,y_bruite_MRU,est_x(2:end,1),est_y(2:end,1))
    end
    
    %pour faciliter l'ecriture
    x_est=est_x(:,1).';
    y_est=est_y(:,1).';
    x_theo=traj_MRU(1,2:end);
    y_theo=traj_MRU(2,2:end);
    
    err_quad_moy=err_quad_moy+((x_theo-x_est).^2+(y_theo-y_est).^2)./(x_theo.^2+y_theo.^2);
end
err_quad_moy=err_quad_moy/n_real;

if plot==1
    semilogy(err_quad_moy)
    hold on
end

%% Robustesse aux variations de parametres // MUA

%%%%
plot=1;
%%%%
T=1;
sigma_jerk=10;
var_mesure=10;
Trans_mat_MUA=[1,T,T^2/2;0,1,T;0,0,1];
Cov=sigma_jerk*[T^3/6;T^2/2;T]*[T^3/6,T^2/2,T];
err_quad_moy=0;
t=500;
flag_lissage=1;

Acc_init=[1,1];
etat_init=zeros(3,2);
etat_init(3,:)=Acc_init;
n_real=50;
err_quad_moy=0;
%err_moy_y=0;
for i=1:n_real
    i
    traj_theo_MUA=MUA_traj(etat_init,0,t);
    traj_MUA=MUA_traj(etat_init,sigma_jerk,t);


    x_bruite=traj_MUA(1,:)+sqrt(var_mesure)*randn(1,t);
    y_bruite=traj_MUA(2,:)+sqrt(var_mesure)*randn(1,t);

    est_x=kalman(Trans_mat_MUA,eye(3),[1,0,0],Cov,1000,x_bruite,[x_bruite(1);0;0],zeros(3),t-1,flag_lissage);
    est_y=kalman(Trans_mat_MUA,eye(3),[1,0,0],Cov,1000,y_bruite,[y_bruite(1);0;0],zeros(3),t-1,flag_lissage);

    
    %pour faciliter l'ecriture
    x_est=est_x(:,1).';
    y_est=est_y(:,1).';
    x_theo=traj_MUA(1,2:end);
    y_theo=traj_MUA(2,2:end);
    err_quad_moy=err_quad_moy+((x_theo-x_est).^2+(y_theo-y_est).^2)./(x_theo.^2+y_theo.^2);
    
end

err_quad_moy=err_quad_moy/n_real;
%err_moy_y=err_moy_y/n_real;;
%err_tot_moy=(err_moy_x+err_moy_y)/2;

if plot==1
    semilogy(err_quad_moy)
    hold on
end

%% Sensibilité à l'erreur de modèle
    err_quad_moy=0;
Cov=sigma_acc*[T^4/4,T^3/2;T^3/2,T^2];
  est_x=kalman(Trans_mat_MRU,eye(2),[1,0],Cov,10,x_bruite_MRU,zeros(2,1),zeros(2),t-1,1);
  est_y=kalman(Trans_mat_MRU,eye(2),[1,0],Cov,10,y_bruite_MRU,zeros(2,1),zeros(2),t-1,1);
  %pour faciliter l'ecriture
  x_est=est_x(:,1).';
  y_est=est_y(:,1).';
  x_theo=traj_MUA(1,2:end);
  y_theo=traj_MUA(2,2:end);
  err_quad_moy=err_quad_moy+((x_theo-x_est).^2+(y_theo-y_est).^2)./(x_theo.^2+y_theo.^2);

semilogy(err_quad_moy)
hold on

