clear
close all
Cov=[1,0.5;0.5,1]; %choisie definie positive
Moy=[0;0];
T=1;
N=20;

fig_MRU=0; %Vitesse MRU 
sigma_acc=10;

fig_MUA=0; %Acceleration MUA 
sigma_jerk=100;
%% verification génération du bruit
R=zeros(size(Cov));
for i=1:1000
    w=Bruit_gen(Cov,1);
    R=R+w*w.';
end
cov_est=R/1000; %doit converger vers la matrice de cov

%% MRU

Trans_mat=[1,T;0,1];
Cov=sigma_acc*[T^4/4,T^3/2;T^3/2,T^2];
vitesse=10; 

%Initialisation
X_MRU=zeros([2,N]);
X_MRU(2,1)=vitesse;
%
for i=1:N
    X_MRU(:,i+1)=next_step(X_MRU(:,i),Trans_mat,Cov); 
end
%plot de vitesse MRU
 if fig_MRU==1
    plot(X_MRU(1,:))
 end

%% MUA

Trans_mat=[1,T,T^2/2;0,1,T;0,0,1];
Cov=sigma_jerk*[T^3/6,T^2/2,T]*[T^3/6;T^2/2;T];
acceleration_in=10;
 
%état initial
X_MUA=zeros([3,N]); 
X_MUA(3,1)=acceleration_in;
%
for i=1:N
    X_MUA(:,i+1)=next_step(X_MUA(:,i),Trans_mat,Cov); 
end
 %plot 
 if fig_MUA==1
    plot(X_MUA(2,:))
 end
 
 %% Trajectoires
 Vit_MRU=[20,10]; %[Vx,Vy]
 Acc_MUA=[1,0.3]; %[Ax,Ay]
 
 init_MRU=zeros(2);
 init_MRU(2,:)=Vit_MRU;
 init_MUA=zeros(3,2);
 init_MUA(3,:)=Acc_MUA;
 
 nb_realisations=1;
 figure 
 for i=1:nb_realisations
     traj_MRU=MRU_traj(init_MRU,sigma_acc,500);
     plot(traj_MRU(1,:),traj_MRU(2,:))
     hold on 
 end
 
  figure 
 for i=1:nb_realisations
     traj_MUA=MUA_traj(init_MUA,sigma_jerk,500);
     plot(traj_MUA(1,:),traj_MUA(2,:))
     hold on 
 end

%% succession de MUA MRU
premier_type=0; %MRU
N=[300,200,300];
etat_init=[0,0;1,2;1,1]
traj=succ_MRU_MUA(etat_init,premier_type,sigma_acc,sigma_jerk,N);

plot(traj(1,1:300),traj(2,1:300),'r')
hold on
plot(traj(1,301:501),traj(2,301:501),'g')
hold on
plot(traj(1,502:end),traj(2,502:end),'b')

%% Kalman estimation

traj_MRU=MRU_traj(init_MRU,sigma_acc,500);


