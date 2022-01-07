function[z]=Bruit_gen(Cov,N)

%Cov: Matrice de Cov

R = chol(Cov);

z = R.'*randn([size(Cov,2),N]);


end