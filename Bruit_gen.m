function[z]=Bruit_gen(Cov,N)

%Cov: Matrice de Cov

%probleme: chol trouve un probleme plus les valeurs contenues dans Cov sont proches de 0
%On normalise et on multiplie par 10000


z=zeros(size(Cov,2),N);

if norm(Cov)~=0
   
    R=chol(Cov+1e-13*eye(size(Cov))).';  
    z = R*randn([size(Cov,2),N]);
    
end

end