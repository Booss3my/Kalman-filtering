function [out]=next_step(Etat_prec,Mat_transition,Cov_bruit)

out=Mat_transition*Etat_prec;
if Cov_bruit~=0 
    u=Bruit_gen(Cov_bruit,1);
    out=out+u;
end

end