function[out]=succ_MRU_MUA(etat_init,first_type,sigma_acc,sigma_jerk,num_steps)
    %first_type =1 si le premier mouvement est un MUA , 0 si MRU
    %num_steps: vecteur contenant les steps dans chaque phase, on en déduit
    %aussi le nombre de changement de phases
    state=etat_init;
    out=[];
    idx=first_type;
    for i=1:length(num_steps)
        if mod(idx,2)==0 %MRU
            traj=MRU_traj(state(1:2,1:2),sigma_acc,num_steps(i));
            
        end
        
        if mod(idx,2)==1 %MUA
            traj=MUA_traj(state,sigma_jerk,num_steps(i));
        end
        idx=idx+1;
        state(1,:)=traj(:,end).';
        out=[out,traj];
    end


end