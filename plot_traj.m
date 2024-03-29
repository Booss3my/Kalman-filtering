function[]= plot_traj(x_theo,y_theo,x_bruite,y_bruite,x_filtre,y_filtre)

subplot(2,2,1);
plot(x_theo,y_theo);
title('Trajectoire théorique')
subplot(2,2,2);
plot(x_bruite,y_bruite);
title('Trajectoire bruitée')
subplot(2,2,3);
plot(x_filtre,y_filtre);
title('Trajectoire estimée')
subplot(2,2,4);
plot(x_theo,y_theo,'DisplayName','Trajectoire théorique');
hold on
plot(x_bruite,y_bruite,'DisplayName','Trajectoire bruitée');
hold on
plot(x_filtre,y_filtre,'DisplayName','Trajectoire estimée');
title('Superposition des trajectoires')
legend

end