tspan =[0 6/600];
y0=[0 0];
[t,y] = ode45(@(t,y)2*pi*600*[-2*y(1)-y(2)+sin(2*pi*600*t); y(1)],tspan,y0);
plot(t,sin(2*pi*600*t));
hold on;
plot(t,y)
xlabel('Time t');
ylabel('Currents and Voltages);
legend('V','i_L','v_C')
title('$$\frac{di_L}{dt} = 2\pi 600 \big[sin(2\pi 600t)-2i_L-v_C\big]\hspace{20pt} i_L(0)=0 \hspace{40pt}\frac{dv_C}{dt} = 2\pi 600i_L\hspace{20pt}v_C(0)=0$$','interpreter','latex')
hold off;