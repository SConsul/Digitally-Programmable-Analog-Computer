tspan =[0 6/600];
y0=1;
[t,y] = ode45(@(t,y)2*pi*600*(-y+sign(sin(2*pi*600*t))),tspan,y0);
plot(t,sign(sin(2*pi*600*t)));
hold on;
plot(t,y)
xlabel('Time t');
ylabel('Solution y');
legend('forcing function,u','state variable,y')
ylim([-1.5,1.5])
title('$$\frac{dy}{dt} = 2\pi 600 \big[-y +sign(sin(2\pi 600t))\big]\hspace{20pt} y(0)=1$$','interpreter','latex')
hold off;