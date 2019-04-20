tspan =[0 6/600];
y0=0;
[t,y] = ode45(@(t,y)2*pi*600*(-y^3+sin(2*pi*600*t)),tspan,y0);
plot(t,sin(2*pi*600*t));
hold on;
plot(t,y)
xlabel('Time t');
ylabel('Solution y');
legend('forcing function,u','state variable,y')
title('$$\frac{dy}{dt} = 2\pi 600 \big[-y^3 +sin(2\pi 600t)\big]\hspace{20pt} y(0)=0$$','interpreter','latex')
hold off;