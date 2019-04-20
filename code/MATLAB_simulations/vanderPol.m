tspan =[0 20];
y0=[2; 0];
mu=1;
[t,y] = ode45(@(t,y)[y(2); mu*(1-y(1)^2)*y(2)-y(1)],tspan,y0);
plot(t,y)
legend('y_1','y_2')
xlabel('Time t');
ylabel('Solution y');
title(['Solution of van der Pol Equation (\mu = ',num2str(mu),')','  y_1(0)=',num2str(y0(1)),'  y_2(0)=',num2str(y0(2))]);
hold off;