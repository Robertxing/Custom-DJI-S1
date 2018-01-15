
%Aerial Robotics. Wk1

% z = 4;
% r = z > 3
% 
% v = [0.001;0.5;6.4;10.5;0.002;7.1;-4.8]
% v < 0.005 %creates logical array of 1 and 0
% 
% y(1) = 1
% for n = 1:6
%     y(n+1) = y(n) - 0.1*y(n)
% end
% 
% if x < 0 %true/false
%     f = 1-x
% end

%advanced content

month = [1,2,3,4,5,6,7,8,9,10,11,12];
avgTemp1 = [5.24,5.20,6.92,9.31,12.26,14.8,16.51,16.93,14.91,12.27,8.73,5.91];
avgTemp2 = [2.24,33.20,6.92,7.31,12.26,14.14,16.51,16.93,14.91,12.27,8.73,9.91];

plot(month,avgTemp1,'r:o')
hold on
plot(month,avgTemp2,'b:o')

%solving systems of linear equations

% 1.5x1 + x2 = 3
% x3 = 4x2
% 4 - x1 + x2 = x3

%rearranging
% 1.5x1 + x2 + 0x3 = 3
% 0x1 + -4x2 + 1x3 = 0
% -x1 + x2 - x3 = -4

A = [1.5,1,0;0,-4,1;-1,1,-1];
b = [3;0;-4];
%x = A\b

syms a b c x
%symobolic variables. declare existence and use in calcs.
y = a*x^2 + b*x + c;

soln = solve(y==0,x)
ysoln = subs(soln, [a,b,c], [3,2,-6]);
ysoln = simplify(ysoln)
d = vpa(ysoln,9) %variable precision arithmetic

%Solving ODEs

%[tSol,ySol] = ode45(deriveFunction,interval of ind.variable,initialVaLUE)

r = 1;
K = 1000;
odefun = @(t,P) r*P*(1-P/K); %independent then dependent
tSpan = [0,10];
p0 = 20;
[tSol,pSol] = ode45(odefun,tSpan,p0);

dxdt = @(t,x) [x(2); -x(2)-x(1)];
tLim = [0,10];
x0 = [0,1]; %initial conditions
[tSol,xSol] = ode45(dxdt,tLim,x0);
%times when numerical solutions was computed. matrix solution
%containing x1 and x2
%Higher order ODE -> first order ODE with new derivatives ->
%first order vectored ODE






