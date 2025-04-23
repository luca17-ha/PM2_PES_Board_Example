clc, clear variables
%%

% default values, adjust these
Kp = 2.0;
Kp_nl = 17.0;

% error in radians (-30, 30) * pi/180
e = (-1:0.02:1).' * 30 * pi/180;

% control signal
u_lin = Kp * e;
u_nl = Kp_nl * e.^2 .* sign(e);
u = u_lin + u_nl;

figure(1)
plot(e, [u_lin, u_nl, u]), grid on
xlabel('Error')
ylabel('Controller Output')
axis([min(e) max(e) min(u) max(u)])
set(findall(gcf, 'type', 'line'), 'linewidth', 2.0)
