% Considering the Bernoulli's Leminscate Trajectory, design the Linear and
% (Almost) NonLinear Controllers for a Unicycle. Suppose that the Unicycle
% at the time t=0 starts from a random position generated by the code
% within 0.5m from the desired initial one-.

%% INIZIALIZATION
clc
clear
close all

%% PARAMETERS & CONFIGURATION

T = 4*pi/10;
r = 1;

t = 0 : 0.001 :T;

xd = (r.*cos(10.*t))./(1 + (sin(10.*t)).^2);
yd = (r.*cos(10.*t).*sin(10.*t))./(1 + (sin(10.*t)).^2);

x_zero = r;
y_zero = 0;

max_dist = 0.5; 

% Random Angle between 0 e 2*pi
beta = 2*pi*rand();

% Random Distance between 0 and max_dist
dist = max_dist*sqrt(rand());

% Random Point Coordinates
x_i = r + dist*cos(beta);
y_i = dist*sin(beta);

% theta in such a way that the Unicycle is oriented toward the Desired Initial Point
% In this way at the beginning omega has no higher peak
delta_x = r - x_i;
delta_y = 0 - y_i;
theta_i = atan2(delta_y, delta_x);

% figure;
% viscircles([r, 0], max_dist, 'LineStyle', '--');
% hold on;
% plot(r, 0, 'ro', 'MarkerSize', 10, 'DisplayName', '(r, 0)');
% plot(x_i, y_i, 'bx', 'MarkerSize', 10, 'DisplayName', 'Random Point');
% axis equal;
% legend;
% grid on;
% title('Random Points within 0.5m from (r, 0)');

q_i = [x_i, y_i, theta_i]';

% Bernoulli's Leminscate Trajectory Plot with Initial Starting Point
plot_1 = figure('Name', 'Bernoulli''s Leminscate Trajectory', 'Renderer', 'painters');
plot(xd, yd, 'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
hold on
plot(x_i, y_i, 'ko', 'MarkerFaceColor', [0.7, 0.7, 0.7], 'MarkerSize', 5)
title('Bernoulli''s Leminscate Trajectory', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$x [m]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$y [m]$', 'Interpreter', 'latex', 'FontSize', 26)
grid on
xlim([min(xd)-(max(0,(x_i-r+0.1))), max(xd)+(max(0,(x_i-r+0.1)))])
ylim([min(yd)+min(0,y_i), max(yd)+max(0,y_i)])
axis equal
legend({'Path', 'Starting Point'}, 'Interpreter', 'latex', 'Location', 'best')
exportgraphics(plot_1, 'Bernoulli_s_Leminscate_Trajectory.pdf');




%% LINEAR CONTROL

% Gains
zita = 0.5;
a = 600;
k_1 = 2*zita*a;
k_3 = k_1;

% Desired Values and Time Derivatives
dt = t(2) - t(1);

dxd = gradient(xd,dt);
dyd = gradient(yd,dt);

ddxd = gradient(dxd,dt);
ddyd = gradient(dyd,dt);

w_d = (ddyd.*dxd - ddxd.*dyd)./(dxd.^2 + dyd.^2);

x_d = timeseries(xd, t);
y_d = timeseries(yd,t);

dx_d = timeseries(dxd,t);
dy_d = timeseries(dyd,t);

ddx_d = timeseries(ddxd,t);
ddy_d = timeseries(ddyd,t);

%% (ALMOST) NONLINEAR CONTROL

% Gain
% c = 60/r^0.4;
% k_2_a = 3*c^2/r;

c = 36/r^0.3;
k_2_a = 3*c^2/r;



%% Simulink

out1 = sim("HW2_FSR_ES3_Linear.slx");
out2 = sim("HW2_FSR_ES3_Almost_NonLinear.slx");

t_out1 = out1.tout;
t_out2 = out2.tout;

e_1_lin = squeeze(out1.e_1_lin.Data);
e_2_lin = squeeze(out1.e_2_lin.Data);
e_3_lin = squeeze(out1.e_3_lin.Data);

v_lin = out1.v_lin.Data;
omega_lin = out1.omega_lin.Data;

x_lin = out1.x_lin.Data;
y_lin = out1.y_lin.Data;
theta_lin = out1.theta_lin.Data;


e_1_almost_nl = squeeze(out2.e_1_almost_nl.Data);
e_2_almost_nl = squeeze(out2.e_2_almost_nl.Data);
e_3_almost_nl = squeeze(out2.e_3_almost_nl.Data);

v_almost_nl = out2.v_almost_nl.Data;
omega_almost_nl = out2.omega_almost_nl.Data;

x_almost_nl = out2.x_almost_nl.Data;
y_almost_nl = out2.y_almost_nl.Data;
theta_almost_nl = out2.theta_almost_nl.Data;


plot_2 = figure('Name', 'Linear Control', 'Renderer', 'painters');
plot(xd, yd, 'k--', 'Color', [0.9, 0.9, 0.9], 'LineWidth', 2)
hold on
plot(x_lin, y_lin, 'Color', [0.2, 0.2, 0.2], 'LineWidth', 1.5)
plot(x_i, y_i, 'ko', 'MarkerFaceColor', [0.9, 0.9, 0.9], 'MarkerSize', 5)
title('Linear Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$x [m]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$y [m]$', 'Interpreter', 'latex', 'FontSize', 26)
grid on
xlim([min(x_lin)-(max(0,(x_i-r+0.1))), max(x_lin)+(max(0,(x_i-r+0.1)))])
ylim([min(y_lin)+min(0,y_i), max(y_lin)+max(0,y_i)])
axis equal
legend({'Bernoulli''s Leminscate Desired Trajectory', 'Bernoulli''s Leminscate Trajectory with Linear Control', 'Starting Position'}, 'Interpreter', 'latex', 'Location', 'northeast')
exportgraphics(plot_2, 'Bernoulli_Lin.pdf');

plot_3 = figure('Name', '(Almost) NonLinear Control', 'Renderer', 'painters');
plot(xd, yd, 'k--', 'Color', [0.9, 0.9, 0.9], 'LineWidth', 2)
hold on
plot(x_almost_nl, y_almost_nl, 'Color', [0.2, 0.2, 0.2], 'LineWidth', 1.5)
plot(x_i, y_i, 'ko', 'MarkerFaceColor', [0.9, 0.9, 0.9], 'MarkerSize', 5)
title('(Almost) NonLinear Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$x [m]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$y [m]$', 'Interpreter', 'latex', 'FontSize', 26)
grid on
xlim([min(x_almost_nl)-(max(0,(x_i-r+0.1))), max(x_almost_nl)+(max(0,(x_i-r+0.1)))])
ylim([min(y_almost_nl)+min(0,y_i), max(y_almost_nl)+max(0,y_i)])
axis equal
legend({'Bernoulli''s Leminscate Desired Trajectory', 'Bernoulli''s Leminscate Trajectory with (Almost) NL Control', 'Starting Position'}, 'Interpreter', 'latex', 'Location', 'northeast')
exportgraphics(plot_3, 'Bernoulli_AlmostNL.pdf');


plot_4 = figure('Name', 'Position Errors Linear Control', 'Position', [10 10 900 350]);
plot(t_out1,e_1_lin,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out1,e_2_lin,'k--','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
title('Position Errors Linear Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([-0.3, 0.3])
legend({'$e_1(t)$', '$e_2(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_4, 'Bernoulli_Position_Errors_Lin.pdf');


plot_5 = figure('Name', 'Position Errors (Almost) NL Control', 'Position', [10 10 900 350]);
plot(t_out2,e_1_almost_nl,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out2,e_2_almost_nl,'k--','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
title('Position Errors (Almost) NL Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([-0.3, 0.3])
legend({'$e_1(t)$', '$e_2(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_5, 'Bernoulli_Position_Errors_AlmostNL.pdf');



plot_6 = figure('Name', 'Heading Velocity (Almost) NL Control', 'Position', [10 10 900 350]);
plot(t_out2,v_almost_nl,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
title('Heading Velocity (Almost) NL Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$v [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min(v_almost_nl), max(v_almost_nl)])
legend({'$v(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_6, 'Bernoulli_Heading_Vel_AlmostNL.pdf');


plot_7 = figure('Name', 'Heading Velocity Linear Control', 'Position', [10 10 900 350]);
plot(t_out1,v_lin,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
title('Heading Velocity Linear Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$v [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min(v_lin), max(v_lin)])
legend({'$v(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_7, 'Bernoulli_Heading_Vel_Lin.pdf');


plot_8 = figure('Name', 'Angular Velocity (Almost) NL Control', 'Position', [10 10 900 350]);
plot(t_out2,omega_almost_nl,'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
title('Angular Velocity (Almost) NL Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\omega [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 22);
grid on
box on
xlim([0, T])
ylim([min(omega_almost_nl), max(omega_almost_nl)])
legend({'$\omega(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_8, 'Bernoulli_Angular_Vel_AlmostNL.pdf');


plot_9 = figure('Name', 'Angular Velocity Linear Control', 'Position', [10 10 900 350]);
plot(t_out1,omega_lin,'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
title('Angular Velocity Linear Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\omega [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 22);
grid on
box on
xlim([0, T])
ylim([min(omega_lin), max(omega_lin)])
legend({'$\omega(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_9, 'Bernoulli_Angular_Vel_Lin.pdf');



plot_10 = figure('Name', 'Orientation Errors Linear Control', 'Position', [10 10 900 350]);
plot(t_out1,e_3_lin,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)

title('Orientation Error Linear Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [rad]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min(e_3_lin), max(e_3_lin)])
legend({'$e_3(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_10, 'Bernoulli_Orientation_Error_Lin.pdf');


plot_11 = figure('Name', 'Orientation Error (Almost) NL Control', 'Position', [10 10 900 350]);
plot(t_out2,e_3_almost_nl,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
title('Orientation Error (Almost) NL Control', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [rad]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min(e_3_almost_nl), max(e_3_almost_nl)])
legend({'$e_3(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_11, 'Bernoulli_Orientation_Error_AlmostNL.pdf');




