% Unicycle Posture Regulator based on Polar Coordinates, with the State
% Feedback computed through the Runge-Kutta Odometric Localization Method.
% Initial Configuration q_i = [x_i y_i theta_i]' = [10 1 pi\4]'
% Final Configuration q_f = [x_f y_f theta_f]' = [0 0 0]'

%% INIZIALIZATION
clc
clear
close all

%% CONFIGURATIONS

q_i = [10 1 pi\4]';
q_f = [0 0 0]';

%% PARAMETERS

Ts = 0.01;
k1 = 1;
k2 = 2;
k3 = 1;
T = 15;
out = sim("HW2_FSR_ES4.slx");

t_out = out.tout;

rho = out.rho.Data;
gamma = out.gamma.Data;
delta = out.delta.Data;

x = out.x.Data;
y = out.y.Data;
theta = out.theta.Data;

v = out.v.Data;
w = out.w.Data;

plot_1 = figure('Name', 'Trajectory', 'Renderer', 'painters');
plot(x, y, 'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
hold on
plot(q_i(1), q_i(2), 'ko', 'MarkerFaceColor', [0.9, 0.9, 0.9], 'MarkerSize', 5)
plot(q_f(1), q_f(2), 'ko', 'MarkerFaceColor', [0.2, 0.2, 0.2], 'MarkerSize', 5)
title('Trajectory', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$x [m]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$y [m]$', 'Interpreter', 'latex', 'FontSize', 26)
grid on
xlim([min(x), max(x)])
ylim([min(y), max(y)])
axis equal
legend({'Unicycle Trajectory', 'Starting Position', 'Final Position'}, 'Interpreter', 'latex', 'Location', 'northeast')
exportgraphics(plot_1, 'Trajectory_Pos_Reg.pdf');

plot_2 = figure('Name', 'Original Variables', 'Position', [10 10 900 350]);
plot(t_out,x,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,y,'k-.','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
plot(t_out,theta,'k--','Color', [0.7, 0.7, 0.7], 'LineWidth', 2.5)
title('Original Variables', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$x,y,\theta [m,rad]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min([min(x), min(y), min(theta)]), max([max(x), max(y), max(theta)])])
legend({'$x(t)$', '$y(t)$', '$\theta(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_2, 'Original_Variables.pdf');


plot_3 = figure('Name', 'Polar Coordinates', 'Position', [10 10 900 350]);
plot(t_out,rho,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,gamma,'k-.','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
plot(t_out,delta,'k--','Color', [0.7, 0.7, 0.7], 'LineWidth', 2.5)
title('Polar Coordinates', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\rho,\delta,\gamma [m,rad]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min([min(x), min(y), min(theta)]), max([max(x), max(y), max(theta)])])
legend({'$\rho(t)$', '$\gamma(t)$', '$\delta(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_3, 'Polar_Coordinates.pdf');

plot_4 = figure('Name', 'Heading Velocity', 'Position', [10 10 900 350]);
plot(t_out,v,'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
title('Heading Velocity', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$v [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min(v), max(v)])
legend({'$v(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_4, 'Heading_Velocity.pdf');

plot_5 = figure('Name', 'Angular Velocity', 'Position', [10 10 900 350]);
plot(t_out,w,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
title('Angular Velocity', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\omega [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min(w), max(w)])
legend({'$\omega(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_5, 'Angular_Velocity.pdf');

