% Path Planning Alhorithm for a Unicycle based on a Cubic Cartesian
% Polynomial
% Initial Configuration q_i = [x_i y_i theta_i]' = [0 0 0]'
% Final Configuration q_f = [x_f y_f theta_f]' generated automatically
% by the code such that ||q_f-q_i||=1

%% INIZIALIZATION
clc
clear
close all

%% EXERCISE 1
%% CONFIGURATIONS

% Initial Configuration
x_i = 0;
y_i = 0;
theta_i = 0;
q_i = [x_i, y_i, theta_i]';
disp('Initial Configuration: ')
q_i

% Final Configuration
q_f_rand = rand(1,3);
q_f_rand = q_f_rand/norm(q_f_rand);

x_f = q_f_rand(1);
y_f = q_f_rand(2);
theta_f = q_f_rand(3);

q_f = [x_f, y_f, theta_f]';
%q_f = q_f_rand';

disp('Final Configuration: ')
q_f

% Check
disp('||q_f-q_i|| = ')
disp(norm(q_f-q_i))

%% TRAJECTORY

k = 1;
alpha_x = k*cos(theta_f) - 3*x_f;
alpha_y = k*sin(theta_f) - 3*y_f;
beta_x = k*cos(theta_i) + 3*x_i;
beta_y = k*sin(theta_i) + 3*y_i;

% s = 0 : 0.001 :1;   % s from 0 to 1

% Time
T = 1;    % Final Time - Initial Time -> Trajectory Duration

dt = 0.001;
t = 0 : dt :T;   % Time from 0 to T

% Coefficients for the Cubic Polynomial
a0 = 0;
a1 = 0;
a2 = 3/(T^2);
a3 = -2/(T^3);

% s(t) and Time Derivatives
s = a3.*t.^3 + a2.*t.^2 + a1.*t + a0;  % Position (Cubic Profile)
s_dot = 3*a3.*t.^2 + 2*a2.*t + a1;    % Velocity (Prabolic Profile)
s_ddot = 6*a3.*t + 2*a2; % Acceleration (Linear Profile)

x_s = s.^3*x_f-(s-1).^3.*x_i + alpha_x.*s.^2.*(s-1) + beta_x.*s.*(s-1).^2;
y_s = s.^3*y_f-(s-1).^3.*y_i + alpha_y.*s.^2.*(s-1) + beta_y.*s.*(s-1).^2;

% Path
plot_1 = figure('Name', 'Path', 'Renderer', 'painters');
plot(x_s, y_s, 'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(x_i, y_i, 'ko', 'MarkerFaceColor', [0.9, 0.9, 0.9], 'MarkerSize', 5)
plot(x_f, y_f, 'ko', 'MarkerFaceColor', [0.1, 0.1, 0.1], 'MarkerSize', 5)
title('Path', 'Interpreter', 'latex', 'FontSize', 20)
xlabel('$x [m]$', 'Interpreter', 'latex', 'FontSize', 20)
ylabel('$y [m]$', 'Interpreter', 'latex', 'FontSize', 20)
grid on
xlim([min(x_s)-0.05, max(x_s)+0.05])
ylim([min(y_s)-0.05, max(y_s)+0.05])
legend({'Path', 'Initial Position', 'Final Position'}, 'Interpreter', 'latex', 'Location', 'best')
exportgraphics(plot_1, 'Path.pdf');

x_s_p = 3.*s.^2*x_f - 3.*(s-1).^2*x_i + alpha_x.*s.*(3.*s-2) + beta_x.*(s-1).*(3.*s-1);
y_s_p = 3.*s.^2*y_f - 3.*(s-1).^2*y_i + alpha_y.*s.*(3.*s-2) + beta_y.*(s-1).*(3.*s-1);

x_s_s = 6.*s.*x_f - 6.*(s-1).*x_i + 2.*alpha_x.*(3.*s-1) + 2.*beta_x.*(3.*s-2);
y_s_s = 6.*s.*y_f - 6.*(s-1).*y_i + 2.*alpha_y.*(3.*s-1) + 2.*beta_y.*(3.*s-2);

% Angular Position
theta_s = atan2(y_s_p,x_s_p);

v_max = 0.5;    % Maximum Heading Velocity
omega_max = 2;  % Maximum Angular Velocity

% Geometric Inputs
v_tilde = sqrt(x_s_p.^2+y_s_p.^2);
omega_tilde = (y_s_s.*x_s_p - x_s_s.*y_s_p)./(x_s_p.^2 + y_s_p.^2);

% Velocities Reconstruction by the Geometric Ones
v = v_tilde.*s_dot;
omega = omega_tilde.*s_dot;

disp(['Trajectory Duration: ', num2str(T)])  
disp(['Maximum Heading Velocity reached: ', num2str(max(v))])  
disp(['Maximum Angular Velocity reached: ', num2str(max(omega))])  

% Plot Velocities
plot_3 = figure('Name', 'Heading Velocity', 'Position', [10 10 900 350]);
plot(t,v,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
title('Heading Velocity', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$v [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min(v)-0.05, max(v)+0.05])
legend({'$v(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_3, 'Heading_Velocity.pdf');

plot_4 = figure('Name', 'Angular Velocity', 'Position', [10 10 900 350]);
plot(t,omega,'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
title('Angular Velocity', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\omega [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T])
ylim([min(omega)-0.05, max(omega)+0.05])
legend({'$\omega(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_4, 'Angular_Velocity.pdf');

disp(' ')

%% TIME SCALING

if (max(abs(v)) > 0.5 || max(abs(omega)) > 2)
    disp('Velocity Bounds Exceeded. It is necessary a Time Scaling')
    
    % Percentage of Overshoot (only if exceeded, otherwise set to 0)
    p_v = max(0, (max(abs(v))-v_max)/v_max * 100);
    p_omega = max(0, (max(abs(omega))-omega_max)/omega_max * 100);

    % Take the Highest Overshoot Percentage between the two
    p_max = max(p_v, p_omega);
    
    % Safety Margin of 10%
    % safety_margin = 10;
    % p_max_safety = p_max + safety_margin;

    % New Trajectory Duration
    %T = T * (1 + p_max_safety/100);
    T = T * (1 + p_max/100);
    
    % New Time Variable
    t = 0 : dt :T;   % Time from 0 to T
    
    % Let's follow, the same previous passages
    % Coefficients for the Cubic Polynomial
    a0 = 0;
    a1 = 0;
    a2 = 3/(T^2);
    a3 = -2/(T^3);

    % s(t) and Time Derivatives
    s = a3.*t.^3 + a2.*t.^2 + a1.*t + a0;  % Position (Cubic Profile)
    s_dot = 3*a3.*t.^2 + 2*a2.*t + a1;    % Velocity (Prabolic Profile)
    s_ddot = 6*a3.*t + 2*a2; % Acceleration (Linear Profile)
    
    x_s = s.^3*x_f-(s-1).^3.*x_i + alpha_x.*s.^2.*(s-1) + beta_x.*s.*(s-1).^2;
    y_s = s.^3*y_f-(s-1).^3.*y_i + alpha_y.*s.^2.*(s-1) + beta_y.*s.*(s-1).^2;

    x_s_p = 3.*s.^2*x_f - 3.*(s-1).^2*x_i + alpha_x.*s.*(3.*s-2) + beta_x.*(s-1).*(3.*s-1);
    y_s_p = 3.*s.^2*y_f - 3.*(s-1).^2*y_i + alpha_y.*s.*(3.*s-2) + beta_y.*(s-1).*(3.*s-1);
    
    x_s_s = 6.*s.*x_f - 6.*(s-1).*x_i + 2.*alpha_x.*(3.*s-1) + 2.*beta_x.*(3.*s-2);
    y_s_s = 6.*s.*y_f - 6.*(s-1).*y_i + 2.*alpha_y.*(3.*s-1) + 2.*beta_y.*(3.*s-2);

    % Angular Position
    theta_s = atan2(y_s_p,x_s_p);

    plot_2 = figure('Name', 'Angular Position', 'Renderer', 'painters');
    plot(t,theta_s, 'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
    hold on
    plot(0, theta_i, 'ko', 'MarkerFaceColor', [0.9, 0.9, 0.9], 'MarkerSize', 5)
    plot(T, theta_f, 'ko', 'MarkerFaceColor', [0.1, 0.1, 0.1], 'MarkerSize', 5)
    title('Angular Position', 'Interpreter', 'latex', 'FontSize', 20)
    xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 20)
    ylabel('$\theta [rad]$', 'Interpreter', 'latex', 'FontSize', 20)
    grid on
    xlim([0, T])
    ylim([min(theta_s)-0.05, max(theta_s)+0.05])
    legend({'$\theta(s)$', '$\theta_i$', '$\theta_f$'}, 'Interpreter', 'latex', 'Location', 'best')
    exportgraphics(plot_2, 'Angular_Position.pdf');


    % Geometric Inputs with Time Scaling
    v_tilde = sqrt(x_s_p.^2+y_s_p.^2);
    omega_tilde = (y_s_s.*x_s_p - x_s_s.*y_s_p)./(x_s_p.^2 + y_s_p.^2);
    
    % Velocities Reconstruction by the Geometric Ones with Time Scaling
    v = v_tilde.*s_dot;
    omega = omega_tilde.*s_dot;

    disp(' ')
    disp(['New Trajectory Duration: ', num2str(T)])  
    disp(['Maximum Heading Velocity reached with Time Scaling: ', num2str(max(v))])  
    disp(['Maximum Angular Velocity reached with Time Scaling: ', num2str(max(omega))])  
    
    % Plot Velocities with Time Scaling
    plot_5 = figure('Name', 'Heading Velocity with Time Scaling', 'Position', [10 10 900 350]);
    plot(t,v,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
    title('Heading Velocity with Time Scaling', 'Interpreter', 'latex', 'FontSize', 26)
    xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
    ylabel('$v [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
    set(gca, 'FontSize', 26);
    grid on
    box on
    xlim([0, T])
    ylim([min(v)-0.05, max(v)+0.05])
    legend({'$v_s(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
    set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
    annotation('rectangle',[0 0 1 1],'Color','w');
    exportgraphics(plot_5, 'Scaled_Heading_Velocity.pdf');
    
    plot_6 = figure('Name', 'Angular Velocity with Time Scaling', 'Position', [10 10 900 350]);
    plot(t,omega,'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
    title('Angular Velocity with Time Scaling', 'Interpreter', 'latex', 'FontSize', 26)
    xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
    ylabel('$\omega [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
    set(gca, 'FontSize', 26);
    grid on
    box on
    xlim([0, T])
    ylim([min(omega)-0.05, max(omega)+0.05])
    legend({'$\omega_s(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
    set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
    annotation('rectangle',[0 0 1 1],'Color','w');
    exportgraphics(plot_6, 'Scaled_Angular_Velocity.pdf');

else
    disp('Time Scaling not necessary. All the Velocities Bounds are satisfied')  

end



%% EXERCISE 2

b=0.6;
k_1 = 60;
k_2 = 60;

x_d = timeseries(x_s,t);
y_d = timeseries(y_s,t);
theta_d = timeseries(theta_s,t);

% y_1_des = x_s + b.*cos(theta_s);
% y_2_des = y_s + b.*sin(theta_s);
% %y_1_dot_des = v.*cos(theta_s) - b.*sin(theta_s).*omega;
% %y_2_dot_des = v.*sin(theta_s) + b.*cos(theta_s).*omega;
% y_1_dot_des = gradient(y_1_des,dt);
% y_2_dot_des = gradient(y_2_des,dt);
% x_d_p = timeseries(x_s_p,t);
% y_d_p = timeseries(y_s_p,t);
% y_1d = timeseries(y_1_des,t);
% y_2d =  timeseries(y_2_des,t);
% y_1d_dot = timeseries(y_1_dot_des,t);
% y_2d_dot = timeseries(y_2_dot_des,t);

% Start Simulink Scheme
out = sim("HW2_FSR_ES2.slx");

% Plot From Simulink
plot_7 = figure('Name', 'Path with I/O Linearization', 'Renderer', 'painters');
plot(out.x.data, out.y.data, 'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(x_i, y_i, 'ko', 'MarkerFaceColor', [0.9, 0.9, 0.9], 'MarkerSize', 5)
plot(x_f, y_f, 'ko', 'MarkerFaceColor', [0.1, 0.1, 0.1], 'MarkerSize', 5)
title('Path with I/O Linearization', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$x [m]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$y [m]$', 'Interpreter', 'latex', 'FontSize', 26)
grid on
xlim([min(out.x.data)-0.05, max(out.x.data)+0.05])
ylim([min(out.y.data)-0.05, max(out.y.data)+0.05])
legend({'Path', 'Initial Position', 'Final Position'}, 'Interpreter', 'latex', 'Location', 'best')
exportgraphics(plot_7, 'Path_I_O_Linearization.pdf');


t_out = out.e_x.Time;
e_x = squeeze(out.e_x.Data);
e_y = squeeze(out.e_y.Data);
e_theta = squeeze(out.e_theta.Data);

plot_8 = figure('Name', 'Position Errors with I/O Linearization', 'Position', [10 10 900 350]);
plot(t_out,e_x,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,e_y,'k--','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
title('Position Errors with I/O Linearization', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T+1])
ylim([min([e_x(:); e_y(:)])-0.0005, max([e_x(:); e_y(:)])+0.0005])
legend({'$e_x(t)$', '$e_y(t)$'}, 'Interpreter', 'latex', 'Location', 'best')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_8, 'Position_Errors_I_O_Linearization.pdf');



plot_9 = figure('Name', 'Orientation Error with I/O Linearization', 'Position', [10 10 900 350]);
plot(t_out,e_theta,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
title('Orientation Error with I/O Linearization', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [rad]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T+1])
ylim([min(e_theta)-0.005, max(e_theta)+0.005])
legend({'$e_\theta(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_9, 'Orientation_Error_I_O_Linearization.pdf');


plot_10 = figure('Name', 'Positions with I/O Linearization', 'Position', [10 10 900 350]);
plot(t_out,out.x.data,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
hold on
plot(t_out,out.y.data,'k--','Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
title('Positions with I/O Linearization', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$x, y [m]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T+1])
ylim([min([out.x.data(:); out.y.data(:)])-0.05, max([out.x.data(:); out.y.data(:)])+0.05])
legend({'$x(t)$', '$y(t)$'}, 'Interpreter', 'latex', 'Location', 'best')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_10, 'Positions_I_O_Linearization.pdf');


plot_11 = figure('Name', 'Heading Velocity with I/O Linearization', 'Position', [10 10 900 350]);
plot(t_out,out.vel.data,'Color', [0.5, 0.5, 0.5], 'LineWidth', 2.5)
title('Heading Velocity with I/O Linearization', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$v [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T+1])
ylim([min(out.vel.data)-0.05, max(out.vel.data)+0.05])
legend({'$v(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_11, 'Heading_Velocity_I_O_Linearization.pdf');
    
plot_12 = figure('Name', 'Angular Velocity with I/O Linearization', 'Position', [10 10 900 350]);
plot(t_out,out.w.data,'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.5)
title('Angular Velocity with I/O Linearization', 'Interpreter', 'latex', 'FontSize', 26)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\omega [rad/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, T+1])
ylim([min(out.w.data)-0.05, max(out.w.data)+0.05])
legend({'$\omega(t)$'}, 'Interpreter', 'latex', 'Location', 'northeast')
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_12, 'Angular_Velocity_I_O_Linearization.pdf');

