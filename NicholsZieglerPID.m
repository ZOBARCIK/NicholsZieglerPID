clc
clear all
close all

%define the transfer function
s = tf('s');
Gs = (1/((s+1)^6))*exp(-0.5*s); %%open loop transfer function

% open loop step response of 20 sec
[y, t] = step(Gs, 20); 

% derivative of step response to draw tangent line
dy = diff(y)./diff(t);
t_mid = t(1:end-1) + diff(t)/2; % time vector for derivative
[~, max_idx] = max(dy);%returns index value fo max point of dy, is where the tangent should be drawn
t_max = t_mid(max_idx); % max slope time 
y_max = y(max_idx); % max step response 
dy_max = dy(max_idx); % max slope value

% slope of tangent line and y intercept point
tangent_slope = dy_max;
tangent_intercept = y_max - tangent_slope * t_max;

% x values at y0=0 and y1=K=1
% calculate T and L 
y0 = -tangent_intercept / tangent_slope;
L=y0 + 0.25;%with time delay
disp("L=: " + L) 

y1 = ((1 - tangent_intercept) / tangent_slope);
T=y1-L;
disp("T=: " + T) 

K=1;
disp("K=: " + K) 


% tangent line vector
tangent_line = tangent_slope * t + tangent_intercept;

% plotting
figure(1)
plot(t, y, 'b', 'LineWidth', 2); hold on;
plot(t_max, y_max, 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Maksimum türev noktası
plot(t, tangent_line, 'r--', 'LineWidth', 2); % Teğet doğru
plot(y0, 0, 'go', 'MarkerSize', 10, 'LineWidth', 2); % y = 0 noktasındaki x değeri
plot(y1, 1, 'mo', 'MarkerSize', 10, 'LineWidth', 2); % y = 1 noktasındaki x değeri
xlabel('Time (s)');
ylabel('Step Response');
ylim([0 1])
title('Step Response and Tangent Line');
legend('Open Loop Step Response', 'Max Derivative', 'Tangent Line', 'y=0 crossing', 'y=1 crossing');
grid on;


Kp = (1.2 * T) / (K * L);
Ki = (2*L);
Kd = 0.5 *  L;

fprintf('\n')
disp("Kp = (1.2 * T) / (K * L) = " + Kp)
disp("Ki = (2*L) = " + Ki)
disp("Kd = (0.5 *  L) = " + Kd)

% PID controller
% pid_controller = pid(Kp, Ki, Kd);
pid_controller = Kp*(1+(1/(Ki*s)) + Kd*s)
% s = tf('s');
% Gs = (1/((s+1)^6))*exp(-0.5*s);
pid_sys=feedback(pid_controller*Gs,1);
[y_pid, t_pid] =step(pid_sys,200);
[y, t] =step(Gs,200);

figure(2)
plot(t, y, 'b', 'LineWidth', 1.5); 
hold on
plot(t_pid, y_pid,'k', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Step Response');
ylim([0 1.5])
xlim([0 200])
title('System Step Response and Controlled Response');
legend('OPen Loop Step Respose','PID Closed Loop Step Response');
grid on;
