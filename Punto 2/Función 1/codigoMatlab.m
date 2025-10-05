clear; clc; close all;

% --- Planta original ---
G = tf([1 20], [1 7 19 45]);

% --- Sistema sin control (lazo cerrado unitario) ---
T_open = feedback(G, 1);

% --- Controlador PID diseñado ---
Kp = 37.75;
Ki = 80;
Kd = 18.6;
C = pid(Kp, Ki, Kd);  % Forma ideal

% --- Sistema en lazo cerrado con PID ---
T_pid = feedback(C*G, 1);

%% 
%  Polinomio deseado y comparación

zeta = 0.7;       % Factor de amortiguamiento
wn   = 4;         % Frecuencia natural deseada
p_extra = -10;    % Polos adicionales

% Polinomio de segundo orden dominante
poly2 = [1 2*zeta*wn wn^2];

% Polos adicionales (doble polo en -10)
poly_extra = [1 -2*p_extra p_extra^2];  % (s+10)^2 = s^2+20s+100

% Polinomio característico deseado completo
poly_desired = conv(poly2, poly_extra);

fprintf('--- Polinomio característico deseado ---\n');
disp(poly_desired);

% Polinomio real obtenido con PID (ecuación característica real)
% s^4 + (7+Kd)s^3 + (19+20Kd+Kp)s^2 + (45+20Kp+Ki)s + 20Ki
poly_real = [1 (7+Kd) (19+20*Kd+Kp) (45+20*Kp+Ki) 20*Ki];

fprintf('--- Polinomio característico real con PID ---\n');
disp(poly_real);

%% ===============================
%  Métricas de desempeño
% ===============================
S_open = stepinfo(T_open);
S_pid  = stepinfo(T_pid);

% Error estacionario
ess_open = 1 - dcgain(T_open);
ess_pid  = 1 - dcgain(T_pid);

fprintf('\n--- Sistema sin control ---\n');
fprintf('Tr = %.3f s, Mp = %.2f %%, Ts = %.3f s, ess = %.4f\n\n', ...
    S_open.RiseTime, S_open.Overshoot, S_open.SettlingTime, ess_open);

fprintf('--- Sistema con PID ---\n');
fprintf('Tr = %.3f s, Mp = %.2f %%, Ts = %.3f s, ess = %.4f\n', ...
    S_pid.RiseTime, S_pid.Overshoot, S_pid.SettlingTime, ess_pid);

%% ===============================
%  Gráfica comparativa
% ===============================
t = 0:0.01:25;
[y_open, t_open] = step(T_open, t);
[y_pid, t_pid]   = step(T_pid, t);

figure;
plot(t_open, y_open, 'b', 'LineWidth', 1.5); hold on;
plot(t_pid, y_pid, 'r', 'LineWidth', 1.5);
yline(1, '--k', 'Referencia');
grid on;
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Respuesta al escalón: Planta sin control vs con PID');
legend('Sin control', 'Con PID', 'Referencia','Location','Southeast');
