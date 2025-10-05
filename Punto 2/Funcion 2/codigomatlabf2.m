clear; clc; close all;

% Planta original
G = tf([1 2], [5 6 13 8 7]);

% Sistema sin control (lazo cerrado unitario)
T_open = feedback(G, 1);


% Parámetros de desempeño deseados

zeta = 0.69;         % Factor de amortiguamiento
wn   = 2.898;        % Frecuencia natural
p_extra = -10;       % Polos adicionales

% Polinomio de segundo orden dominante
poly2 = [1 2*zeta*wn wn^2];

% Polos adicionales (doble polo en -10)
poly_extra = [1 -2*p_extra p_extra^2];  % (s+10)^2

% Polinomio característico deseado completo
poly_desired = conv(poly2, poly_extra);

fprintf('--- Polinomio característico deseado ---\n');
disp(poly_desired);


% Ganancias calculadas manualmente

Kp = 4743.86;
Ki = 10498.75;
Kd = 1753.99;

C = pid(Kp, Ki, Kd);  % Controlador PID ideal

% Sistema con control (lazo cerrado)
T_pid = feedback(C*G, 1);


% Métricas de desempeño

S_open = stepinfo(T_open);
S_pid  = stepinfo(T_pid);

% Error estacionario
ess_open = 1 - dcgain(T_open);
ess_pid  = 1 - dcgain(T_pid);

fprintf('\n--- Sistema sin control ---\n');
fprintf('Tr = %.3f s, Mp = %.2f %%, Ts = %.3f s, ess = %.4f\n\n', ...
    S_open.RiseTime, S_open.Overshoot, S_open.SettlingTime, ess_open);

fprintf('--- Sistema con PID (ganancias manuales) ---\n');
fprintf('Tr = %.3f s, Mp = %.2f %%, Ts = %.3f s, ess = %.4f\n', ...
    S_pid.RiseTime, S_pid.Overshoot, S_pid.SettlingTime, ess_pid);

% Gráfica comparativa
t = 0:0.01:20;
[y_open, t_open] = step(T_open, t);
[y_pid, t_pid]   = step(T_pid, t);

figure;
plot(t_open, y_open, 'b', 'LineWidth', 1.5); hold on;
plot(t_pid, y_pid, 'r', 'LineWidth', 1.5);
yline(1, '--k', 'Referencia');
grid on;
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Respuesta al escalón: Planta sin control vs con PID manual');
legend('Sin control', 'Con PID manual', 'Referencia','Location','Southeast');

% ===============================
%Diseño con pidtune
% ===============================
[C_auto, info] = pidtune(G,'PID');   % Sintonia automática
T_auto = feedback(C_auto*G,1);

% Métricas con pidtune
S_auto = stepinfo(T_auto,'YFinal',1);
ess_auto = 1 - dcgain(T_auto);

fprintf('\n--- Sistema con PID (pidtune) ---\n');
fprintf('Ganancias obtenidas: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', ...
    C_auto.Kp, C_auto.Ki, C_auto.Kd);
fprintf('Tr = %.3f s, Mp = %.2f %%, Ts = %.3f s, ess = %.4f\n', ...
    S_auto.RiseTime, S_auto.Overshoot, S_auto.SettlingTime, ess_auto);

% Gráfica comparativa incluyendo pidtune
[y_auto, t_auto] = step(T_auto, t);
figure;
plot(t_open, y_open, 'b', 'LineWidth', 1.5); hold on;
plot(t_pid, y_pid, 'r', 'LineWidth', 1.5);
plot(t_auto, y_auto, 'g', 'LineWidth', 1.5);
yline(1, '--k', 'Referencia');
grid on;
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Comparación: sin control, PID manual, PID pidtune');
legend('Sin control','PID manual','PID pidtune','Referencia','Location','Southeast');

%%
% % T_open = feedback(G,1);
% disp('Polos de T_open:'); disp(pole(T_open));
% disp('dcgain(T_open) = '); disp(dcgain(T_open));
% [yT, tT] = step(T_open, 0:0.01:50);
% figure; plot(tT, yT); grid on; title('step(T\_open)');
