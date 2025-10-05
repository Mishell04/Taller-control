clear; clc; close all;

% --- Planta ---
num = 100;
den = [0.5 50.1 110];
G = tf(num, den);

% --- Sistema sin control ---
S0 = stepinfo(G);
ess0 = 1 - dcgain(G);
fprintf('--- SIN CONTROL ---\n');
fprintf('Tr = %.2f s, Mp = %.1f %%, Ts = %.1f s, ess = %.4f\n\n', ...
    S0.RiseTime, S0.Overshoot, S0.SettlingTime, ess0);

% --- Polinomio característico deseado ---
zeta_d = 0.7;
ts_d = 0.5; 
wn_d = 4/(zeta_d*ts_d);  % frecuencia natural deseada
p3 = -10*wn_d;           % polo no dominante

% Polinomio deseado (s^2 + 2ζwn s + wn^2)(s - p3)
num_des = [1 2*zeta_d*wn_d wn_d^2];
den_des = [1 -p3];
poly_des = conv(num_des, den_des);

fprintf('--- POLINOMIO DESEADO ---\n');
fprintf('s^3 + (%.4f) s^2 + (%.4f) s + (%.4f)\n\n', ...
    poly_des(2), poly_des(3), poly_des(4));

% --- PID teórico (comparación de coeficientes) ---
Kp = 8.6959;
Ki = 74.6356;
Kd = 0.1504;
C = pid(Kp, Ki, Kd);

T = feedback(C * G, 1);

% Simular con tiempo adecuado
t = 0:0.001:3;
[y_pid, t_out] = step(T, t);
[y_open, ~] = step(G, t);

% Calcular métricas
try
    S = stepinfo(y_pid, t_out, 'RiseTimeLimits', [0.1 0.9]);
    Tr = S.RiseTime; Mp = S.Overshoot; Ts = S.SettlingTime;
catch
    Tr = NaN; Mp = NaN; Ts = NaN;
end
ess = 1 - dcgain(T);

fprintf('--- CON PID (Kp=%.4f, Ki=%.4f, Kd=%.4f) ---\n', Kp, Ki, Kd);
if any(isnan([Tr, Mp, Ts]))
    fprintf('Respuesta muy lenta (métricas no calculables)\n');
    fprintf('Error estacionario: %.4f\n', ess);
else
    fprintf('Tr = %.3f s, Mp = %.2f %%, Ts = %.3f s, ess = %.4f\n', ...
        Tr, Mp, Ts, ess);
end

% --- Gráfica comparativa ---
figure;
plot(t, y_open, 'b', 'LineWidth', 1.5); hold on;
plot(t_out, y_pid, 'r', 'LineWidth', 1.5);
yline(1, '--k', 'Referencia');
title('Respuesta al escalón: Planta sin control vs con PID');
xlabel('Tiempo (s)'); ylabel('Amplitud');
legend('Sin control','Con PID','Referencia','Location','SouthEast');
grid on;
