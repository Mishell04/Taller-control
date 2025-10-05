%% --- Planta identificada ---
K   = 8.66; 
tau = 7.07; 
G   = tf(K, [tau 1]);

%% --- Controlador PID diseñado ---
Kp = 6.351; 
Ki = 18.48; 
Kd = 0.338;
C  = pid(Kp, Ki, Kd);

%% --- Lazo cerrado ---
T_open = feedback(G, 1);      % Planta sin control
T_pid  = feedback(C*G, 1);    % Planta con PID

%% --- Simulación ---
t = 0:0.001:30;
[y_open, ~] = step(T_open, t);
[y_pid,  ~] = step(T_pid, t);

%% --- Cálculo del polinomio deseado ---
zeta = 0.7;           % coeficiente de amortiguamiento deseado
wn   = 4;             % frecuencia natural deseada
poly_desired = [1 2*zeta*wn wn^2];
fprintf('\n--- Polinomio característico deseado ---\n');
disp(poly_desired);

alpha = 10;   % factor de escala elegido en el diseño
Kd_calc = (alpha - tau) / K;
Kp_calc = (alpha*2*zeta*wn - 1) / K;
Ki_calc = (alpha*wn^2) / K;

fprintf('\n--- Parámetros PID obtenidos (alpha=%.2f) ---\n', alpha);
fprintf('Kp = %.3f, Ki = %.3f, Kd = %.3f\n', Kp_calc, Ki_calc, Kd_calc);

poly_real = [tau + K*Kd_calc, 1 + K*Kp_calc, K*Ki_calc];
fprintf('\nPolinomio característico real:\n');
disp(poly_real);

poly_real_norm = poly_real / poly_real(1);
fprintf('\nPolinomio real normalizado:\n');
disp(poly_real_norm);

%% --- Métricas de desempeño ---
S_open = stepinfo(T_open);
S_pid  = stepinfo(T_pid);

ess_open = abs(1 - dcgain(T_open));
ess_pid  = abs(1 - dcgain(T_pid));

%% --- Mostrar resultados en consola ---
fprintf('--- Planta sin control ---\n');
fprintf('Tr=%.3f s, Mp=%.2f %%, Ts=%.3f s, ess=%.4f\n', ...
    S_open.RiseTime, S_open.Overshoot, S_open.SettlingTime, ess_open);

fprintf('--- Planta con PID ---\n');
fprintf('Tr=%.3f s, Mp=%.2f %%, Ts=%.3f s, ess=%.4f\n', ...
    S_pid.RiseTime, S_pid.Overshoot, S_pid.SettlingTime, ess_pid);

%% --- Tabla de métricas lista para LaTeX ---
metrics = {'Tiempo de subida T_r [s]', S_open.RiseTime, S_pid.RiseTime;
           'Tiempo de establecimiento T_s [s]', S_open.SettlingTime, S_pid.SettlingTime;
           'Sobrepico M_p [%]', S_open.Overshoot, S_pid.Overshoot;
           'Error estacionario e_{ss}', ess_open, ess_pid};

disp('--- Tabla de métricas de desempeño ---');
disp('             Métrica                 Sin_control     Con_PID');
disp(metrics);

%% --- Gráfica comparativa ---
figure;
plot(t, y_open, 'r', 'LineWidth', 1.5); hold on;
plot(t, y_pid, 'b', 'LineWidth', 1.5);
yline(1,'--k','Referencia');
xlabel('Tiempo [s]');
ylabel('Salida');
legend('Sin control','Con PID','Referencia');
grid on;
title('Comparación de la respuesta del intercambiador con y sin PID');
