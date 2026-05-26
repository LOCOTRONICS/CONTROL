clc; clear; close all;

%% CONEXIÓN
sim = remApi('remoteApi');
sim.simxFinish(-1);

clientID = sim.simxStart('127.0.0.1',19997,true,true,5000,5);

if clientID < 0
    error('No se pudo conectar con CoppeliaSim');
end

disp('Conectado a CoppeliaSim');

%% HANDLES
[~, t1] = sim.simxGetObjectHandle(clientID, '/target1', sim.simx_opmode_blocking);
[~, t2] = sim.simxGetObjectHandle(clientID, '/target2', sim.simx_opmode_blocking);
[~, t3] = sim.simxGetObjectHandle(clientID, '/target3', sim.simx_opmode_blocking);

%% INICIALIZACIÓN DE STREAMING
sim.simxGetObjectPosition(clientID, t1, -1, sim.simx_opmode_streaming);
sim.simxGetObjectPosition(clientID, t2, -1, sim.simx_opmode_streaming);
sim.simxGetObjectPosition(clientID, t3, -1, sim.simx_opmode_streaming);
pause(1);

%% TRAYECTORIA DE REFERENCIA (SPLINES LINEALES)
% 1. Define los puntos clave por los que quieres que pasen los drones [x, y]
waypoints = [0, 0; 
             2, 2;
             4, 0;
             2, 2;
             0, 0; 
             -2, 2;
             0, 4;
             -2, 2;
             0, 0;
             2, -2;
             0, -4;
             2, -2;
             0, 0;
             -2, -2;
             -4, 0;
             -2, -2;
             0, 0]; 

N = 4000; % Número total de puntos en la simulación
t_puntos = linspace(1, size(waypoints, 1), N); % Vector de tiempo ficticio

% 2. Interpolación lineal (Spline Lineal)
xs = interp1(1:size(waypoints, 1), waypoints(:, 1), t_puntos, 'linear');
ys = interp1(1:size(waypoints, 1), waypoints(:, 2), t_puntos, 'linear');

% (Opcional) Si quieres que las esquinas no sean tan "bruscas" y sean curvas suaves, 
% solo cambia 'linear' por 'spline' o 'makima'.
%% PARÁMETROS DINÁMICOS
m = 0.5;          % Masa del dron (kg)
b = 0.1;          % Coeficiente de fricción/viscosidad (aire)
dt = 0.05;        % Paso de tiempo

% Ganancias del controlador dinámico (Ajustar según respuesta)
Kp = 1.5;         % Ganancia proporcional (atracción al objetivo)
Kd = 0.8;         % Ganancia derivativa (amortiguamiento)

% Inicializar velocidades
v1 = [0 0 0]; v2 = [0 0 0]; v3 = [0 0 0];
%% PARÁMETROS DE CONTROL

Kc = 0.7;

Kt = 1.4;

v = 0.08;

v_max = 0.15;
%% DEFINICIÓN DE FORMACIÓN
d_long = 1.2;
d_lat  = 1.0;

r1 = [0 0 0];                    
r2 = [-d_long  d_lat 0];        
r3 = [-d_long -d_lat 0];        

%% VARIABLES DE HISTORIAL
x1=zeros(1,N); y1=zeros(1,N);
x2=zeros(1,N); y2=zeros(1,N);
x3=zeros(1,N); y3=zeros(1,N);

err1 = zeros(1,N);
form_err2 = zeros(1,N);
form_err3 = zeros(1,N);

%% VISUALIZACIÓN EN TIEMPO REAL
figure
hold on; grid on; axis equal
title('Movimiento de los drones')
xlabel('X'); ylabel('Y')

h1 = plot(0,0,'r','LineWidth',2);
h2 = plot(0,0,'g','LineWidth',2);
h3 = plot(0,0,'b','LineWidth',2);

%% BUCLE PRINCIPAL
for i = 1:N
    
    % 1. Referencia global
    pd = [xs(i) ys(i) 1];
    
    % 2. Referencias individuales
    pd1 = pd + r1;
    pd2 = pd + r2;
    pd3 = pd + r3;
    
    % 3. Lectura de posiciones (IMPORTANTE: deben ser arreglos de 3 elementos)
    [~, p1] = sim.simxGetObjectPosition(clientID, t1, -1, sim.simx_opmode_buffer);
    [~, p2] = sim.simxGetObjectPosition(clientID, t2, -1, sim.simx_opmode_buffer);
    [~, p3] = sim.simxGetObjectPosition(clientID, t3, -1, sim.simx_opmode_buffer);
    
    % Si la lectura falla al inicio, saltar iteración
    if length(p1) < 3, continue; end

    % 4. Cálculo de Errores
    e1 = pd1 - p1;
    e2 = pd2 - p2;
    e3 = pd3 - p3;
    
    % 5. Consenso Dinámico
    u_c2 = -( (p2 - p1) - r2 ) - (v2 - v1); 
    u_c3 = -( (p3 - p1) - r3 ) - (v3 - v1);

    % 6. Ley de Control (Fuerzas)
    F1 = Kp*e1 - Kd*v1; 
    F2 = Kt*e2 - Kd*v2 + Kc*u_c2; % Nota: Usamos Kt para seguidores
    F3 = Kt*e3 - Kd*v3 + Kc*u_c3;

    % 7. INTEGRACIÓN (Indispensable para los 3 drones)
    % Drone 1
    a1 = (F1 - b*v1)/m;
    v1 = v1 + a1*dt;
    p1_new = p1 + v1*dt;
    
    % Drone 2
    a2 = (F2 - b*v2)/m;
    v2 = v2 + a2*dt;
    p2_new = p2 + v2*dt;
    
    % Drone 3
    a3 = (F3 - b*v3)/m;
    v3 = v3 + a3*dt;
    p3_new = p3 + v3*dt;

    % 8. ENVÍO A COPPELIASIM (¡Sin esto no se mueven!)
    sim.simxSetObjectPosition(clientID, t1, -1, p1_new, sim.simx_opmode_oneshot);
    sim.simxSetObjectPosition(clientID, t2, -1, p2_new, sim.simx_opmode_oneshot);
    sim.simxSetObjectPosition(clientID, t3, -1, p3_new, sim.simx_opmode_oneshot);
    
    % 9. Almacenamiento para gráficas
    x1(i)=p1(1); y1(i)=p1(2);
    x2(i)=p2(1); y2(i)=p2(2);
    x3(i)=p3(1); y3(i)=p3(2);
    
    % Errores
    err1(i) = norm(e1);
    form_err2(i) = norm((p2-p1)-r2);
    form_err3(i) = norm((p3-p1)-r3);
    
    % 10. Actualización Gráfica en MATLAB
    set(h1,'XData',x1(1:i),'YData',y1(1:i));
    set(h2,'XData',x2(1:i),'YData',y2(1:i));
    set(h3,'XData',x3(1:i),'YData',y3(1:i));
    
    drawnow limitrate
    pause(0.01); % Pausa crítica para sincronizar con la física del simulador
end
%% RESULTADOS

% Seguimiento del líder
figure
plot(xs, ys,'g--','LineWidth',2)
hold on
plot(x1,y1,'r','LineWidth',2)
legend('Trayectoria deseada','Líder')
title('Seguimiento de trayectoria del líder')
xlabel('X'); ylabel('Y')
grid on
axis equal

% Error del líder
figure
plot(err1,'r','LineWidth',2)
title('Error de seguimiento del líder')
xlabel('Iteraciones')
ylabel('Error (m)')
grid on

% Error de formación
figure
plot(form_err2,'g','LineWidth',2)
hold on
plot(form_err3,'b','LineWidth',2)
legend('Seguidor 1','Seguidor 2')
title('Error de formación respecto al líder')
xlabel('Iteraciones')
ylabel('Error (m)')
grid on

%% CIERRE DE CONEXIÓN
sim.simxFinish(clientID);
sim.delete();