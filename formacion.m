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

%% TRAYECTORIA DE REFERENCIA
N = 400;
x = linspace(0,10,N);
y = 2*sin(0.8*x);

xs = x;
ys = y;

%% PARÁMETROS DE CONTROL
Kc = 0.7;    
Kt = 1.4;    

v  = 0.08;   
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
    
    % Referencia global
    pd = [xs(i) ys(i) 1];
    
    % Lectura de posiciones
    [~, p1] = sim.simxGetObjectPosition(clientID, t1, -1, sim.simx_opmode_buffer);
    [~, p2] = sim.simxGetObjectPosition(clientID, t2, -1, sim.simx_opmode_buffer);
    [~, p3] = sim.simxGetObjectPosition(clientID, t3, -1, sim.simx_opmode_buffer);
    
    % Referencias individuales
    pd1 = pd + r1;
    pd2 = pd + r2;
    pd3 = pd + r3;
    
    % Término de consenso
    u1_c = -( (p1 - p2) + (p1 - p3) );
    u2_c = -( (p2 - p1) + (p2 - p3) );
    u3_c = -( (p3 - p1) + (p3 - p2) );
    
    % Término de seguimiento
    u1_t = pd1 - p1;
    u2_t = pd2 - p2;
    u3_t = pd3 - p3;
    
    % Ley de control
    u1 = u1_t;
    u2 = Kc*u2_c + Kt*u2_t;
    u3 = Kc*u3_c + Kt*u3_t;
    
    % Saturación del líder
    if norm(u1) > v_max
        u1 = v_max * u1 / norm(u1);
    end
    
    % Normalización de seguidores
    u2 = v * u2 / (norm(u2)+1e-6);
    u3 = v * u3 / (norm(u3)+1e-6);
    
    % Actualización de posiciones
    p1_new = p1 + u1;
    p2_new = p2 + u2;
    p3_new = p3 + u3;
    
    % Envío a simulador
    sim.simxSetObjectPosition(clientID, t1, -1, p1_new, sim.simx_opmode_oneshot);
    sim.simxSetObjectPosition(clientID, t2, -1, p2_new, sim.simx_opmode_oneshot);
    sim.simxSetObjectPosition(clientID, t3, -1, p3_new, sim.simx_opmode_oneshot);
    
    % Almacenamiento de datos
    x1(i)=p1(1); y1(i)=p1(2);
    x2(i)=p2(1); y2(i)=p2(2);
    x3(i)=p3(1); y3(i)=p3(2);
    
    % Cálculo de errores
    err1(i) = norm(p1(1:2) - pd(1:2));
    form_err2(i) = norm( (p2 - p1) - r2 );
    form_err3(i) = norm( (p3 - p1) - r3 );
    
    % Actualización gráfica
    set(h1,'XData',x1(1:i),'YData',y1(1:i));
    set(h2,'XData',x2(1:i),'YData',y2(1:i));
    set(h3,'XData',x3(1:i),'YData',y3(1:i));
    
    drawnow
    pause(0.01);
end

%% RESULTADOS

% Seguimiento del líder
figure
plot(xs, ys,'k--','LineWidth',2)
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