%% Archivo de datos

%% Datos de líneas de transmisión
nn = 5; %Número de nodos
nlts = 4; % Número de líneas
Zlts = [0.32694+0.66307i 0.32694+0.66307i 0.32694+0.66307i 0.093412 + 0.18945i]; %Impedancia Serie en Ohms
S = [450+0i 0+0i 1400+0i 50+0i 475+0i]; %Potencia aparente en kVA
YShuntlts = [0 0 0 0]; 
nalts = [1 2 2 4]; %Nodo a de Impedancia Serie
nblts = [2 3 4 5]; %Nodo b de Impedancia Serie
Vs = 13800; %Voltaje del alimentador en Volts