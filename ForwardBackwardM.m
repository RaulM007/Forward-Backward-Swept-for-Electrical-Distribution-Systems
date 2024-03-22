%% Inicialización de vectores de voltaje y corriente

I = zeros(nn); %Corrientes
V = zeros(1,nn); %Voltajes de nodo
V(1,1) = Vs; %Voltaje de alimentador primario

%% Formar YBUS
%Se forma el vector de admitancias
Ymhos = zeros(1,nlts); %Dimensión: 1 X Numero de impedancias
for i=1:nlts
    Ymhos(i)=1/Zlts(i);
end

% MATRIZ DE CONECTIVIDAD X´s y NODOS
MCRN = zeros(nn, nlts); %Dimensión: Nodos X Impedancias
for i=1:nlts
    MCRN (nalts(i),i) = 1;
    MCRN (nblts(i),i) = 1;
end

% MATRIZ DIAGONAL DE X´s
MDY = zeros(nlts); %Dimension: Impedancias X Impedancias
for i=1:nlts
    MDY (i,i)=Ymhos(i);
end

% SE QUITA EL RENGLON DEL NODO DE REFERENCIA
MCR = zeros(nn, nlts); %Dimension: Nodos X Impendancias
for i=1:nn
    for j=1:nlts
        MCR(i,j) = MCRN(i,j); 
    end
end

MPG = MCR*MDY;
MPenult = MCR*transpose(MPG);
Ynodal = MPenult;

% MATRIZ CON LOS ELEMENTOS FUERA DE LA DIAGONAL NEGATIVOS
for i=1:nn
    for j=1:nn
       if i~=j
           Ynodal(i,j)= Ynodal(i,j)*(-1);
       end
    end
end


%% Caminos por nodo
numpath = zeros(1,nn); %Vector con el número de caminos por nodo
for i=1:nn
    contador = 0;
    for j=i+1:nn
       if Ynodal(i,j) ~= 0 + 0i
           contador = contador + 1;
       end
    end
   numpath(1,i) = contador;
end
%% Determinación de camino más corto
%Contar nodos terminales de la red
nodos_cero = 0;
for i = 1: nn
    %Si numpath(i) == 0 entonces i es un nodo terminal
    if numpath(i) == 0
       nodos_cero = nodos_cero + 1; 
    end
end

%Obtener vector con los nodos terminales de la red
nodos_terminales = zeros(1, nodos_cero);
conta = 0;
for i=1: nn
    if numpath(i) == 0
        conta = conta + 1;
        nodos_terminales(1, conta) = i;
    end
end
%Matriz de caminos
caminos = zeros(nodos_cero, nn);%Contiene las rutas desde los nodos finales hasta el nodo 2
long_camino = zeros(nodos_cero,1);%Contiene la cantidad de nodos de los caminos
for i=1:nodos_cero
   nodo = nodos_terminales(1,i); %Iniciando en nodo terminal
   indices2 = find(nblts == nodo); %Buscar en nodos b al nodo terminal
   posicion = indices2(1,1); %Posicion dentro del vector nblts del nodo terminal
   cant_nodo = 1; %Contador 
   
   while(nodo ~= 2)
       caminos(i,cant_nodo) = nodo; %Representación de caminos en i filas
       cant_nodo = cant_nodo + 1;
       nodo = nalts(1,posicion); %Buscar en nalts al nodo de llegada
       indices2 = find(nblts == nodo);%Posicion dentro del vector nblts del nodo de llegada
       posicion = indices2(1,1); %Posicion dentro del vector nblts del nodo llegada
   end
   caminos(i,cant_nodo) = nodo;
   long_camino(i,1) = cant_nodo-1; 
end

%Selección de camino principal para barrido hacia atrás
Spath = nn; %Se inicia en camino más largo (camino de todos los nodos)
for i = 1: nodos_cero
    if long_camino(i,1) < Spath
       Spath = long_camino(i,1);
       camino_corto = i; %Fila de la variable caminos con la menor cantidad de nodos 
    end
end

camino_principal = zeros(1,long_camino(camino_corto,1)+1);
camino_principal(1,long_camino(camino_corto,1)+1)=2;
for i = 1:long_camino(camino_corto,1)
    
    camino_principal(1,i) = caminos(camino_corto,i);
end


%% Barrido
contador = 1;
error = 100;    
% Barrido hacía adelante
while error>=1 && contador <= 100

V(1,1) = Vs;

nodos_as = size(nalts);
nodos_a = nodos_as(1,2); %Cantidad de nodos iniciales

for i = 1:nodos_a
    
    a = nalts(1,i);
    b = nblts(1,i);
    
    V(1,b) = V(1,a) - (Zlts(1,i))*(I(a,b));

end
%disp(V);
% Barrido hacia atrás
I = zeros(nn);%Reinicio de corrientes
for i=1:nodos_cero
    for j=1: long_camino(i,1)
        if i == camino_corto
            %Camino principal
            a = caminos(i,j+1); %a es el nodo más lejano
            b = caminos(i,j);%b es el nodo más cercano
            
            I(b,b) = 1000*conj(S(b)/V(b)); %%Corriente en la carga del nodo b
            
            indices = find(nalts == b); %Posición dentro de nalts que contiene a nodo b
            indT = size(indices);
            cant = indT(1,2);%Cantidad de veces que b está en nalts
            I(a,b) = I(b,b);

            for k = 1:cant
                pos = indices(1,k);
                c = nalts(1,pos); %Nodo auxiliar      
                d = nblts(1,pos);
                I(a,b) = I(a,b) + I(c,d);
            end
           
            caida = I(a,b)*Zlts(1,a); 
            V(1,a) = V(1,b) + caida;
        else
            %Caminos secundarios más largos
            a = caminos(i,j+1); %a es el nodo más lejano
            b = caminos(i,j);%b es el nodo más cercano
            
            I(b,b) = 1000*conj(S(b)/V(b)); %%Corriente en la carga del nodo b
            
            indices = find(nalts == b); %Posición dentro de nalts que contiene a nodo b
            indT = size(indices);
            cant = indT(1,2);%Cantidad de veces que b está en nalts
            I(a,b) = I(b,b);

            for k = 1:cant
                pos = indices(1,k);
                c = nalts(1,pos); %Nodo auxiliar      
                d = nblts(1,pos);
                I(a,b) = I(a,b) + I(c,d);
            end
             
           if a ~= 2  
               %V(1,2) no se calcula porque es el camino corto el que dicta
               %el valor de dicho voltaje
               caida = I(a,b)*Zlts(1,a); 
               V(1,a) = V(1,b) + caida;
           end
        end     
    end   
end

% En este punto se tienen todas las corrientes de las ramas salientes del
% nodo 2 por lo que la suma de estas será la corriente I(1,2)
for i = 1:nn
    I(1,2) = I(1,2) + I(2,i);
end
%disp(I);
%Cálculo de V1
V(1,1) = V(1,2) + Zlts(1,1)*I(1,2);
%Cálculo I(1,1)
I(1,1) = 1000*conj(S(1,1)/V(1,1));

error = abs(abs(Vs) - abs(V(1,1)));
contador = contador + 1;


%% Impresión de resultados
fprintf('\nIteración %i: \n',contador-1);
fprintf('Voltajes nodales: \n');
for i = 1:nn

    fprintf('V%i = %s V\n',i, num2str(V(1,i)));

end
fprintf('Corrientes de carga: \n');
for i = 1:nn

    fprintf('I%i = %s A\n',i, num2str(I(i,i)));

end
fprintf('Corrientes en líneas de transmisión: \n');
for i = 1:nn

    for j = 1:nn
        if I(i,j) ~= 0 + 0i && i~=j
            fprintf('I%i%i = %s A\n',i,j, num2str(I(i,j)));
        end
    end
end
end