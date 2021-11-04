% Calcola il gradiente numerico per una funzione fun in un dato punto x0.
% Se x0 è una matrice, la funzione calcola il gradiente in vari punti, dove
% le colonne di x0 sono considerate come i punti in cui calcolare il
% gradiente.
% -> fun: online function tipo f = @(x)
% -> x0: punto/i in cui calcolare il gradiente (vettore colonna)
% -> method: metodo di derivazione numerica ('forward', 'backward',
% 'central'; default: 'forward')
% -> h: passo di differenziazione (default: 1e-6)


function [dy] = mygradient(fun,x0,method,h)
  
  if isempty(method)
    method = 'forward';
  end
  if isempty(h)
    h = 1e-6;
  end
  
  n = size(x0,1); % numero di variabili
  if strcmp(method,'forward')
    y0 = fun(x0);
    dx = repmat(x0,1,n) + h*eye(n);
    %Ih = diag(reshape(repmat(1./(((n/4):-1:1))*1000,4,1),n,1));
    %dx = repmat(x0,1,n) + h*Ih;
    dy = (fun(dx) - y0)/h;
  elseif strcmp(method,'backward')
    y0 = fun(x0);
    dx = repmat(x0,1,n) - h*eye(n);
    dy = (y0 - fun(dx))/h;
  elseif strcmp(method,'central')
    dx1 = repmat(x0,1,n) - h*eye(n);
    dx2 = dx1 + 2*h*eye(n);
    dy = (fun(dx2) - fun(dx1))/(2*h);
  end
  
  dy(dy<=1e-1*h & dy>=-1e-1*h) = 0;