% settings
DISTRIBUTION='uniform';
TORQ_CONST = 1;
MODIFY_SREF=0;
lambdaU = 1e0; 
lmd = 1e7;
lambdaTRJ = [lmd, 1e5];
Kclik = blkdiag(50e0*eye(robot.df));
optqpOASES = qpOASES_options('MPC','maxIter',1000,'terminationTolerance',1e6*eps, 'boundTolerance', 1e10*eps,'maxCpuTime',-1); % ,'maxCpuTime',1e-3


df=robot.df;
Qmax=robot.q_max;
Qmin=robot.q_min;
Qpmax=robot.dq_max;
Qpmin=robot.dq_min;
Qppmax=robot.ddq_max;
Qppmin=robot.ddq_min;

tau_max=robot.tau_max;
tau_min=robot.tau_min;

Ntau=Np;


% initialize thor controller

if Np==1
  hp = st;
  hc = st;
else
  if strcmp(DISTRIBUTION,'uniform')     % Distribuzione uniforme (classic MPC)
    tp = st*linspace(0,horizon,Np+1);
    hp = diff(tp);
    hc = hp(1:Np);
  elseif strcmp(DISTRIBUTION,'uniform2')     % Distribuzione uniforme con hp(1)=st
    a = (horizon-1)/(Np-1);
    tp= st*(1+a*((1:Np)-1));
    hp = diff([0 tp]);
    hc = hp;
  elseif strcmp(DISTRIBUTION,'quadratic')     % Distribuzione parabolica
    a = (horizon-1)/(Np-1)^2;
    b = -2*(horizon-1)/(Np-1)^2;
    c = 1+(horizon-1)/(Np-1)^2;
    tp = st*round(a*(1:Np).^2+b*(1:Np)+c);
    hp = diff([0 tp]);
    hc = hp;
  end
end

tp = cumsum(hp);              % cumulata degli intervalli hp
index_p = round(tp/st);          % indice relativo agli istanti di previsione
index_c = [0, round(cumsum(hc(1:end-1))/st)]; % indice relativo agli istanti di controllo

%% Modello dinamico del sistema
order = 2;                    % order of the model 1/s^order
s = tf('s');
sys = ss(1/s^order);          % definizione modello dinamico per singolo giunto
sys = xperm(sys,order:-1:1);  % riordina vettore stato q = [q qp qpp ...]'
[A,B,C,D] = ssdata(sys);  
[A,B,C,D] = siso2mimo(A,B,C,D,df); % genera sistema multi giunto nello spazio degli stati
sys = ss(A,B,C,D);
[Ad,Bd,Cd,Dd] = ssdata(c2d(ss(sys),st));  % sistema multi giunto a tempo discreto

syms t
expA = expm(A*t);                     % risp. libera
IexpAB = simplify(int(expA*B,t,0,t)); % risp. forzata

if length(symvar(expA))>0             % controllo se expA � una variabile simbolica
  expA=matlabFunction(expA);          % converto da simbolico a funzione
else
  expA=@(t) eval(expA);               % converto da simbolico a funzione
end
IexpAB=matlabFunction(IexpAB);        % converto da simbolico a funzione

As = A(1:order,1:order);
Bs = B(1:order,1);
Cs = C(1,1:order);
expAs = expm(As*t);                     % risp. libera
IexpABs = simplify(int(expAs*Bs,t,0,t)); % risp. forzata

if length(symvar(expAs))>0             % controllo se expA � una variabile simbolica
  expAs=matlabFunction(expAs);          % converto da simbolico a funzione
else
  expAs=@(t) eval(expAs);               % converto da simbolico a funzione
end
IexpABs=matlabFunction(IexpABs);        % converto da simbolico a funzione

% Calcolo matrici F e L per tutti gli intervalli fissi
[F,L]=FLmatrix(tp,hc,A,B,C,expA,IexpAB);
[Fs,Ls]=FLmatrix(tp,hc,As,Bs,Cs,expAs,IexpABs);

%% MPC constraints

Tau_max = repmat(tau_max,Ntau,1);
Tau_min = repmat(tau_min,Ntau,1);

ub = repmat(Qppmax,Np,1);
lb = repmat(Qppmin,Np,1);
  
Fc =  F(1:df*Np,1:df*Np,:);
Lc =  L(1:df*Np,:,:);
Abnd = Fc(:,:,2); % solo vel, no pos

clear b1

BU1 = repmat(Qpmax,Np,1);
BL1 = repmat(Qpmin,Np,1);
b2 = -Lc(:,:,2); % solo vel, no pos

% aumento il numero di variabili di ottimizzazione per task scaling
Ns = Np; 
Abnd = [Abnd, zeros(size(Abnd,1),Ns)];
lb = [lb;   0.00001*ones(Ns,1)]; % IMPO: lower bound su s
%ub = [ub;   0.99999*ones(Ns,1)];    % IMPO: upper bound su s
%ub = [ub;   1.15*ones(Ns,1)];
ub = [ub;  1.15*ones(Ns-1,1); 1.15];

%% Funzione di costo

Whp = diag(reshape(repmat(hp,df,1),df*Np,1)); % pesi dati dall'ampiezza degli intervalli di previsione
Whc = diag(reshape(repmat(hc,df,1),df*Np,1)); % pesi dati dall'ampiezza degli intervalli di controllo
Wsc = diag([lambdaTRJ(1)*ones(1,df*Np), lambdaTRJ(2)*ones(1,Ns)]); % IMPO: pesi dovuti a lambdaTRJ
Hu  = blkdiag(Whc,zeros(Ns,Ns));
Hp = eye(df*Np+Ns); 

% matrice PD serve per adattare le dimensioni di s a quelle di qd_des
PD=num2cell(ones(df,1,Np),[1 2]);  % converto la matrice 3d in una diagonale a blocchi
PD=cellfun(@(x) sparse(x),PD,'uni',0);
PD=blkdiag(PD{:});

% matrice PQ serve per adattare la q0 nella clik nel QP
PQ = [];
for i=1:df
  PQ = blkdiag(PQ,[1 0]);
end

% init

H = zeros(size(Abnd,2));
f = zeros(1,size(Abnd,2));
U = zeros(size(Abnd,2),1);
Hmat = zeros(df*Ntau);
Bmat = zeros(df*Ntau,1);

is_initialized=0;
fifo_iter=1;
