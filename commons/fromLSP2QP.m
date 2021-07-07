%% 

%ccc

syms F u Dq s L x0 delta lambda real sref sold
U = [u;s];

eqn = (F*u - (Dq)*s + L*(x0))^2 + lambda*(s-1)^2;

display('funzione di costo')
pretty(eqn)

%eqn = (u-L*x0+F*s)^2

H = hessian(eqn,U)/2;
f = simplify(gradient(eqn,U)-2*H*U)/2;

%cond(H)
display('H = ')
pretty(H)
display('f = ')
pretty(f)