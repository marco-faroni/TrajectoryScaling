% Dato un sistema Xp = A*X+B*u; Y = C*X nello spazio degli stati, genera un
% nuovo sistema Xp = Am*X+Bm*u; Y = Cm*X che rappresenta un sistema MIMO
% che ripete il sistema iniziale n volte.
% Lo stato e l'uscita del nuovo sistema saranno dei vettori colonna dati
% dalla concatenazione degli stati e delle uscite dei singoli sistemi.
% Il vettore u del nuovo sistema sarà il vettore colonna composto dagli
% ingressi dei singoli sistemi.

function [Am, Bm, Cm, Dm] = siso2mimo(A,B,C,D,n)

  ACell = repmat({A}, 1, n);  
  Am = blkdiag(ACell{:});
  BCell = repmat({B}, 1, n);  
  Bm = blkdiag(BCell{:});
  CCell = repmat({C}, 1, n);
  Cm = blkdiag(CCell{:});
  DCell = repmat({D}, 1, n);
  Dm = blkdiag(DCell{:});

end