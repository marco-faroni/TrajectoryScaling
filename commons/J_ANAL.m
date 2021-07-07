function Ja = J_ANAL(q,M,Ta,rows,cols)
  
  xreal = FKIN(q,M);
  Ja = Ta(xreal)\jacNS16(q,1:6);
  Ja = Ja(rows,cols);
  
end