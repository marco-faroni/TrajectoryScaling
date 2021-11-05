%% Function that permits to evaluate an inline function with vector entries
%  when the original function does not accept vectorization
%  fcn = inline function
%  x   = vector entry
%  fcn_vec = vectorized fcn

function fcn_vec = vectorizedFcn(fcn,x)
    fcn1 = fcn(x(:,1));
    rows = size(fcn1,1);
    cols = size(x,2);
    fcn_vec = [ fcn1,zeros(rows,cols-1) ];
    for i=2:1:cols
        fcn_vec(:,i) = fcn(x(:,i));
    end
end
        
        