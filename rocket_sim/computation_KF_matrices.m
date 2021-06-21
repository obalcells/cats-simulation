A = [0 1 0; 0 0 1; 0 0 0];
B = [0; 1; 0];
G = [0 0; 1 0; 0 0];
Ts = 0.01;
Ad = expm(A*Ts)
Bd = Ad*B
Gd = Ad*G


