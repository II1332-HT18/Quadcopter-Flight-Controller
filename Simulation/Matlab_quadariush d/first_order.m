function quad = first_order(r,y)

    kp = 7;
    ki = 0.3;
    kd = 0.54;

    G1(s) = (kp + ki/s + kd*(s));
    G2(s) = 3;
    
    E(s) = r(s) - G2(2)*y(s);
    y(s) = G1(s)*E(s);

    quad = (G1(s))/(1 + G1(s)*G2(s));
    
end
    
     




