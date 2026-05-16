zeta=0.1; Wc=2*pi*30 ; T =1/200;

% notch filter TF
Hs = tf([(1/Wc)^2 0 1], [(1/Wc)^2 2*zeta*(1/Wc) 1]) ;

Wc_bar = (2/T)*tan((T/2)*Wc) ;
Hs_bar = tf([(1/Wc_bar)^2 0 1], [(1/Wc_bar)^2 2*zeta*(1/Wc_bar) 1]) ;

Hz = c2d(Hs_bar, T, 'tustin');

figure;
hold on, grid on;
bode(Hs), bode(Hz);
legend('CT', 'DT');
