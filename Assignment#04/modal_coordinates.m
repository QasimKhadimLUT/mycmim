function cp = modal_coordinates (t,p)


M_p = diag([1,1,1]);
K_p = diag([2.6e+03,1.3e+04,2.7e4]);

P=[p(1);p(2);p(3)];
eom=M_p\(-K_p*P); %eqqwefgbnh

cp(1)=  p(4);
cp(2)=  p(5);
cp(3)=  p(6);
cp(4)=eom(1);
cp(5)=eom(2);
cp(6)=eom(3);
cp=cp(:);