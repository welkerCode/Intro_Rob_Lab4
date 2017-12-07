
a1 = 69;
a2 = 370.82;

d1 = 270.35;
d4 = 374.29;
dtool = 368.3;

a = [a1 a2];
d = [d1 d4 dtool];
Tw_tool = [0.984 0.177 0.024 727.089; 0.177 -0.984 0.020 418.581; 0.027 -0.015 -0.999 732.233; 0 0 0 1]



thetas = ikinelbow(a,d,Tw_tool)

a = [a1 a2 0 0 0 0 0]
d = [d1 0 0 d4 0 0 0]
alpha = [-pi/2 0 -pi/2 pi/2 -pi/2 0 0]

T_final = fkine(a, d, alpha, thetas)

