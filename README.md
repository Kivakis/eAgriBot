# eAgriBot
MATLAB/Simulink 2D model of a compact battery electric agricultural robot pulling a tine harrow

You can select from 5 models in the first section of the .m script:
1. Rear-wheel drive (RWD) with 40/60 weight distribution
2. RWD with 30/70 weight distribution
3. Front-wheel drive (FWD) with 60/40 weight distribution
4. FWD with 70/30 weight distribution
5. All-wheel drive with 50/50 weight distribution

Two soil types available:
1. Sandy loam
2. Clayey loam

Efficiency map used is not included due to copyright. You may add your own efficiency map. It requires three arrays:
1. effmap (n x m)
-This matrix contains the efficiency values (within the range 0.0-1.0)
2. T_effmap (1 x m)
-This vector contains the torque values (Nm) used by the efficiency map
3. w_effmap (1 x n)
-This vector contains the rotational speed values (rad/s) used by the efficiency map
