A PID controller design using Nichols Ziegler Tuning Method

Terminal Output for the included transfer function

L=: 3.6606
T=: 5.451
K=: 1

Kp = (1.2 * T) / (K * L) = 1.7869
Ki = (2*L) = 7.3212
Kd = (0.5 *  L) = 1.8303

pid_controller =
 
  23.94 s^2 + 13.08 s + 1.787
  ---------------------------
            7.321 s
 
Continuous-time transfer function.

Plots:
![plots](https://github.com/ZOBARCIK/NicholsZieglerPID/assets/119112572/9617b4a0-aed2-4225-8d44-a42b65680f3e)
