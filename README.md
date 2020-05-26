# Control-System-Monitoring-And-Stabilization-On-MATLAB
PID Controller Design for an Inverted Pendulum on a Moving Cart on MATLAB

# Introduction
The Inverted pendulum is a classical control problem used for illustrating non-linear control techniques. The system is motivated by applications such as control of rockets and anti-seismic control of buildings. The inverted pendulum system consists of a pendulum which is attached to a cart that can move horizontally. The cart is driven by a motor that exerts a horizontal force on the cart. This force is the only control input to the system. By manipulating the force, the position of pendulum is to be controlled. The aim of the project is to control the pendulum at the balanced position vertically upwards. The balancing of pendulum in vertical position is unstable using an open loop system. So a feedback mechanism is required to balance the pendulum at the desired vertical position. By applying Newton’s law vertically and horizontally at the centre of gravity of the pendulum yields two equations which are then manipulated to get the required equations of motion. The state equation for the system is found out. The pendulum at vertical position ( = 0 ) is found to be unstable in open loop. The Non-linear state space equation is then linearized and checked for controllability. Using linearization a feedback law is designed. A simulation is run on Matlab program to plot the response of the system. The simulation change is observed when a 10% change on initial parameter values is applied. Using simulation the largest range of initial disturbance for which the pendulum will return to equilibrium is analysed.

# System Modeling and Transfer Function Synthesis
We assume a single-input single-output system. Since, we are attempting to control the pendulum's position, which should return to the vertical after the initial disturbance, the reference signal we are tracking should be zero, r = 0. The external force applied to the cart can be considered as an impulsive disturbance.
![system-modeling](https://github.com/singhishita/Control-System-Monitoring-And-Stabilization-On-MATLAB/blob/master/SystemModelling.PNG)

The pendulum is attached to the cart via a pivot on which the pendulum is allowed to rotate freely. In natural state without any external force the pendulum will stay down vertically downwards (an angle of 180 degrees). The pendulum is in stable equilibrium at this state. When the mechanical cart is moved from its initial position, it affects the rotation of the pendulum. 

The proposed project aim is in fact to stabilize the pendulum at its vertical upward position (angle of 0 degrees). The cart is driven by a motor which exerts a horizontal force F on the cart. This force is the only input to the system. The system is unstable in open condition. But with the implementation of a feedback mechanism, the pendulum can be stabilized. The forces acting on the pendulum are mg at the centre of gravity, a horizontal reaction force H and a vertical reaction force Vat the pivot.

Newtonian laws are followed to carry out the calculation for the transfer function of the system.

The resulting transfer function T(s) for the closed-loop system from an input of force F to an output of pendulum angle Φ is then determined to be the following:
T(s) = Φ(s)/ (s) = P(s) / (1+ C(s)P(s))
![system-design](https://github.com/singhishita/Control-System-Monitoring-And-Stabilization-On-MATLAB/blob/master/SystemDesign.PNG)

# Defining The PID Controller

C = pid(Kp,Ki,Kd,Tf) creates a continuous-time PID controller with proportional, integral, and derivative gains Kp, Ki, and Kd and first-order derivative filter time constant Tf:
C=Kp + Ki/s + Kds/(Tfs+1).

We define our controller using the pid object within MATLAB. Then, we use the feedback command to generate the closed-loop transfer function T(s) as depicted in the figure above where the disturbance force F is the input and the deviation of the pendulum angle from the vertical Φ is the output.
![PID](https://github.com/singhishita/Control-System-Monitoring-And-Stabilization-On-MATLAB/blob/master/PIDcontroller.PNG)

# Tuning PID Controller:
Here, we examine the response of the closed-loop system to an impulse disturbance for this initial set of control gains. Then the gains are tuned until we get a stabilized system response.

Moreover, at the end, we can also visualize that the cart moves in the negative direction with approximately constant velocity.
