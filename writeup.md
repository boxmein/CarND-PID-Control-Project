# PID Controller

This is my writeup addressing rubric points for the PID Controller Project.

## PID component effects

When tuning the three parameters of the controller, it quickly became apparent which effect each of them had on the
car's steering.

I started off by using manual tuning, but quickly (after 20 minutes of trying) I realized that I should automate the
process. Implementing ~~coordinate descent~~ Twiddle was quite easy and I could do other things as the simulator itself
figured out how to optimize the parameters.

### The effect of the P factor

The P factor `Kp` made the car's steering react quicker but also steer with a sharper angle, when increased. My
optimizer ended up reducing the Kp factor to keep the car under control. The main reason for that is, that the increased
Kp factor also contributes to the car oversteering and oscillating around the ideal center line.

Kp: 0.0632701

This is usually balanced off by...

### The effect of the D factor

The D factor `Kd` made the car's steering react to a change in the error, mainly by making turns softer as the car
got closer to the center line. My optimizer ended up giving quite a heavy weight to the D factor, since the derivative
of the error quickly increases during a turn.

Kd: 3.68445

### The effect of the I factor

The I factor Ki made the car's steering take care of any systematic bias in the entire control chain. If the car would
steer a bit off every time, the integral factor would be larger.

In this case, however, the optimizer resulted in a minuscule value for the I factor, meaning that the car did not have
much systematic bias.

Ki: 0.00097335
