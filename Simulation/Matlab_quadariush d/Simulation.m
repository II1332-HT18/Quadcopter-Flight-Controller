clear all; close all; clc

t = linspace(0,10);
y0 = 0.5;
u = 1.0;

z = ode23(@(r,y)first_order(t,y,r),t,y0);
time = z.x;
y = z.x;

plot(time,y); 

  