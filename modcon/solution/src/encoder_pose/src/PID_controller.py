#!/usr/bin/env python
# coding: utf-8

# In[6]:


import numpy as np

# Heading control

# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your PIDController function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

# TODO: write your PID function for heading control!

def PIDController(v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t):
    """
    Args:
        v_0 (:double:) linear Duckiebot speed (given).
        theta_ref (:double:) reference heading pose
        theta_hat (:double:) the current estiamted theta.
        prev_e (:double:) tracking error at previous iteration.
        prev_int (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    
    # TODO: these are random values, you have to implement your own PID controller in here
    
    e = theta_ref-theta_hat
    # derivative of the error
    e_der = (e - prev_e)/delta_t
    
    e_int = prev_int + e * delta_t
    # integral of the error
    e_int = prev_int + e*delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,2),-2)
    
    k_p = 1.5
    k_i = 0.1
    k_d = 0.10
    # solution values
    k_p = 5
    k_i = 0.2
    k_d = 0.1
    
    omega = k_p*e+k_i*e_int+k_d*e_der
    #print(f'theta_hat={theta_hat} theta_ref={theta_ref} e={e} e_der={e_der} e_int={e_int} omega={omega}')
    
    return [v_0, omega], e, e_int

