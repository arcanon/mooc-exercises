#!/usr/bin/env python
# coding: utf-8

# In[140]:


import numpy as np

# Lateral control

# TODO: write the PID controller using what you've learned in the previous activities

# Note: y_hat will be calculated based on your DeltaPhi() and poseEstimate() functions written previously 

def PIDController(
    v_0, # assume given (by the scenario)
    y_ref, # assume given (by the scenario)
    y_hat, # assume given (by the odometry)
    prev_e_y, # assume given (by the previous iteration of this function)
    prev_int_y, # assume given (by the previous iteration of this function)
    delta_t): # assume given (by the simulator)
    """
    Args:
        v_0 (:double:) linear Duckiebot speed.
        y_ref (:double:) reference lateral pose
        y_hat (:double:) the current estiamted pose along y.
        prev_e_y (:double:) tracking error at previous iteration.
        prev_int_y (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e_y (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int_y (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    
    # TODO: these are random values, you have to implement your own PID controller in here
    omega = np.random.uniform(-8.0, 8.0)
    e_y = y_ref - y_hat
    e_der = (e_y - prev_e_y)/delta_t    
    e_int_y = prev_int_y + e_y * delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int_y = max(min(e_int_y,0.06),-0.06)
    
    k_p = 0.01
    k_i = 0.05
    k_d = 5.0
    
    #quite good
    #k_p = 3
    #k_i = 0.5
    #k_d = 10.0
    
    #if e_y < 0.01:
    #    e_y = 0
    
    e_y_2 = e_y
    if (e_y_2 < 0):
        e_y_2 = -2**(-e_y_2*0.5)-1
    else:
        e_y_2 = 2**(e_y_2*0.5)-1
    
    #e_y**3 is pretty ood
    omega = k_p*e_y+k_i*e_int_y+k_d*e_der
    #print(f'omega={omega}')
    
    return [v_0, omega], e_y, e_int_y

