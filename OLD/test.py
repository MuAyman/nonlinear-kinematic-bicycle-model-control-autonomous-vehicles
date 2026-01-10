sample_time = 0.01
time_end = 30
model.reset()

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
w_data = np.zeros_like(t_data)
# delta_data = np.zeros_like(t_data)

# ==================================
#  Learner solution begins here
# ==================================

v_data[:] = 16/15*np.pi + 0.02 # m/s
# print(f'v_data: {v_data[1]}')
# delta_data[0:375] = 0.245 # rad
# delta_data[375:1875] = -0.245 # rad
# delta_data[1875:] = 0.245 # rad

transition_steps = 50

for i in range(1, len(t_data)):
#     model.delta = delta_data[i]
#     model.step(v_data[i],w_data_data[i])
    
    if i < 375 - transition_steps:
        if model.delta < np.arctan(2/8):  # rad
            w_data[i] = model.w_max
        else:
            model.delta = np.arctan(2/8)
            w_data[i] = 0
            
    elif i < 375 + transition_steps:
        if i == 375:
            model.theta = np.deg2rad(90)
        progress = (i - (375 - transition_steps)) / (2 * transition_steps)
        target_delta = np.arctan(2/8) * (1 - 2 * progress)  # From +max to -max
        
        if model.delta > target_delta:
            w_data[i] = -model.w_max
        elif model.delta < target_delta:
            w_data[i] = model.w_max
        else:
            w_data[i] = 0
    
    elif i < 1875 - transition_steps:
        if model.delta > -np.arctan(2/8):  # rad
            w_data[i] = -model.w_max
        else:
            w_data[i] = 0
            model.delta = -np.arctan(2/8)
            
    elif i < 1875 + transition_steps:
        if i == 1875:
            model.theta = np.deg2rad(90)
        progress = (i - (1875 - transition_steps)) / (transition_steps)
        target_delta = -np.arctan(2/8) * (1 - progress)  # From +max to -max
        
        if model.delta < target_delta:
            w_data[i] = model.w_max
        elif model.delta > target_delta:
            w_data[i] = -model.w_max
        else:
            w_data[i] = 0
    
    else:
        if i == 1870:
            model.theta = np.deg2rad(0)
        if model.delta > np.arctan(2/8):  # rad
            w_data[i] = model.w_max
        else:
            v_data[i] += 0.07  # Slightly increase speed to help close the loop
            w_data[i] = 0
            model.delta = np.arctan(2/8)
            
    model.step(v_data[i], w_data[i])
    x_data[i] = model.xc
    y_data[i] = model.yc

    
# ==================================
#  Learner solution ends here
# ==================================
plt.axis('equal')
plt.plot(x_data, y_data)
plt.grid()
plt.show_data()