import pandas as pd
import numpy as np
import json

# Questions:
# - Can/should I apply some noise for the timestamps?
# - we add (offset + noise) to all the sensors except for temperature?

# List of things that we can tweak that might uncover errors:
# - One of the first sensors appears for the first time in a very late timestamp
# - We send no data from one of the sensors
# - We send data from an extra sensor

def fake_imu(data, number_of_imus, noise_config):
    # we will clone the data of one of the IMUs and add random noise
    imu_idx_to_clone = np.random.randint(0, number_of_imus)

    # we clone all the rows of the IMU that we have selected
    new_data = data.loc[data['id'] == f'IMU{imu_idx_to_clone}'].copy(deep=True)
    number_of_rows = len(new_data)

    new_data['id'] = f'IMU{number_of_imus}'

    # Which standard deviation should I add to the timestamps?!?
    # new_data['ts'] += np.around(np.random.np.random.normal(0, 0.05, number_of_rows), decimals=3)

    offset = np.random.uniform(low = noise_config['ANGULAR_VELOCITY']['offset_lb'], high = noise_config['ANGULAR_VELOCITY']['offset_up'], size = 1)
    noise = np.random.normal(loc = 0, scale = noise_config['ANGULAR_VELOCITY']['std'], size = number_of_rows) 
    new_data['Gx'] += noise + offset

    offset = np.random.uniform(low = noise_config['ANGULAR_VELOCITY']['offset_lb'], high = noise_config['ANGULAR_VELOCITY']['offset_up'], size = 1)
    noise = np.random.normal(loc = 0, scale = noise_config['ANGULAR_VELOCITY']['std'], size = number_of_rows) 
    new_data['Gy'] += noise + offset

    offset = np.random.uniform(low = noise_config['ANGULAR_VELOCITY']['offset_lb'], high = noise_config['ANGULAR_VELOCITY']['offset_up'], size = 1)
    noise = np.random.normal(loc = 0, scale = noise_config['ANGULAR_VELOCITY']['std'], size = number_of_rows) 
    new_data['Gz'] += noise + offset

    offset = np.random.uniform(low = noise_config['LINEAR_ACC']['offset_lb'], high = noise_config['LINEAR_ACC']['offset_up'], size = 1)
    noise = np.random.normal(loc = 0, scale = noise_config['LINEAR_ACC']['std'], size = number_of_rows) 
    new_data['Ax'] += noise + offset

    offset = np.random.uniform(low = noise_config['LINEAR_ACC']['offset_lb'], high = noise_config['LINEAR_ACC']['offset_up'], size = 1)
    noise = np.random.normal(loc = 0, scale = noise_config['LINEAR_ACC']['std'], size = number_of_rows) 
    new_data['Ay'] += noise + offset

    offset = np.random.uniform(low = noise_config['LINEAR_ACC']['offset_lb'], high = noise_config['LINEAR_ACC']['offset_up'], size = 1)
    noise = np.random.normal(loc = 0, scale = noise_config['LINEAR_ACC']['std'], size = number_of_rows) 
    new_data['Az'] += noise + offset

    return data.append(new_data)


def fake_baro(data, number_of_baros, noise_config):
    # we will clone the data of one of the IMUs and add random noise
    baro_idx_to_clone = np.random.randint(0, number_of_baros)

    # we clone all the rows of the BARO that we have selected
    new_data = data.loc[data['id'] == f'BARO{baro_idx_to_clone}'].copy(deep=True)
    number_of_rows = len(new_data)

    new_data['id'] = f'BARO{number_of_baros}'

    # Which standard deviation should I use for the timestamps?!?
    # new_data['ts'] += np.around(np.random.np.random.normal(0, 0.05, number_of_rows), decimals=3)

    offset = np.random.uniform(low = noise_config['PRESSURE']['offset_lb'], high = noise_config['PRESSURE']['offset_up'], size = 1).astype(int)
    noise = np.random.normal(loc = 0, scale = noise_config['PRESSURE']['std'], size = number_of_rows).astype(int)
    new_data['P'] += noise + offset

    return data.append(new_data)
