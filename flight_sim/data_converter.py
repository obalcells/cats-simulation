
def imu_to_si(input :float):
    return input * (9.81 * 1024)

def magnometer_to_si(input : float):
    return input

def baro_to_si(input :float):
    return input

def gyro_to_si(input :float):
    return input * (1.0 / 16.4)
