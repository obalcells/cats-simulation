import pandas as pd
import json
import numpy as np
import converter
import faker

# checks that data input has certain properties using assertions
def check_input_files(imu_data: pd.DataFrame, number_of_imus: int, baro_data: pd.DataFrame, number_of_baros: int) -> None:
    # correct columns (imu_csv's must include the correct columns)
    for column in ['ts', 'id', 'Gx', 'Gy', 'Gz', 'Ax', 'Ay', 'Az']:
        assert column in imu_data.columns

    for column in ['ts', 'id', 'T', 'P']:
        assert column in baro_data.columns 
    
    # if there's N IMU sensors then IMU0, IMU1, ..., IMU(N-1) must be present
    imu_sensors = np.unique(imu_data['id']) 
    imu_sensors = sorted(imu_sensors)

    assert len(imu_sensors) == number_of_imus

    for sensor_idx in range(number_of_imus):
        assert imu_sensors[sensor_idx] == f'IMU{sensor_idx}' 

    baro_sensors = np.unique(baro_data['id'])
    baro_sensors = sorted(baro_sensors)

    assert len(baro_sensors) == number_of_baros

    for sensor_idx in range(number_of_baros):
        assert baro_sensors[sensor_idx] == f'BARO{sensor_idx}' 

    # all sensors of a certain type appear in the first 100 timesteps
    for sensor in [f'IMU{sensor_idx}' for sensor_idx in range(number_of_imus)]:
        assert sensor in list(imu_data['id'][:100])

    for sensor in [f'BARO{sensor_idx}' for sensor_idx in range(number_of_baros)]:
        assert sensor in list(baro_data['id'][:100])

# reads and merges different csv files from the same sensor type ("IMU", "BARO")
def read_and_merge_data(file_names: list[str], sensor_type: str) -> tuple[pd.DataFrame, int]:
    assert sensor_type == "IMU" or sensor_type == "BARO"
    sensor_columns = {'IMU': ['ts', 'id', 'Gx', 'Gy', 'Gz', 'Ax', 'Ay', 'Az'],
                      'BARO': ['ts', 'id', 'T', 'P']}
    df = pd.DataFrame(columns=sensor_columns[sensor_type])
    number_of_sensors = 0
    for file in file_names:
        tmp_df = pd.read_csv(file, usecols=sensor_columns[sensor_type])
        number_of_sensors_here = len(np.unique(tmp_df['id'][:100])) 
        for sensor_idx in range(number_of_sensors_here):
            tmp_df.replace(f'{sensor_type}{sensor_idx}', f'{sensor_type}{number_of_sensors + sensor_idx}', inplace=True)
        df = df.append(tmp_df)
        number_of_sensors += number_of_sensors_here
    return df, number_of_sensors

def process_data(args):
    # np.random.seed(0)

    # Step 1: Read the data
    imu_data, number_of_imus = read_and_merge_data(args.imu, "IMU")
    baro_data, number_of_baros = read_and_merge_data(args.baro, "BARO")

    # Step 2: Check that data is well formatted
    check_input_files(imu_data, number_of_imus, baro_data, number_of_baros)
    
    # Step 3: Add or delete sensors to adjust for sensor-config.json requirements
    with open('sensor_config.json') as f:
        sensor_config = json.load(f)

    with open('noise-config.json') as f:
        noise_config = json.load(f)

    while number_of_imus < sensor_config['NUM_IMU']:
        imu_data = faker.fake_imu(imu_data, number_of_imus, noise_config)
        assert len(np.unique(imu_data['id'])) == number_of_imus + 1
        number_of_imus += 1

    while number_of_imus > sensor_config['NUM_IMU']:
        imu_data = imu_data.iloc[imu_data['id'] != f'IMU{number_of_imus}']
        number_of_imus -= 1

    while number_of_baros < sensor_config['NUM_BARO']:
        baro_data = faker.fake_baro(baro_data, number_of_baros, noise_config)
        assert len(np.unique(baro_data['id'])) == number_of_baros + 1
        number_of_baros += 1

    while number_of_baros > sensor_config['NUM_BARO']:
        baro_data = baro_data.iloc[baro_data['id'] != f'BARO{number_of_baros}']
        number_of_baros -= 1

    # Step 4: Convert sensor measurements
    for column in imu_data[['Gx', 'Gy', 'Gz', 'Ax', 'Ay', 'Az']]:
        imu_data[column] = imu_data[column].apply(lambda x: converter.acc_to_si(x))

    for column in baro_data[['T', 'P']]:
        baro_data[column] = baro_data[column].apply(lambda x: converter.baro_to_si(x))

    # Step 5: Adjust timestamps
    imu_data.sort_values(by=['ts'], inplace=True)
    baro_data.sort_values(by=['ts'], inplace=True)

    imu_ts = list(imu_data['ts'])
    first_ts_imu = imu_ts[0]
    last_ts_imu = imu_ts[-1]
    imu_change = last_ts_imu - first_ts_imu

    baro_ts = list(baro_data['ts'])
    first_ts_baro = baro_ts[0]
    baro_change = baro_ts[-1] - first_ts_baro

    baro_ts[0] = first_ts_imu
    for i in range(1, len(baro_ts)):
        delta = baro_ts[i] - first_ts_baro
        delta = (delta / baro_change) * imu_change
        baro_ts[i] = baro_ts[0] + delta

    baro_data['ts'] = pd.Series(baro_ts)

    # Step 6: Merge all dataframes into one
    output_df = pd.merge(imu_data, baro_data, how="outer")

    # Step 7: Sort by timestamp
    output_df.sort_values(by=['ts'], inplace=True)

    # Step 8: Save in csv file
    if args.out != None:
        output_df.to_csv(args.out)
    else:
        output_df.to_csv("processed_data.csv")
