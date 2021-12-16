import argparse
import pandas as pd
import json
import numpy as np

import data_converter
import data_faker

# python3 data_processor.py serialize --imu imu.csv [imu2.csv] --baro baro.csv

def parse_args():
    parser = argparse.ArgumentParser()

    commands = parser.add_subparsers(dest='command')
    commands.required = True

    serialize = commands.add_parser('serialize')
    serialize.add_argument('--imu', nargs='+', help='At least one IMU CSV file must be provided', required=True)
    serialize.add_argument('--baro', nargs='+', help='At least one Baro CSV file must be provided', required=True)
    serialize.add_argument('--acc', nargs='*', required=False)

    return parser.parse_args()

def check_input_files(args):
    # Requirements:
    # sensors start with 0 (so there can't be BARO1 if there's no BARO0)
    # correct columns (imu_csv's must include the correct columns)
    # all sensors of a certain type appear in the first 100 timesteps
    # once I improve the program I'll keep dropping file input requirements
    pass

# merges different csv files from the same sensor type ("IMU", "BARO")
def merge_dataframes(file_names, sensor_type):
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

def main():
    np.random.seed(0)

    args = parse_args()

    check_input_files(args)

    imu_data, number_of_imus = merge_dataframes(args.imu, "IMU")
    baro_data, number_of_baros = merge_dataframes(args.baro, "BARO")

    # imu_data.sort_values(by=['ts'], inplace=True)
    
    with open('sensor_config.json') as f:
        sensor_config = json.load(f)

    with open('noise-config.json') as f:
        noise_config = json.load(f)

    while number_of_imus < sensor_config['NUM_IMU']:
        imu_data = data_faker.fake_imu(imu_data, number_of_imus, noise_config)
        assert len(np.unique(imu_data['id'][:100])) == number_of_imus + 1
        number_of_imus += 1

    while number_of_imus > sensor_config['NUM_IMU']:
        imu_data = imu_data.iloc[imu_data['id'] != f'IMU{number_of_imus}']
        number_of_imus -= 1

    while number_of_baros < sensor_config['NUM_BARO']:
        data_faker.fake_baro(baro_data, noise_config)
        assert len(np.unique(baro_data['id'][:100])) == number_of_baros + 1
        number_of_baros += 1

    while number_of_baros > sensor_config['NUM_BARO']:
        baro_data = baro_data.iloc[baro_data['id'] != f'BARO{number_of_baros}']
        number_of_baros -= 1

    print("New IMU data")
    for column in imu_data[['Gx', 'Gy', 'Gz', 'Ax', 'Ay', 'Az']]:
        imu_data[column] = imu_data[column].apply(lambda x: data_converter.acc_to_si(x))
    imu_data.to_csv("converted_imu.csv")
    print(imu_data.head(20))

    for column in baro_data[['T', 'P']]:
        baro_data[column] = baro_data[column].apply(lambda x: data_converter.baro_to_si(x))
    baro_data.to_csv("converted_baro.csv")
    print(baro_data.head(20))

if __name__ == '__main__':
    main()
