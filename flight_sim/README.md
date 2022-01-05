## To-do's
-  Specify angular acceleration units
-  Specify temperature and pressure units
-  Accelerometer data section

Flight Sim is a small Python application which can process sensor data and feed it to the CATS flight computer (FC) to simulate flights.

There's 2 different things that are done within the application. First, processing the input data to match what the CATS FC expects to receive, and second, sending this data to the CATS FC.

## Processing data
The flight will be simulated based on sensor data which is either 'fake' or from some other flight. This data is passed as different CSV files.  The `process_data` function formats the data, adds or deletes sensors to match the number of sensors of a certain type expected by the FC, and merges all the data into one CSV file.

Some facts:
-  For each sensor type you must specify which CSV files correspond to that sensor.
-  There can be multiple CSV files for the same sensor.
-  Every CSV file for each sensor must have certain columns. The other columns will be ignored.
-  All the sensor measurements must be converted to SI units. You can specify your own conversions in `converter.py`. More details below on this.

```
python3 cli.py process --imu imu_data.csv imu_data2.csv --baro baro_data.csv --out processed_data.csv
```

#### Data conversion
**Important!** The data that's passed in as input will be converted to SI units based on the functions in `converter.py`.

If your flight data to be simulated is already in SI units you should change those functions so that they just return what you input into them. Otherwise the processed data will be incorrect!

#### Timestamps
**Important!** The timestamps of the different sensors must be adjusted ad-hoc to be on the same scale.

In the sample data the timestamps of the IMU data are in seconds (see IMU data section) and the barometer timestamps are in miliseconds (see barometer data section).

Therefore the program adjusts the barometer timestamps to fit those of the IMU by looking at the difference between the first measurement and the last one of both data types.

```python
first_ts_imu = imu_ts[0] # timestamp of first IMU measurement
last_ts_imu = imu_ts[-1] # timestamp of last IMU measurement
imu_change = last_ts_imu - first_ts_imu

first_ts_baro = baro_ts[0] # timestamp of first Baro measurement
last_ts_baro = baro_ts[-1] # timestamp of last Baro measurement
baro_change = last_ts_baro - first_ts_baro

baro_ts[0] = first_ts_imu
for i in range(1, len(baro_ts)):
	delta = baro_ts[i] - first_ts_baro
    delta_scaled = (delta / baro_change) * imu_change
    baro_ts[i] = baro_ts[0] + delta_scaled

baro_data['ts'] = baro_ts
```

This conversion doesn't work well if you stop getting measurements from one sensor much earlier than from the IMU sensor or viceversa+.

#### IMU data
Expected columns: `['ts', 'id', Gx', 'Gy', 'Gz', 'Ax', 'Ay', 'Az']`

- `ts`: Timestamp
	- Input units: Seconds
	- Output units: Seconds
- `id`: ID of the sensor. It must be of the form `IMU0`, `IMU1`, ...
- `Gx`, `Gy`, `Gz`: Gyroscope
	- Input units: Anything you want as long as you specify the conversion in `converter.py`
	- Output units:  (SI units)
- `Ax`, `Ay`, `Az`: Accelerometer
	- Input units: Anything you want as long as you specify the conversion in `converter.py`
	- Output units: $m / s^2$ (SI units)

This is how the IMU data from our sample flight looks like:

| ts     | id   | Gx                   | Gy                  | Gz                   | Ax            | Ay           | Az            |
|--------|------|----------------------|---------------------|----------------------|---------------|--------------|---------------|
| -0.331 | IMU0 | 0.9146341463414634   | 0.853658536585366   | 0.30487804878048785  | -0.0576171875 | 0.9921875    | -0.04296875   |
| -0.331 | IMU1 | 0.0                  | 0.3658536585365854  | -0.1829268292682927  | -0.0390625    | 0.998046875  | -0.0830078125 |
| -0.321 | IMU0 | 0.9146341463414634   | 0.6707317073170732  | 0.24390243902439027  | -0.0576171875 | 0.9921875    | -0.0439453125 |
| -0.321 | IMU1 | 0.0                  | 0.48780487804878053 | -0.24390243902439027 | -0.0380859375 | 0.9990234375 | -0.0849609375 |
| -0.311 | IMU0 | 1.0975609756097562   | 0.6097560975609757  | 0.24390243902439027  | -0.056640625  | 0.9931640625 | -0.0419921875 |
| -0.311 | IMU1 | -0.06097560975609757 | 0.6707317073170732  | -0.24390243902439027 | -0.037109375  | 0.998046875  | -0.083984375  |

In this sample our units aren't in SI

#### Barometer data
Expected columns: `['ts', 'id', 'T', 'P']`

- `ts`: Timestamp
- `id`: ID of the sensor. It must be of the form `BARO0`, `BARO1`, ...
- `T`: Temperature
	- Input units: Anything you want as long as you specify the conversion in `converter.py`
	- Output units: ... (SI units)
- `P`: Pressure
	- Input units: Anything you want as long as you specify the conversion in `converter.py`
	- Output units: ... (SI units)

This is how the barometer sample data looks like:

| ts      | id    | T    | P     |
|---------|-------|------|-------|
| 2289131 | BARO0 | 2131 | 98044 |
| 2289131 | BARO1 | 2145 | 98010 |
| 2289131 | BARO2 | 2231 | 98020 |
| 2289141 | BARO0 | 2131 | 98047 |
| 2289141 | BARO1 | 2145 | 98004 |
| 2289141 | BARO2 | 2230 | 98022 |
| 2289151 | BARO0 | 2131 | 98053 |


#### Accelerometer data (Currently not supported!)
Expected columns: `['ts', 'id', 'Ax', 'Ay', 'Az']`

## Sending data
To be written later...

Pyserial, USB, cats-embedded

## Building
To be written later...
