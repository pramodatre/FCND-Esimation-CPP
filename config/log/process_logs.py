import pandas as pd
import numpy as np

def compute_std_gps_sensor():
    # process GPS log file Gaph1.txt
    gps_data = pd.read_csv('Graph1.txt')
    print(gps_data)
    print(gps_data.columns)
    standad_deviation = np.std(gps_data[' Quad.GPS.X'])
    print(f'Standard deviation of Quad.GPS.X: {standad_deviation}')

def compute_std_imu_sensor():
    # process acceleration log file Graph2.txt
    gps_data = pd.read_csv('Graph2.txt')
    print(gps_data)
    print(gps_data.columns)
    standad_deviation = np.std(gps_data[' Quad.IMU.AX'])
    print(f'Standard deviation of Quad.IMU.AX: {standad_deviation}')

if __name__ == '__main__':
    compute_std_gps_sensor()
    compute_std_imu_sensor()