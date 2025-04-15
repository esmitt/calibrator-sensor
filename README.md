# Calibrator-Sensor

This repository contains the implementation for assessing sensor orientation in the quaternion domain, as presented in the paper "An Interpretable Assessment of Sensor’s Orientation in the Quaternion Domain", published in the 2021 International Conference on Indoor Positioning and Indoor Navigation (IPIN).

## Overview

The `calibrator-sensor` project provides tools to evaluate the orientation of Magnetic and Inertial Measurement Units (MIMUs) using quaternion-based metrics. Unlike traditional Euler angle-based assessments, this approach offers a more reliable and interpretable way to measure orientation errors in terms of rotation axis and angle, avoiding issues like gimbal lock and error propagation inherent in Euler angles.

## Features

- **Quaternion-Based Assessment**: Computes orientation errors using quaternion distances for improved reliability.
- **Comparison with Euler Angles**: Includes tools to compare quaternion-based metrics against Euler angle-based metrics.
- **Experimental Validation**: Supports analysis of rotational trajectories, as tested with a Stäubli robotic arm.

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/esmitt/calibrator-sensor.git
   cd calibrator-sensor
   ```

2. **Dependencies**: Ensure you have the following installed:
   - Python 3.6+
   - NumPy
   - SciPy
   - Matplotlib (for visualization)

   Install dependencies using pip:
   ```bash
   pip install -r requirements.txt
   ```

3. **Hardware Requirements**:
   - A MIMU sensor (e.g., MPU-6050, MPU-9250, or similar) for data collection.
   - A setup for controlled rotations (e.g., a robotic arm or turntable) for experimental validation.
   - A microcontroller or interface (e.g., Arduino, Raspberry Pi) to collect sensor data.

## Usage

1. **Data Collection**:
   - Connect your MIMU sensor to the host device.
   - Collect raw data (accelerometer, gyroscope, magnetometer) during controlled rotational trajectories.
   - Save the data in a compatible format (e.g., CSV or text files).

2. **Orientation Assessment**:
   - Run the quaternion-based assessment script:
     ```bash
     python assess_orientation.py --input data/sample_data.csv --output results.json
     ```
   - The script computes orientation errors using quaternion distances and optionally compares them to Euler angle-based errors.

3. **Visualization**:
   - Visualize the results to compare quaternion and Euler angle metrics:
     ```bash
     python visualize.py --input data/sample_data.csv --results results.json
     ```

## Example

```bash
# Collect MIMU data and save to data.csv
# Run orientation assessment
python assess_orientation.py --input data.csv --output results.json
# Visualize quaternion vs. Euler angle metrics
python visualize.py --input data.csv --results results.json
```

## Citation

If you use this code in your research, please cite the associated paper:

```bibtex
@INPROCEEDINGS{GILRAMIREZ2021,
  author={Gil Resina, Debora and Navarrete, Manuel and Ramírez, Esmitt and Sanchez Ramos, Carles and García Calvo, Carlos and Castells-Rufas, David},
  booktitle={2021 International Conference on Indoor Positioning and Indoor Navigation (IPIN)}, 
  title={An Interpretable Assessment of Sensor’s Orientation in the Quaternion Domain}, 
  year={2021},
  pages={1-8},
  doi={10.1109/IPIN51156.2021.9662635}
}
```

## Notes

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

Contributions are welcome! Please open an issue or submit a pull request for bug fixes, improvements, or new features.

For questions or support, contact the authors at [esmittramirez@gmail.com](mailto:esmittramirez@gmail.com) or open an issue on this repository.
