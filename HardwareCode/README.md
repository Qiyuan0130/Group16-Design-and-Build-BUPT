# Smart Car Control System

A comprehensive autonomous car control system built with STM32 Nucleo F446RE, featuring Bluetooth communication, IMU-based navigation, and precision motor control.

## Overview

This project implements a smart car with the following key capabilities:
- **Bluetooth Control**: Remote control via Bluetooth commands
- **IMU Navigation**: MPU6500-based orientation and movement control
- **Precision Motors**: PID-controlled dual-motor system with encoders
- **Distance Control**: Accurate distance-based movement
- **Radar Integration**: Optional radar sensor support
- **Odometry**: Real-time position and speed tracking

## Hardware Requirements

- **MCU**: STM32 Nucleo F446RE
- **IMU**: MPU6500 (6-axis accelerometer + gyroscope)
- **Motors**: Dual DC motors with encoders
- **Communication**: Bluetooth module
- **Optional**: Radar sensor for obstacle detection

## Project Structure

```
Main/
├── src/                    # Source code
│   ├── main.cpp           # Main control loop
│   ├── Bluetooth.cpp/h    # Bluetooth communication
│   ├── IMU.cpp/h          # MPU6500 IMU driver
│   ├── Motor.cpp/h        # Motor control with PID
│   ├── Encoder.cpp/h      # Encoder reading
│   ├── PID.cpp/h          # PID controller
│   └── Radar.cpp/h        # Radar sensor
├── test/                  # Test scripts
│   ├── bt_test_sequence_improved.py
│   ├── test_straight_accuracy.py
│   └── test_startup_stability.py
├── platformio.ini         # PlatformIO configuration
└── README.md              # This file
```

## Key Features

### Control Commands
- `F` - Forward movement
- `S` - Stop
- `Lp` - Left turn 90°
- `Rp` - Right turn 90°
- `M<distance>` - Move specific distance (0-10m)
- `Y` - Get current yaw angle
- `D` - Get distance traveled
- `Q` - Get current position (X,Y)
- `C` - Correct current angle
- `W` - Get robot speed
- `Z` - Reset distance counter

### Advanced Features
- **Startup Stabilization**: Prevents motor twitching during initialization
- **Dynamic PID Tuning**: Adjusts parameters for short/long distance movements
- **Odometry Tracking**: Real-time position and speed calculation
- **Angle Correction**: Automatic yaw angle correction
- **Distance Control**: Precise movement to target distances

## Software Dependencies

- **PlatformIO**: For building and uploading
- **Arduino Framework**: For STM32 development
- **Madgwick Library**: For IMU sensor fusion
- **Python 3**: For test scripts
- **Bluetooth Library**: For communication testing

## Building and Uploading

1. Install PlatformIO
2. Open the project in PlatformIO
3. Connect STM32 Nucleo F446RE via USB
4. Build and upload:
   ```bash
   pio run --target upload
   ```

## Testing

The project includes comprehensive test scripts:

### Basic Movement Test
```bash
python bt_test_sequence_improved.py --port COM5
```

### Straight Line Accuracy Test
```bash
python test_straight_accuracy.py --port COM5 --distance 2.0 --trials 3
```

### Startup Stability Test
```bash
python test_startup_stability.py --port COM5 --wait 10
```

## Configuration

### Serial Communication
- **Baud Rate**: 115200 (USB), 9600 (Bluetooth)
- **Port**: COM7 (configurable in platformio.ini)

### Motor Control
- **Control Frequency**: 25 Hz
- **IMU Update**: 100 Hz
- **Radar Reading**: 1 kHz

### PID Parameters
The system uses separate PID controllers for:
- Left motor speed control
- Right motor speed control  
- Yaw angle control

## Usage Examples

### Basic Movement
1. Connect via Bluetooth
2. Send `F` to start forward movement
3. Send `S` to stop
4. Send `Lp` for left turn
5. Send `Rp` for right turn

### Distance Control
1. Send `M` followed by distance (e.g., `M2.5` for 2.5 meters)
2. Car will move exactly 2.5 meters and stop
3. Short distances (< 1m) use reduced speed for better accuracy

### Status Monitoring
- Send `Y` to get current angle
- Send `D` to get distance traveled
- Send `Q` to get current position
- Send `W` to get current speed

## Troubleshooting

### Common Issues
1. **Motor Twitching on Startup**: Normal behavior during 2-second stabilization period
2. **Bluetooth Connection**: Ensure correct COM port and baud rate
3. **IMU Calibration**: System auto-calibrates on startup
4. **Distance Accuracy**: Use `C` command to correct angle drift

### Debug Commands
- Monitor serial output at 115200 baud
- Check Bluetooth communication with test scripts
- Verify IMU readings with `Y` command

## License

This project is open source. Feel free to modify and distribute according to your needs.

## Contributing

Contributions are welcome! Please ensure:
- Code follows existing style
- Test scripts are updated
- Documentation is maintained
- Hardware compatibility is verified
