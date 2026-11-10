# EchoCore

A USB-powered, two-channel, low-noise acquisition board for biopotentials (EEG/ECG). Two differential inputs with a shared reference go through a high-impedance analog front end into a 24-bit delta-sigma ADC at 64× gain, and the samples stream to a computer over USB. An onboard IMU records motion for artifact removal.

## Specs

- 2 differential channels, shared reference
- 24-bit ΔΣ ADC (TI ADS131M02), 64× gain, 250 SPS per channel
- 0.42 µVrms input-referred noise (datasheet)
- ~1–100 Hz band, >100 MΩ input impedance
- USB 2.0 Full Speed (CDC) over USB-C, bus-powered
- 6-DOF IMU

> Not a medical device. The inputs are not isolated from USB.
