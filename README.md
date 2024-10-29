# trackitpod

A patent-pending self-tracking tripod for recording water sports

For a demo of the product, please utilize the following modules with 2 Arduino NANOs as the transmitter and receiver:

**Transmitter**
- HC-12 Radio transmitter
- NEO-6M or GT-U7 GPS module
- 2 28BYJ-48 step motors (pan and tilt)
- 3D model for base encasing found over "model" directory

**Receiver**
- HC-12 Radio transmitter
- NEO-6M or GT-U7 GPS module

To test the code, simply upload the TrackIt_transmitter.ino file to the transmitting Arduino NANO and the TrackIt_receiver.ino to the receiving microcomputer.

The ESP-32 version for micropython is still under development and all code in the repo is still in draft level.
