# Liquid System Control

This project controls a liquid dispensing system using an Ezo Pump Controller and an Ultrasonic Sensor to monitor liquid levels. The system communicates via serial commands and performs periodic updates to both the pump and the sensor.


## Wiring Details
To set up the hardware for this project, use the following wiring details:

### Ezo Pump Controller:

- RX (Receive): Connect to digital pin `PUMP_TX_PIN` (pin 3) on the Arduino.
- TX (Transmit): Connect to digital pin `PUMP_RX_PIN` (pin 2) on the Arduino.
- GND: Connect to the Arduino ground pin.
- Power (VCC): Connect to the `3.3V` power output from the Arduino.

### Ultrasonic Sensor:

- Trigger Pin: Connect to `US_TRIG_PIN` (pin 4) on the Arduino.
- Echo Pin: Connect to `US_ECHO_PIN` (pin 5) on the Arduino.
- GND: Connect to the Arduino ground pin.
- Power (VCC): Connect to the `5V` output from the Arduino.

Ensure that all grounds are connected to avoid potential issues with signal transmission.

## System Components

### EzoPumpController Class

The `EzoPumpController` class manages communication with the Ezo Pump, a peristaltic pump that dispenses liquids in precise quantities. This class uses a software serial interface to communicate with the pump, sending commands to control dispensing and receiving feedback on the pump’s status.

- **Constructor (`EzoPumpController(uint8_t rx, uint8_t tx)`)**: 
   
   Initializes the pump’s serial communication using specified RX (receive) and TX (transmit) pins, enabling a dedicated communication line between the Arduino and the pump. This ensures that pump data does not interfere with the main serial port, which is used for system monitoring and debugging.

- **begin(long baudRate)**: 
   Sets the baud rate for pump communication. The baud rate must match the pump’s expected communication speed to ensure commands and responses are accurately transmitted.

- **processCommand(const String &command)**: 
   
   Sends a command string to the pump, appending a carriage return (`\r`) to signify the end of the command. This method enables the main program or user to control pump operations (like starting or stopping) dynamically by specifying commands via the Serial Monitor.

- **update()**: 
   
   Checks if there is any response data available from the pump, indicating the completion of a previous command. If a full response has been received, it returns that response as a string; otherwise, it returns an empty string. This enables non-blocking communication with the pump, as `update()` can be called repeatedly without holding up the main program.

- **handleDeviceResponse()** (private): Reads and accumulates incoming data from the pump’s serial interface. When a carriage return (`\r`) is received, it marks the response as complete. This internal method organizes and finalizes pump responses, which the `update()` method then retrieves for system use.

### UltrasonicSensor Class

The `UltrasonicSensor` class provides functionality for measuring liquid levels by calculating the distance from the sensor to the liquid’s surface. The class uses trigger and echo pins to initiate and measure ultrasonic pulses, allowing it to determine the distance to the liquid accurately.

- **begin()**: 
   
   Sets up the sensor by configuring the specified trigger and echo pins. This initialization prepares the sensor for distance measurements and is typically called in the main setup phase.

- **getDistance()**: 
   
   Sends an ultrasonic pulse from the sensor’s trigger pin, then measures the time it takes for the echo to return. Based on this round-trip time, it calculates and returns the distance to the liquid’s surface in centimeters. This distance can then be used to determine the liquid level, which is essential for making refill or stop decisions.

### Main Code (liquid_system.ino)

This main Arduino sketch integrates and manages the EzoPumpController and UltrasonicSensor classes, coordinating their operations to ensure proper liquid dispensing and monitoring.

- **update_usensor()**: 
   
   Measures the liquid level every 5 seconds by calling the getDistance() method on the ultrasonic sensor. If a valid distance is returned, it is printed to the Serial Monitor with the prefix LL: to indicate liquid level readings. This periodic check ensures the system always has an up-to-date measurement of the liquid level.

- **update_pump()**: 
   
   Invokes the update() method on the pump controller, which checks for any available responses from the pump. If a response is present, it is printed to the Serial Monitor. This allows real-time monitoring of pump operations and error checking.

- **serialEvent()**: 
   
   Listens for input commands from the Serial Monitor and forwards these commands to the pump controller for execution. This enables direct user control over the pump, allowing for real-time adjustments to pump behavior.

- **setup()**: 
   
   Configures the Serial Monitor, initializes the pump controller at its specified baud rate, and prepares the ultrasonic sensor. This setup is essential for initializing communication and ensuring all components are ready to operate.

- **loop()**: 
   
   Continuously calls update_pump() and update_usensor() to ensure the system remains responsive and up-to-date. This loop structure allows the program to operate both the pump and the sensor concurrently without blocking behavior.

## Usage

- The system continuously monitors the liquid level and prints the readings to the Serial Monitor every 5 seconds.
- It also periodically checks for responses from the pump controller and prints those to the Serial Monitor as well.
- You can send commands to the pump controller via the Serial Monitor. Any valid command sent will be passed to the pump controller for processing.
- [Here](https://files.atlas-scientific.com/EZO_PMP_Datasheet.pdf) is the document for all the commands supported by EzoPump
