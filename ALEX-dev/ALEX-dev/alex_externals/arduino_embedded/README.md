# Arduino Setup

## Serial Communication

Data communication with the Arduino will be conducted via serial protocol. Incoming data will consist of a continuous stream of characters, which will be parsed and processed to interact with the appropriate sensors or devices.

### Data Handling

1. **Receiving Data:**
   - The Arduino receives a continuous stream of characters through serial communication.
   - This data is parsed to identify the specific sensor or device providing the information, allowing the system to process the data accordingly.

2. **Sending Data:**
   - When sending data, it will be mapped to a predefined range to indicate the relevant sensor or device.
   - This mapping enables clear identification and processing of data from each sensor, ensuring accurate communication.

## Prerequisites

Before starting, make sure the necessary software is installed on your system. Run the following commands:

```bash
sudo apt update
sudo apt install arduino-mk
```

## Code Uploading

To upload code to the Arduino, follow these steps:

1. **Connect the Arduino**: Use a USB cable to connect the Arduino to your computer, ensuring a secure connection.
2. **Specify the Serial Number**: Ensure you have correctly entered the serial number in the [params](../params/alex_external.yaml) file.
3. **Navigate to the Code Directory**: Change to the directory where your Arduino code is located.
4. **Ensure Device Availability**: Confirm that the Arduino device is free and available for uploading.
5. **Run the Upload Script**:
   ```bash
   bash make_upload.sh
   ```
   Example: 
   ```bash
   'alex_externals/arduino_embedded/liquid_system'$ bash make_upload.sh 
   ```
6. **Verify Successful Upload**: Look for the message "avrdude done.  Thank you." in the terminal to confirm the code was uploaded successfully.

## About `make_upload.sh`

The `make_upload.sh` script is a versatile tool designed to simplify the process of compiling and uploading code to the Arduino. Here’s how it works and how you can reuse it in future projects:

1. **Serial Number Detection**: The script first checks the [params](../params/alex_external.yaml) file for the specific serial number associated with your Arduino device.

2. **Automatic Port Identification**: Using the detected serial number, the script identifies the correct port to which the Arduino is connected. This eliminates the need for manual port specification, streamlining the upload process.

3. **Code Compilation and Upload**: Once the port is identified, the script compiles the Arduino code and uploads it to the device, ensuring the code is transferred correctly and without manual intervention.

### Reusing `make_upload.sh` in New Projects

If you’re creating a new codebase and would like to set up the same automated upload process, simply copy the `make_upload.sh` script into the new project folder. As long as your new project also references the correct serial number in the [params](../params/alex_external.yaml) file, this script will handle the upload seamlessly.

This makes `make_upload.sh` a flexible tool that can be reused across different Arduino projects, saving setup time and reducing the potential for errors.
