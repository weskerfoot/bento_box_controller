### Bambu Printer air filter controller

This is the code for my air filtration control system. It is designed to integrate with a Bambu printer and control an air filtration system, e.g. a Bento Box air filter

It uses two sensors, one for air temperature and humidity, and one for VOCs.
The temperature/humidity sensor is used to calibrate the VOC sensor.

It is designed to allow you to to integrate with the MQTT server built into
Bambu printers.

It also has a JSON based HTTP api which lets you configure thresholds for when
the air filter will run (which is controlled by MOSFETs)

### Configure the project
Run `idf.py menuconfig` and set the variables in the fan controller config
section.

E.g.
WiFI SSID: <your wifi SSID>
WiFI Password: <your wifi password>
Maximum WiFI Retry Count: 10
Broker URI: `mqtts://bblp:<password>@192.168.0.186:8883`
Device Serial Number: <your serial number> (can be obtained from the printer itself)

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view the serial output:

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.
