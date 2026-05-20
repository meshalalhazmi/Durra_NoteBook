| Supported Targets |   ESP32-S3 |
| ----------------- |   -------- |

# Durra's Notebook 


 
### Hardware Required

* A development board with any supported Espressif SOC chip (see `Supported Targets` table above)
* A USB cable for power supply and programming

### Configure the Project
This project uses custom partition table, make sure to configure partition.csv based on the selected MCU. 
### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT build flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
 
# Conversion Cmmand
ffmpeg -i input.mp3 -ar 16000  -ac 1 -c:a pcm_s16le -t 5  output.wav