# Thermostat-Arduino
 Thermostat for refrigerator Beko CN335220 on Arduino Leonardo
 Native display replaced by OLED_I2C SSD1306 128x64

 ## Info
 When the mains voltage is turned on - a pause of 10 minutes until the engine is turned on, regardless of temperature.
When disconnected, re-enable no earlier than 10 minutes, regardless of temperature.
When restarting - a pause of 10 minutes until the engine is turned on, regardless of temperature.
If the compressor runs for more than 1 hour - forced stop regardless of temperature.
If the temperature sensor is open or defective, the compressor is running
in a cycle (20 min work / 20 min rest), while the alarm LED lights up
