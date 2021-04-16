# Stm32_i2c_with_MPU6050

![Screenshot from 2021-04-16 12-05-43](https://user-images.githubusercontent.com/29373058/115057414-a8296180-9f01-11eb-8fb1-2367a9d82a99.png)
### The clock tree used in the code

The reasons whenthe MPU6050 sensor might not be working:
- Initially the sensor/module may not be powered up with appropiate/specified value, in this case we can verify it using voltage and current measuring devices.
- The PIN connection should be appropiate with the dedicated microcontroller/host device so that the sensor and the host is in same platform to interact with each other.
Generally this happens a lot in the prototype stage where we use jumpers to connect with different modules.
- The configuration of each and every register should be properly verfied. This is most time taking place where developers seems to gets stucked. 
- The developer should keep in mind the steps before reading data from the sensor that all the prerequisites of reg config is done.
- Another point is if the sensor module gets detected from the master device or not so that it can proceed with further steps.
- It needs to make sure that the sensor is in its operating range.
- Another issue seen in this sensor is noise spikes which might distract the original purpose or reliability in fetching values, in this case proper filtering algorithms needs to be used to get a consistent level of readings.

### Thank you
