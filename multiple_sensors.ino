/*------------------------------------------------------------I/O explain 
MQ_2          analog_pin-A0      //Ammonia
KY-038        analog_pin-A1      //MIC
HC_SR501      digital_pin-3      //PIR motion sensor NOT INSTALLED
BH1750        I2C SDA-A4 SCL-A5  //LightMeter
BMP180        I2C SDA-A4 SCL-A5  //Baro Temp
HTU21D        I2C SDA-A4 SCL-A5  //Humi Temp
--------------------------------------------------------------I/O explain
By:   yuhang@tarsbot.com
Date: 20160321
*/

#include <Wire.h>
#include <ros.h>
#include <dependant_api/all_sensor.h>
#include "Statistic.h"

#include "SFE_BMP180.h"
#include "HTU21D.h"
#include "BH1750.h"

#define Sensor_Ammonia  A0
#define Sensor_Noise  A1

dependant_api::all_sensor  data_msg;
ros::Publisher chatter("/robot/env_sensor", &data_msg);
ros::NodeHandle  nh;

//Create an instance of the object
Statistic myStats; 

HTU21D myHumidity;
SFE_BMP180 pressure;
BH1750 lightMeter;

void setup()
{
  //Serial.begin(9600);
  nh.initNode();
  nh.advertise(chatter);
  
  myStats.clear();
  pinMode(Sensor_Ammonia,INPUT);
  myHumidity.begin();
  pressure.begin();
  lightMeter.begin();
}

void loop()
{
  char status;
  double T,P,p0,a;
  double lux = lightMeter.readLightLevel();
  float humd = myHumidity.readHumidity();
  float temp = myHumidity.readTemperature();
  int Ammonia = analogRead(Sensor_Ammonia);
  //int Noise= analogRead(Sensor_Noise);
  //int data_noise[1000];
  status = pressure.startTemperature();
  
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

        }
        else;// Serial.println("error retrieving pressure measurement\n");
      }
      else;// Serial.println("error starting pressure measurement\n");
    }
    else;// Serial.println("error retrieving temperature measurement\n");
  }
  else;// Serial.println("error starting temperature measurement\n");
  
//  int noise_max = 0;
//  int noise_min = 1024;
//  int i;
//  for(i=0;i<1000;i++){
//    data_noise[i] = analogRead(Sensor_Noise);
//    noise_max = max(noise_max,data_noise[i]);
//    noise_min = min(noise_min,data_noise[i]);
//  }
  myStats.clear();
  int i;
  for(i=0;i<100;i++){
    myStats.add(analogRead(Sensor_Noise));
  }
  int noise_level = 50*log10(myStats.pop_stdev())+60;
  
  
//  int noise_amp = noise_max - noise_min;
  data_msg.humidity= humd;
  data_msg.temperature= temp;
  data_msg.smoke=Ammonia;
  data_msg.illumination=lux;
  data_msg.human=0;
  data_msg.noise= noise_level;
  chatter.publish( &data_msg );
  nh.spinOnce();
  
  
  
  delay(3000);
}
