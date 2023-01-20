
#define NOMBRE_PRAC "ARTICUBOT-ESP32"
#define VERSION_SW "0.1"

// Choose the wheel
//#define LEFT
#define RIGHT

#define ACTIVA_DEBUG
#define ACTIVA_R
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "global.h"
#include "extras.h"
#include "keyinput.h"
#include "1-loopcontr.h"
#include "log.h"
#define SDA_PIN 21
#define SCL_PIN 22
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = 0;

#ifdef LEFT ////////////////

#define I2C_SLAVE_ADDR_L 0x01
void receiveEven_l(int howMany);
void requestEvent_l();
String data = "";
String data_enc = "";

void receiveEvent_l(int howMany)
{ 
  lastMotorCommand = millis();  
  data = "";
  while( Wire.available()){
    data += (char)Wire.read();
    
  }
  if(data == "r"){
    clean();
  } else {
    Serial.println(data.toFloat());
    ref_val = setpoint = data.toFloat();
    start_stop=1;
  }
}

void requestEvent_l()
{
  data_enc = String(ang_cnt);
  char char_array[data_enc.length()+1];
  data_enc.toCharArray(char_array, data_enc.length()+1);
  Wire.write(char_array);
  //Serial.println(char_array);
}

#endif /////////////////////



#ifdef RIGHT /////////////////

#define I2C_SLAVE_ADDR_R 0x02
void receiveEven_r(int howMany);
void requestEvent_r();
String data = "";
String data_enc = "";

void receiveEvent_r(int howMany)
{ 
  lastMotorCommand = millis();  
  data = "";
  while( Wire.available()){
    data += (char)Wire.read();
    
  }
  if(data == "r"){
    clean();
  } else {
    Serial.println(data.toFloat());
    ref_val = setpoint = data.toFloat();
    start_stop=1;
  }
}

void requestEvent_r()
{
  data_enc = String(ang_cnt);
  char char_array[data_enc.length()+1];
  data_enc.toCharArray(char_array, data_enc.length()+1);
  Wire.write(char_array);

}

#endif //////////////

#include "setup.h"

void loop() {

  // LED BUILD IN
  if (start_stop){
    digitalWrite(2,HIGH);
  } else {
    digitalWrite(2,LOW);
  }

  // TIMEOUT CONTROL
  if (millis()>(AUTO_STOP_INTERVAL + lastMotorCommand) and lastMotorCommand > 0){
    ref_val = setpoint = 0;
  }

  delay(1);
  
}

// FUNCIONES ////////////////////////////////////////////////////////////////////////

void excita_motor(float v_motor){
    
    // Obtención de la dirección
    if(v_motor > 0){
        direccion = 1;
    }else{
        direccion = 0;
    }
    if(direccion_ant != direccion){ //("Cambio de sentido");

    }
    #ifdef LEFT
        if(v_motor > 0){    
            digitalWrite(PWM_f, HIGH);
            //digitalWrite(PWM_r, LOW);
        }
        if(v_motor < 0){      
            v_motor = abs(v_motor);// valor en positivo del voltaje el cambio de direccion lo hacen las variables
            digitalWrite(PWM_f, LOW);
            //igitalWrite(PWM_r, HIGH);
       }
    #endif
    #ifdef RIGHT
        if(v_motor > 0){     
            digitalWrite(PWM_f, LOW);
            //digitalWrite(PWM_r, LOW);
        }
        if(v_motor < 0){     
            v_motor = abs(v_motor);// valor en positivo del voltaje el cambio de direccion lo hacen las variables
            digitalWrite(PWM_f, HIGH);
            //igitalWrite(PWM_r, HIGH);
        }
    #endif

    direccion_ant = direccion;

    dutyCycle = (int) ((v_motor * PWM_Max)/12);

    if(dutyCycle >= PWM_Max){
        dutyCycle = PWM_Max;
    }
    if(dutyCycle <= 0){
        dutyCycle = 0;
    }
  	
  	// DAC INSTEAD LEDC
  	//ledcWrite(0, dutyCycle);
    dacWrite(DAC2,dutyCycle); 
}  
