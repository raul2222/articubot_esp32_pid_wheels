
/*
SET UP -----------------------------------------------------------------------------------
*/


void setup() {
  // Configuracion puerto serie
  config_sp();
  digitalRead(A_enc_pin);

  // Configuracion PWM
  config_PWM();
  Kp=0.6;
  Ki=0.008;
  Kd=0.1;
  start_stop=1;
  flancos = 834;

  setpoint = ref_val = 0;

  N=9;

  dt = (BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
  // Configuracion PWM
  //config_ADC();

  // Crear cola_enc
  cola_enc = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
  if(cola_enc == NULL){
      Serial.println("Error en creacion de cola_enc");
      exit(-1);
  }
   //Crear la tarea task_enc
  if(xTaskCreatePinnedToCore( task_enc , "task_enc", 2048, NULL, 5, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_enc");
      exit(-1);
  }
/*
  // Crear la tarea task_config
  if(xTaskCreatePinnedToCore( task_config , "task_config", 2048, NULL, 1, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_config");
      exit(-1);
  }
*/
  // Crear la tarea task_loopcontr
  if(xTaskCreatePinnedToCore( task_loopcontr , "task_loopcontr", 2048, NULL, 2, NULL,1) != pdPASS){
      Serial.println("Error en creacion tarea task_loopcontr");
      exit(-1);
  }
/*
   // Crear la tarea task_medidas
  if(xTaskCreatePinnedToCore( task_medidas , "task_medidas", 2048, NULL, 3, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_medidas");
      exit(-1);
  }
*/
  // Crear la tarea task_adc
  /*if(xTaskCreatePinnedToCore( task_adc , "task_adc", 2048, NULL, 3, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_medidas");
      exit(-1);
  }*/

  // Configuracion del encoder
  config_enc();
  config_wire();

}



/*
 RUTINAS ATENCION INTERRUPCIONES ########################################################################
*/
/* 
 Rutina de atención a interrupción ISC_enc --------------------------------------------
*/
uint8_t r = 0;
void IRAM_ATTR ISR_enc() {
  // Lee las salidas del Encoder    
  //uint8_t a = digitalRead(A_enc_pin);
  
  if(r == 0 ){
      r ++;
  } else {
      r--;
  }
  
  // Enviar los bytes a la cola 
  if (xQueueSendFromISR( cola_enc , &r ,NULL) != pdTRUE)
  {
      printf("Error de escritura en la cola cola_enc \n");
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del adc
////////////////////////////////////////////////////////////////////////////////////

void config_ADC(){
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  /*if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }*/
}  


////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del encoder
////////////////////////////////////////////////////////////////////////////////////

void config_enc(){
    // Configuracion de pines del encoder
    pinMode(A_enc_pin, INPUT_PULLUP);
    //pinMode(B_enc_pin, INPUT);
    // Configuracion interrupcion
    attachInterrupt(A_enc_pin, ISR_enc, CHANGE);
    //attachInterrupt(B_enc_pin, ISR_enc, CHANGE);
} 

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del PWM
////////////////////////////////////////////////////////////////////////////////////

void config_PWM(){
    // Configuracion de pines de control PWM
    pinMode(PWM_f, OUTPUT);
    digitalWrite(PWM_f, LOW);
    pinMode(2, OUTPUT);digitalWrite(2, HIGH);
    dacWrite(DAC2, 0);
    // Configuracion LED PWM 
    //ledcSetup(pwmChannel, pwmfreq, pwmresolution);
    // Asignar el controlador PWM al GPIO
    //ledcAttachPin(PWM_Pin, 0);
}  

void config_wire(){
   #ifdef LEFT ///////////////

    bool success = Wire.begin(I2C_SLAVE_ADDR_L);
    if (!success) {
        Serial.println("I2C slave init failed");
        while(1) delay(100);
    }
    Wire.onReceive(receiveEvent_l);
    Wire.onRequest(requestEvent_l);
    #endif/////////////////

    #ifdef RIGHT//////////////
    bool success = Wire.begin(I2C_SLAVE_ADDR_R);
    if (!success) {
        Serial.println("I2C slave init failed");
        while(1) delay(100);
    }
    Wire.onReceive(receiveEvent_r);
    Wire.onRequest(requestEvent_r);
    #endif/////////////
}

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del puerto serie
////////////////////////////////////////////////////////////////////////////////////
void config_sp(){

    Serial.begin(115200);
    while (!Serial){
      delay(10);
    }
    Serial.println("  ");
    Serial.println("--------------------------------------------");
    Serial.println("******  PID RELEASE  ******");
    Serial.println("--------------------------------------------");
  
}  

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del OLED
////////////////////////////////////////////////////////////////////////////////////
void config_oled(){
  /*
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C) ;
    display.clearDisplay();
    display.setTextColor(WHITE);        // 
    display.setCursor(0,0);             // Start at top-left corner
    display.println(F("CONTR. MOTOR " NOMBRE_PRAC));
    display.display();
    delay(1000);
    display.setTextColor(BLACK,WHITE);        // 
    display.setCursor(0,20);             // Start at top-left corner
    display.println(F(" SW v." VERSION_SW));
    display.display();
    delay(1000);
   */
}  


////////////////////////////////////////////////////////////////////////////////////
// Funcion de interpolacion LUT de Velocidad-Voltaje
////////////////////////////////////////////////////////////////////////////////////

float interpola_vel_vol_lut(float x) {
    // Buscar el valor superior más pequeño del array
    int8_t i = 0;
    if ( x >= Vel_LUT[LONG_LUT - 2] ) {i = LONG_LUT - 2;}
    else {while ( x > Vel_LUT[i+1] ) i++;}
  
    // Guardar valor superior e inferior
    float xL = Vel_LUT[i];
    float yL = Vol_LUT[i];
    float xR = Vel_LUT[i+1];
    float yR = Vol_LUT[i+1];
  
    // Interpolar
    float dydx = ( yR - yL ) / ( xR - xL );
  
    return yL + dydx * ( x - xL );
}
