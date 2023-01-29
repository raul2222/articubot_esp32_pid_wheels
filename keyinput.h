/* 
Tarea de configuración de parámetros  #####################################################################
*/
void task_config(void *pvParameter) {
    char ini_char = '0';

    while(1) { 
        // Detectar caracter enviado
       if(Serial.available() > 0){
            String str = Serial.readStringUntil('\n');
            ini_char = str[0];
            if(str.indexOf("V") == 0 or str.indexOf("v") == 0){
                str.replace("V","");str.replace("v","");str.replace(",",".");
                
                pwm_volt = str.toFloat();
                Serial.print("Voltaje motor = ");
                Serial.println(pwm_volt, 2);
                //ref_val = pwm_volt; estabamos usando esta forma
                
                //puesta_a_cero();
                
                
                ACTIVA_P1C_MED_ANG = 0;
                //init_eeprom();
                #ifdef ACTIVA_DEBUG
                Serial.print("----Modo Velocidad----");
                #endif
                
            }
            if (str.indexOf("R") == 0  or str.indexOf("r") == 0) {
                  str.replace("R", "");str.replace("r", "");str.replace(",",".");
                  ref_val = str.toFloat();
                  start_stop = 1;
                  if( ACTIVA_P1C_MED_ANG == 1 ){
                    #ifdef ACTIVA_DEBUG
                      Serial.print("Angulo= ");
                      Serial.print(ref_val, 2);
                      Serial.println("º");
                    #endif
                  }else{

                      
                      #ifdef ACTIVA_DEBUG
                      Serial.print("Velocidad= ");
                      Serial.print(ref_val);
                      Serial.println("rps");
                      #endif
                  //}
                  //Serial.print(Kp);
                  //Serial.print(ref_val);
                  //Serial.println("rps");
                  //init_params();
                  setpoint = ref_val;
                  
              }
              if(str.indexOf("A") == 0 or str.indexOf("a") == 0){
                    str.replace("A", "");str.replace("a", "");str.replace(",",".");
                    ACTIVA_P1C_MED_ANG = 1;
                    
                    //puesta_a_cero();
                    //init_eeprom();
                    #ifdef ACTIVA_DEBUG
                    Serial.println("----Modo Angulo----");
                    #endif
              }

              if(str.indexOf("Z") == 0 or str.indexOf("z") == 0){
                    str.replace("Z", "");str.replace("z", "");str.replace(",",".");
                    start_stop = 1;
                    #ifdef ACTIVA_DEBUG
                    Serial.println("--START--");
                    #endif
                    
              }
              
              
              if(str.indexOf("S") == 0 or str.indexOf("s") == 0){
                  str.replace("S", "");str.replace("s", "");str.replace(",",".");
                  if(str == "1"){
                      start_stop = 1;
                      
                      #ifdef ACTIVA_DEBUG
                      Serial.println("--START--");
                      #endif
                  }
                  else{
                      start_stop = 0;
                      #ifdef ACTIVA_DEBUG
                      Serial.println("--STOP--");
                      #endif
                  }
                  puesta_a_cero();
              }
              if(str.indexOf("P") == 0 or str.indexOf("p") == 0  ){
                  str.replace("P",""); str.replace("p","");str.replace(",",".");
                  //K_p = str.toFloat();
                  Kp = str.toFloat();
                  Serial.println(Kp);
                
                  
              }
              if(str.indexOf("I") == 0 or str.indexOf("i") == 0){
                  str.replace("I","");str.replace("i","");str.replace(",",".");
                  //T_i = str.toFloat();
                  Ki = str.toFloat();
                  Serial.println(Ki);
                
                
              }
              if(str.indexOf("D") == 0 or str.indexOf("d") == 0){
                  str.replace("D","");str.replace("d","");str.replace(",",".");
                  //T_d = str.toFloat();
                  Kd = str.toFloat();
                  Serial.println(Kd);
                  
              }
              if(str.indexOf("N") == 0 or str.indexOf("n") == 0){  // Tcb
                  str.replace("N","");str.replace("n","");str.replace(",",".");
                  N = str.toInt();
                  //save_eeprom_data();
              }
              if(str.indexOf("X") == 0 or str.indexOf("x") == 0){  // Tcb
                  str.replace("X","");str.replace("x","");str.replace(",",".");
                  clean();
                  ang_cnt=0;
                  //save_eeprom_data();
              }
              if(str.indexOf("F") == 0 or str.indexOf("f") == 0){  // Tcb
                  str.replace("F","");str.replace("f","");str.replace(",",".");
                  fd_bool = str.toInt();
                  //save_eeprom_data();
              }
              if(str.indexOf("U") == 0 or str.indexOf("u") == 0){  // Tcb
                  str.replace("U","");str.replace("u","");str.replace(",",".");
                  alpha = str.toFloat();
                  //save_eeprom_data();
              }
              if(str.indexOf("W") == 0 or str.indexOf("w") == 0){  // Tcb
                  str.replace("W","");str.replace("w","");str.replace(",",".");
                  windup_bool = str.toInt();
                //save_eeprom_data();
              }
              if(str.indexOf("T") == 0 or str.indexOf("t") == 0){  // Tcb
                  str.replace("T","");str.replace("t","");str.replace(",",".");
                  Tcb = str.toFloat();
                  //save_eeprom_data();
              }

              setpoint = ref_val;
        }

        // Activacion de la tarea cada 0.1s
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
}
