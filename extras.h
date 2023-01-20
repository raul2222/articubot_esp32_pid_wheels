#include <EEPROM.h>
void reset_params();
void save_eeprom_data_2(int eeAddress);
void read_eeprom(int eeAddress);

void windup(){
  // si volt es positivo y error negativo
    if(abs(volt) > windup_value){
        
        windup_bool = true;       
       
    }else{
        windup_bool = false;
    }
    if((abs(volt) > windup_value and volt>0 and error <0) or (abs(volt) > windup_value and volt<0 and error >0)){
      windup_bool = true;  
    } else {
      windup_bool = false;
    }
}


void back_calculation(){
  if(abs(volt) > windup_value ){

        V_i_windup = volt / Tcb;

  }
  else{
    V_i_windup = 0;
  }
  
}

void windup_ang(){
  // si volt es positivo y error negativo
    if(abs(volt) > 3){
        volt=3;
        if (direccion == 0){
          volt=-3;
        }
        windup_bool = true;       
       
    }else{
        windup_bool = false;
    }
    if(abs(volt) > 3 and volt>0 and error <0 or volt<0 and error >0){
      windup_bool = true;  
    } 
}


void back_calculation_ang(){
  if(abs(volt) > 3 ){

        V_i_windup = volt / Tcb;

  }
  else{
    V_i_windup = 0;
  }
  
}

float ErrorPrecision = 1.9;
float ZM_min = 0.09;
float ZM_max = 0.50;

void zona_muerta(){
      int centinela = 0;
      if(volt > (-1)*ZM_max and volt < (-1)*ZM_min or error < ErrorPrecision*(-1)  ){
          centinela = 1;
          voltdeadzone=(-1)*ZM_max;
      } 
      if( volt < ZM_max and volt > ZM_min or error > ErrorPrecision   ) {
          centinela = 1;
          voltdeadzone=ZM_max;
      }
      if (centinela == 0){
          voltdeadzone = 0;
      }

}

void no_error_motor_break(){
      if(abs(error) < 1){
             //ANTES AL CALCULAR ERROR 
            //reset_params();
            clean();
     } 
}


void reset_params(){
  V_p = 0;
  V_i = 0;
  V_d = 0;
  V_i_windup=0;
  volt = 0;
  V_i_anterior = 0;
  fd = 0;
  fd_anterior = 0;
  da = 0;
  error_anterior = 0;
  digitalWrite(PWM_f, LOW); 
  digitalWrite(PWM_r, LOW);
  direccion = 0;
  direccion_ant = 0; 
  ang_cnt = 0;
  anterior = 0;
  pwm_motor = 0;
  v_medida = 0;
  a_medida = 0;    
}

void puesta_a_cero(){

  output=0;
  reset_params();
}
int aviso_vel = 0;
long timeto_vel = 0;
void no_error_motor_break_vel(){

     /*
    if(ref_val == 0 and abs(volt) < 0.1 and abs(error) < 1.2) {
          if (millis() > timeto_vel+100 and aviso_vel==1){
                reset_params();
                aviso_vel==0;
                digitalWrite(PWM_f, LOW); 
                digitalWrite(PWM_r, LOW);
                reset_params();
          }
        aviso_vel=1;
        timeto_vel = millis();
     } 
     */
}

////////////////////////////////////////////////////////////////////////////////////
// Funcion de lectura y grabación de las variables en la eeprom
////////////////////////////////////////////////////////////////////////////////////


void save_eeprom_data(){
    int eeAddress = 192;
    if (ACTIVA_P1C_MED_ANG == 1){
      eeAddress = 0;
    } else {
      eeAddress = 64;
    }
    
    struct K_x myconst;
    myconst.kp=K_p;
    myconst.ti=T_i;
    myconst.td=T_d;
        myconst.kp=Kp;
    myconst.ti=Ki;
    myconst.td=Kd;
    myconst.tcb=Tcb;
    myconst.alpha=alpha;
    EEPROM.put(eeAddress, myconst);
    EEPROM.commit();
    #ifdef ACTIVA_DEBUG
    Serial.print("write:");Serial.println(ACTIVA_P1C_MED_ANG);
    Serial.print(" dir:");Serial.println(eeAddress);
    #endif
 
}

int init_eeprom(){
    int eeAddress = 192;
    if (ACTIVA_P1C_MED_ANG == 1){
      eeAddress = 0;
    } else {
      eeAddress = 64;
    }
    
    EEPROM.begin(EEPROM_SIZE);
    struct K_x myconst;
    EEPROM.get(eeAddress, myconst);
    
    //K_p = myconst.kp;
    //T_i = myconst.ti;
    //T_d = myconst.td;
    Kp = myconst.kp;
    Ki = myconst.ti;
    Kd = myconst.td;
    //Tcb = myconst.tcb;
    //alpha = myconst.alpha;
    #ifdef ACTIVA_DEBUG
    Serial.print("read:");Serial.println(ACTIVA_P1C_MED_ANG);
    Serial.print(" dir:");Serial.println(eeAddress);
    #endif
}

void save_eeprom_data_2(int eeAddress){ // for backup propouses
    struct K_x myconst;
    myconst.kp=K_p;
    myconst.ti=T_i;
    myconst.td=T_d;
    

    
    myconst.tcb=Tcb;
    myconst.alpha=alpha;
    EEPROM.put(eeAddress, myconst);
    EEPROM.commit();
    Serial.print("write:");Serial.println(ACTIVA_P1C_MED_ANG);
    Serial.print(" dir:");Serial.println(eeAddress);
}

void read_eeprom(int eeAddress){

    EEPROM.begin(EEPROM_SIZE);
    struct K_x myconst;
    EEPROM.get(eeAddress, myconst);
    Serial.println(myconst.kp);
    Serial.println(myconst.ti);
    Serial.println(myconst.td);
    Serial.println(myconst.tcb);
    Serial.println(myconst.alpha);

    Serial.print("read:");Serial.println(ACTIVA_P1C_MED_ANG);
    Serial.print(" dir:");Serial.println(eeAddress);
}
