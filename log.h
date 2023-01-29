/*
  Tarea imprimir por terminal del controlador  #####################################################################
*/



void task_medidas(void* arg){
  int contador =0;
  float tv_medida=0;
  float toutput = 0;
  int limite=10;
  while (1) {

/*

    if (start_stop == 1) {  

      if (contador ==limite){
       // Mostrar medidas de angulo y velocidad del motor
        Serial.print("D: ");
        Serial.print(dutyCycle);
        Serial.print(" C: ");
        Serial.print(ang_cnt);

        if ( ACTIVA_P1C_MED_ANG == 1 ) { // Medida de angulo
          a_medida = (ang_cnt * 360) / flancos;
          Serial.print(" M: ");
          Serial.print(a_medida);
          Serial.print(" A: ");
          Serial.print(ref_val, 2);

        } else { // Medida de velocidad
          Serial.print(" Med: ");
          //Serial.print(v_medida);
          Serial.print(tv_medida/limite);
          Serial.print(", Ref: ");
          Serial.print(ref_val, 2);
        }
        Serial.print(", V: ");
      //Serial.print(output, 2);
        Serial.print(toutput/limite, 3);

        Serial.print(" Kp:");
        Serial.print(Kp, 3);
        Serial.print(" Ki:");
        Serial.print(Ki, 3);
        Serial.print(" Kd:");
        Serial.print(Kd, 3);
        Serial.println(" ");
        tv_medida=0;
        toutput=0;
        contador=0;
      } else {
        //Acumular
                    
        tv_medida = tv_medida + v_medida;
        toutput = toutput + output;
        contador++;
      }
    }

   vTaskDelay(BLOQUEO_TAREA_MEDIDA_MS / portTICK_PERIOD_MS);*/
  }
}



void task_adc(void* arg)
{
  while (1) {
    /*
    if (start_stop == 1) {
        adc0 = ads.readADC_SingleEnded(0);
        //current = (adc0 * 0.1875)/1000;
        current = ads.computeVolts(adc0)/0.5;

    } else {

    }
*/
    // Activacion de la tarea cada 1s
    vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
  }

}
