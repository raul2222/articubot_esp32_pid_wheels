

#define RXD2 16
#define TXD2 17

#define EEPROM_SIZE 512
#define EE_1 0
#define EE_2 64
#define EE_3 128
#define ACTIVA_P1A
//#define DEBUG_P1A
#define ACTIVA_P1B1
#define ACTIVA_P1B2
#define ACTIVA_P1B3
#define ACTIVA_P1C
#define DEBUG_P1C
#define ACTIVA_P1D2
#define ACTIVA_P1D3
// Display OLED ///////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// Parametros cola de la interrupcion del encoder ///////////////////////////////////////
#define TAM_COLA_I 1024 /*num mensajes*/
#define TAM_MSG_I 1 /*num caracteres por mensaje*/

// TIEMPOS
#define BLOQUEO_TAREA_LOOPCONTR_MS 10
#define BLOQUEO_TAREA_MEDIDA_MS 250

int dutyCycle =0;

float volt_max = 12.0;
float volt_min = 1.05;
float flancos = 0;
float Akpi=0;
float Akp=0;
float error_2=0; // e(t-2)
float error_1 = 0; // e(t-1)
float error_0 = 0; // e(t)
float output = 0;  
float A0d=0;
float A1d=0;
float A2d=0;
float N=5;
float tau = 0; // IIR filter time constant
float alpha = 0;
float d0 = 0;
float d1 = 0;
float fd0 = 0;
float fd1 = 0;
float Kp = 0;
float Ki = 0;
float Kd = 0;
float setpoint = 0;
float dt = (BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);;
float z_i = 0;
float z_d = 0;
float Tiuser = 0;
float Tduser = 0;
float Ti = 0;
float Td = 0;

// Configuración PWM  ////////////////////////////////////////////////////////////////////
uint32_t pwmfreq = 8000; // 1KHz
const uint8_t pwmChannel = 0;
const uint8_t pwmresolution = 8;
const int PWM_Max = pow(2,pwmresolution)-1; //

//adc
//Adafruit_ADS1115 ads;
float current = 0;
int16_t adc0 = 0;


// Pines driver motor ///////////////A/////////////////////////////////////////////////////
#ifdef ACTIVA_R
const uint8_t PWM_Pin = 26; // Entrada EN // para mi controlador es la unica salida pwm
const uint8_t PWM_f = 33; // Entrada PWM1 // direccion
const uint8_t PWM_r = 14; // Entrada PWM2 // para mi controlador es enable

#else
const uint8_t PWM_Pin = 26; // Entrada EN 
const uint8_t PWM_f = 33; // Entrada PWM1 
const uint8_t PWM_r = 17; // Entrada PWM2 
#endif

// Voltaje maximo motor ////////////////////////////////////////////////////////////////////
float SupplyVolt = 12;

// Pines encoder ////////////////////////////////////////////////////////////////////
const uint8_t A_enc_pin = 35;
const uint8_t B_enc_pin = 34;

// Conversión a angulo y velocidad del Pololu 3072
//const float conv_rad = ; 
//const float conv_rev = ;
//const float conv_rad_grados = ; 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;

// Declarar funciones ////////////////////////////////////////////////////////////////////
void config_sp(); // Configuracion puerto serie
void config_oled(); // Configuracion OLED
void config_enc(); // Configuracion del encoder
void config_PWM(); // Configuracion PWM
void excita_motor(float v_motor); // Excitacion motor con PWM
float interpola_vel_vol_lut(float x); // Interpolacion velocidad/voltios LUT
void proceso_angulo();
void proceso_rpm();
void puesta_a_cero();
void init_params();
void config_ADC();
void clean();
void config_wire();

// TABLA VELOCIDAD-VOLTAJE P1D
#define LONG_LUT 12
//Vector de tensiones
const float Vol_LUT[LONG_LUT] = {0.6,  1 ,  1.5 , 2 ,  3,   4,  5,  6, 7, 8, 9, 100};
// Vector de velocidades
const float Vel_LUT[LONG_LUT] = {0.25,0.7, 1.12, 1.58, 2.5, 3.25, 4.1, 5, 5.91, 6.75,7.58,7.58 };

// Variables globales ////////////////////////////////////////////////////////////////////
int ACTIVA_P1C_MED_ANG = 0;
int32_t ang_cnt = 0;
float pwm_volt = 0;
int32_t pwm_motor = 0;
//int32_t sign_v_ant = 0;
float v_medida = 0;     // Valor medido de angulo o velocidad -----------------
float a_medida = 0;
float ref_val = 0;      // Valor de referencia de angulo o velocidad
int8_t start_stop = 0;  //1 -> en funcionamiento | 0 -> parado 
float K_p = 0;
float T_i = 0; // Ki
float T_d = 0; // Kd
int direccion = 0;
int direccion_ant = 0;  // variable global de función excita_motor
struct K_x {
  float kp;
  float ti;
  float td;
  float tcb;
  float alpha;
};


// Variables globales task_loopcontr ///////////////////////////////////////////////////////
float anterior = 0;
float V_i_anterior = 0;
float error = 0;
float volt = 0;
float V_p = 0;
float V_i = 0;
float V_d = 0;
float da = 0;
float error_anterior = 0;
bool windup_bool = false; bool windup_state = false;float windup_value = 6.1;
float Tcb = 2;
float V_i_windup = 0;
float voltdeadzone = 0; 

// Variables filtro derivativo
//float alpha = 4.0;
bool fd_bool = 1;
float fd = 0;
float fd_anterior = 0;

// Declaracion objetos  ////////////////////////////////////////////////////////////////////
xQueueHandle cola_enc; // Cola encoder

/*
// KP KI KD para velocidad ##########################################################################
void obtenerKvel(){ // Constantes para Velocidad RPM
  K_p = 1.9;
  T_i = 0.7;
  T_d = 0.0001;
  Tcb = 0.9;
}
// KP KI KD para angulo #############################################################################
void obtenerKang(){ // Constantes para ángulo
  K_p = 0.0026;
  T_i = 2;
  T_d = 0.0399;
  Tcb = 0.9;
}
*/
