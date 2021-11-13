/*
 * ---------- OCONTROL DE VELOCIDAD BASADO EN UN CONTROLADOR PI PARA LA PLATAFORMA XSPACE v1.0----------
 * 
 * Descripción: Este programa permite realizar un control de velocidad con un controlador PI sobre el contector
 *              del motor A + Encoder 1 con una referencia de 180 grados x segundo.
 * Version:     1.0
 * Creador:     Pablo Cárdenas Cáceres
 * Fecha:       25-09-2020 05:41pm
 *  
 */

#include <FreeRTOS_XSpace.h>

//STANDBY DEL TB6612FNG
#define STBY 8

//PARA CONTROLAR CON EL CONECTOR MOTOR A
#define BIN1 7
#define PWMB 10

//PARA SENSAR CON EL CONECTOR ENCODER 1
#define ENCODER1_CH_A 2
#define ENCODER1_CH_B 4


//Variables para el Encoder 1
volatile double T_act_1 = 0;
volatile double T_ant_1 = 0;
volatile double Periodo_1 = 100000000;
volatile double contador_1=0;

//Variables globales
uint16_t PWM_resolucion = 400;

void setup() {

  Serial.begin(1000000);
  
  pinMode(STBY,OUTPUT);
  pinMode(BIN1,OUTPUT);

  pinMode(ENCODER1_CH_A, INPUT_PULLUP);
  pinMode(ENCODER1_CH_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_CH_A), ISR_ENCODER_1, RISING); //Activamos la interrupcion por flanco de subida en el canal A del encoder 1

  XSpace_PWMinit(PWM_resolucion); //Configura los pines 9 y 10 como salidas PWM de 400 de resolucion. La Frecuencia de la señal será F_CPU/resolucion
  
  digitalWrite(STBY,HIGH); //Habilitamos el driver

  xTaskCreate(ControlPosicionMotorA,"-", 200, NULL, 1, NULL); //Creacion de la tarea ControlVelocidadMotorA
  vTaskStartScheduler();  //Empieza a trabajar el planificador
}

void loop() {
  // put your main code here, to run repeatedly:

}

void ControlPosicionMotorA(void* Parameters){
  
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = 10/portTICK_PERIOD_MS; //Para que la tarea tenga un periodo de 10ms de ejecución (solo puede ser muliplo de 2)
  xLastWakeTime = xTaskGetTickCount();

  double e = 0;
  double e_1 = 0;
  double u = 0;
  double u_1 = 0;
  double ui_1 = 0;
  double T = 0.01;
  double Kp = 0.168;
  double Ki = 0.2309;
  double Kd = 0.0175;

  double wr = -20;
  double tetar = 90;
  double w = 0;
  double vm = 5;
  double teta=0;

  int Duty = 0;

  double ui=0;
  double ud=0;
  
  while(1){

    vTaskDelayUntil( &xLastWakeTime, xPeriod); //Espera a que se cumpla el periodo  para realizar el algoritmo de control

    w = 375000.0/Periodo_1; //Calculo de la velocidad del motor usando el contector Encoder 1
    teta = contador_1/960*360; //Angulo en grados
    e = tetar - teta;
    
    //Algorito de control PID
    ui=Ki*T/2*(e+e_1)+ui_1;
    ud=Kd*(e-e_1)/T;
    u = Kp*e+ui+ud; 

    //Se actualizan los valores anteriores
    u_1 = u;
    ui_1 = ui;
    e_1 = e;

    //Se determina el sentido de giro de acuerdo al signo de u
    if(u<0) digitalWrite(BIN1,HIGH);// Si BIN1==HIGH -> NEGATIVO, Si AIN1==LOW -> POSITIVO
    else    digitalWrite(BIN1,LOW);

    //Saturación
    if(u>5) u=5;
    if(u<-5)u=-5;
    
    //Calculo del duty
    Duty = (int) ((abs(u)/vm)*PWM_resolucion);
    


    //Se aplica el voltaje al motor DC
    XSpace_SetDuty(PWMB,Duty);

    //Se imprime el valor de la velocidad (Se puede ver usando el Serial Plotter)
    Serial.println(teta);
  
  }

  vTaskDelete(NULL);
}


/* 
  Rutina de interrupción externa para calcular el periodo 
  entre flancos de subida en el canal A del conector ENCODER 1.
*/
void ISR_ENCODER_1(){
  
  T_act_1 = micros();
  
  Periodo_1 = T_act_1 - T_ant_1;

  if(digitalRead(ENCODER1_CH_B) == LOW) 
  {
    Periodo_1 = Periodo_1*(-1);
    contador_1--;
  }
  else
  {
    contador_1++;
  }
 
  T_ant_1 = T_act_1;
}
