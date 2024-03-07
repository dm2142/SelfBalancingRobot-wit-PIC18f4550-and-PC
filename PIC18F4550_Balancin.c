#include <18F4550.h>
#fuses NOWDT, NOPROTECT, HS, CPUDIV1, NOBROWNOUT, NOLVP, NOPUT
#use delay (clock = 20M, crystal = 20M) // Configuración usando un oscilador externo de 20 MHz
#use RS232 (baud=57600,XMIT=PIN_C6,RCV=PIN_C7,BITS=8,PARITY=N)
#use I2C( MASTER, SDA=PIN_B0, SCL=PIN_B1, FAST)

#include <MPU6050.c>

/* REGISTROS */
#BYTE T0CON = 0XFD5     // Configuración TIMER0
#BIT TMR0ON = 0XFD5.7

#BYTE T1CON = 0XFCD     // Configuración TIMER1
#BIT  TMR1ON = 0XFCD.0

#BYTE PORTA = 0xF80
#BYTE PORTB = 0xF81
#BYTE PORTD = 0xF83
#BIT  Sen_I = 0xF83.0 
#BIT  Sen_D = 0xF83.1
#BIT  Sen_P = 0xF80.1

// Selectores del motor izquierdo
#BIT PD6 = 0xF83.6 // In3
#BIT PD7 = 0xF83.7 // in4
// Selectores del motor derecho
#BIT PD4 = 0xF83.4 // In1
#BIT PD5 = 0xF83.5 // In2

/* VARAIBLES GLOBALES */

unsigned int16 tope = 0;
int16 ENC_A[2], ENC_B[2] = {0};
int16 PUERTO_B, AUX_CNT = 0;
int16 CNT_ENC_A = 32467;
int16 CNT_ENC_B = 32467;
unsigned int8 ENC_AH, ENC_AL;
unsigned int8 ENC_BH, ENC_BL;
unsigned int16 AUX_ENC;

unsigned int8 Sensores = 0;
unsigned int8 AUX_A, AUX_B = 0;

signed int8 A_data_x[2];
signed int16 accel_value_x;
   
signed int8 A_data_y[2];
signed int16 accel_value_y;
   
signed int8 A_data_z[2];
signed int16 accel_value_z;
   
signed int8 G_data_x[2];
signed int16 gyro_value_x;
   
signed int8 G_data_y[2];
signed int16 gyro_value_y;
   
signed int8 G_data_z[2];
signed int16 gyro_value_z;

int8 dato[3], pwmI, pwmD;
int8 rda = 0;
int1 data_received = 0;

#int_rb  // Interución por cambio de estado en el puerto B (B4 - B7)
void rb_isr(){
   PUERTO_B = PORTB;
   
   // PINES B4 y B5 --- Motor Derecho
   ENC_A[0] = ((PUERTO_B)&(0x30))>>4;
   AUX_CNT = ENC_A[0]^ENC_A[1];
   if( AUX_CNT != 0){
      if(AUX_CNT != 3){
         if(((ENC_A[1]<<1)^ENC_A[0])&(0x02))
            CNT_ENC_A --;
         else
            CNT_ENC_A ++;            
      }
   }
   
   ENC_A[1] = ENC_A[0];
   
   // PINES B6 y B7 -- Motor Izquierdo
   ENC_B[0] = ((PUERTO_B)&(0xC0))>>6;
   AUX_CNT = ENC_B[0]^ENC_B[1];
   if( AUX_CNT != 0){
      if(AUX_CNT != 3){
         if(((ENC_B[1]<<1)^ENC_B[0])&(0x02))
            CNT_ENC_B --;
         else
            CNT_ENC_B ++;            
      }
   }
   
   ENC_B[1] = ENC_B[0];
    
}

#int_rda
void rda_isr(){
   if( rda == 0 ){
      dato[0] = getc();
      if( dato[0] == 0xAA )
         rda = 1;
   }
   
   if( rda == 1 ){
      dato[1] = getc();
      if( dato[1] >= 128){ // Detectar si el esfuerzo de control es negativo
         PD6 = 0;
         PD7 = 1;
         dato[1] = dato[1] - 128;
         pwmI = dato[1] << 1;
      }else{
         PD6 = 1;
         PD7 = 0;
         pwmI = dato[1] << 1;
      }
      set_pwm2_duty(pwmI); // Establecer el pwm del motor izquierdo
      rda = 2;
   }

   if( rda == 2 ){
      dato[2] = getc();
      if( dato[2] >= 128){ // Detectar si el esfuerzo de control es negativo
         PD4 = 1;
         PD5 = 0;
         dato[2] = dato[2] - 128;
         pwmD = dato[2] << 1;
      }else{
         PD4 = 0;
         PD5 = 1;
         pwmD = dato[2] << 1;
      }
      set_pwm1_duty(pwmD); // Establecer el pwm del motor derecho
      rda = 0;
      data_received = 1;
      output_low(PIN_D2);
   }

   
}

void main( void ){

   MPU6050_init();
   
   /* CONFIGURACIÓN DEL TIMER 0 FREE COUNTER  -> 00000010
      - 0 : Stops Timer0
      - 0 : Timer0 is configured as a 16-bit timer/counter 
      - 0 : Internal instruction cycle clock (CLKO)
      - 0 : Increment on low-to-high transition on T0CKI pin
      - 0 : Timer0 prescaler is assigned. Timer0 clock input comes from prescaler output.
      - 010 : 1:8 Prescale value
     
   */
   T0CON = 0x02; // Base de tiempo de 1.5 us 
   TMR0ON = 0;
   
   /* CONFIGURACIÓN DEL TIMER 1 FREE COUNTER  -> 10110000
      - 1 : Enables register read/write of Timer1 in one 16-bit operation 
      - 0 : Device clock is derived from another source
      - 11 : 1:8 Prescale value
      - 0 : Timer1 oscillator is shut off
      - 0 : Synchronize external clock input
      - 0 : Internal clock (FOSC/4)
      - 0 : Stops Timer1  
   */
   T1CON = 0xB0; // Base de tiempo de 1.5 us 
   TMR1ON = 0;
   
   // %%%%%% CONFIGURACIÓN DEL PWM PARA AMBAS SEÑALES DE LOS DOS MOTORES %%%%%%%%%%
   
   setup_timer_2(T2_DIV_BY_16,207,1); // Configuración del timer2 con los valores para generar PWM con una frecuencia aprox de 1.5 kHz.
   // Configurar el modulo captura y compara en modo PWM.
   setup_ccp1(CCP_PWM); // Configurar para uso como PWM.
   setup_ccp2(CCP_PWM); // Configurar para uso como PWM.
   delay_us(20);
   //setup_ccp1(CCP_OFF); // Inicializar con la salida de la señal apagada.
   //setup_ccp2(CCP_OFF);
   delay_us(20);
   set_pwm1_duty( 0 );   // Función para establer el ciclo de trabajo de cada salida PWM
   set_pwm2_duty( 0 );
   
   set_tris_b(0b11111111); // Lectura de Encoders
   set_tris_d(0b00000011); // Lectura Sensores de línea
   set_tris_a(0b00000011); // Lectura Sensores de línea
   enable_interrupts(int_rb);
   enable_interrupts(int_rda);
   enable_interrupts(global);
   
   
   /* ESPERA PARA LA INICIALIZACIÓN DE LOS ESPERIFICOS*/
   
  
   delay_ms(2000);
   
   
   
   while( True ){
      
      output_high(PIN_D3);
      
      // Uso de la velocidad angular en 'y' y la aceleración en 'x' para obtener el ángulo
      
      A_data_x[0] = MPU6050_read(ACCEL_XOUT_H);
      A_data_x[1] = MPU6050_read(ACCEL_XOUT_L);
      //delay_us(20);
      //A_data_y[0] = MPU6050_read(ACCEL_YOUT_H);
      //A_data_y[1] = MPU6050_read(ACCEL_YOUT_L);
      //delay_us(20);
      //A_data_z[0] = MPU6050_read(ACCEL_ZOUT_H);
      //A_data_z[1] = MPU6050_read(ACCEL_ZOUT_L);
      delay_us(20);
      G_data_y[0] = MPU6050_read(GYRO_YOUT_H);
      G_data_y[1] = MPU6050_read(GYRO_YOUT_L); 
      
      AUX_ENC = (CNT_ENC_B)&(0xFF00);
      ENC_BH = AUX_ENC>>8;
      ENC_BL=(CNT_ENC_B)&(0x00FF); 
      
      AUX_ENC = (CNT_ENC_A)&(0xFF00);
      ENC_AH = AUX_ENC>>8;
      ENC_AL=(CNT_ENC_A)&(0x00FF); 
      
      AUX_A = (PORTA & 0x02)<<1;
      AUX_B = PORTD & 0x03;
      
      // Revisar estado de los sensores
      Sensores = AUX_A | AUX_B;
      
      
      putc(0x01);          //mandando dato de reconocimieno al puerto serial
      putc(ENC_BH);        //Cuenta del Motor Izquierdo
      putc(ENC_BL);
      putc(ENC_AH);        //Cuenta del Motor Derecho
      putc(ENC_AL);
      
      //putc(CNT_ENC_B);
      //putc(CNT_ENC_A);
      //CNT_ENC_A = 127;
      //CNT_ENC_B = 127;
      
      putc(A_data_x[0]);
      putc(A_data_x[1]);
      //putc(A_data_y[0]);
     // putc(A_data_y[1]);
      //putc(A_data_z[0]);
      //putc(A_data_z[1]);
      putc(G_data_y[0]);
      putc(G_data_y[1]);
      
      putc(Sensores);
      
      set_timer0(0);
      data_received = 0;
      output_low(PIN_D3);
      output_high(PIN_D2);
      
      TMR0ON = 1;
      
      do{ 
        
         tope = get_timer0(); // Actualización del valor del Timer0
         //output_high(PIN_D2);
         
      }while(tope < 21875); // Espera de 35 ms
      TMR0ON = 0;
      output_low(PIN_D2);
     
      if(data_received == 0){
         set_pwm1_duty( 0 );   // Función para establer el ciclo de trabajo de cada salida PWM
         set_pwm2_duty( 0 );
      }
      
   }

}
