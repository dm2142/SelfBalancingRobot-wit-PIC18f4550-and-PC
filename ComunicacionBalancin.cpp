#include <iostream>
#include <string.h>
#include <dos.h>
#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <conio.h>
#include <stdlib.h>
#include <unistd.h>

/* DECLARAR DATOS ESTÁTICOS */

#define _PI 		3.1416		// Constante matemática
#define T      		0.035		// Tiempo de muestreo

#define VM          11.20			// Voltaje máximo de la batería
#define VNM			10.5 //7.4		// Voltaje nominal máximo (volts)
#define CPR			1632			// Cuentas por revolución a la salida de cada motor  modelo #4824
#define ESC_RAD		(2*_PI)/(CPR)	// Relación para convertir cuentas del encoder a radianes
#define ESC_PWM		127/VM			// Escalamiento para convertir voltaje a valor de PWM

#define	VELNM	5			// Velocidad máxima de los motores

/* OFFSET PARA DATOS MPU6050 */
 
#define Ax_o	0	// Aceleraciones
#define Ay_o	0
#define Az_o	0
#define Gy_o	0		// Velocidad angular con respecto el eje y		

/* GANANCIAS DE LOS CONTROLADORES */

/* Controlador de posición maestro*/
float kpP = 50.0;
float kiP = 0.0;
float kdP = 1.6;

/* Controlador de velocidad motor derecho */
float kpD = 1.125;  //1.5
float kiD = 9.0;   //12
float kdD = 0.0;
/* Controlador de velocidad motor izquierdo*/
float kpI = 1.125;  //1.5 1.0
float kiI = 9.0;     //12 7.5
float kdI = 0.0;
/* Velocidad de avance */
float omega_a = 0; // Rad/s velocidad de avance

/* Setpoints de posición */
float phi_horizontal = -0.120;
float phi_d	= phi_horizontal;	// Para plano horizontal
float alpha_p = -0.21;  // Para plano inclinado

/* DECLARACIÓN DE VARIABLES Y DATOS */

unsigned char flag_COM = 0 ;	// Dato para sincronizar la recepción de datos
unsigned char flagfile=0;
float t = 0;					// Variable para almacenar el tiempo transcurrido

// Variables para las posición del motor derecho
unsigned char posD_H, posD_L;	// Datos para guardar los bytes de posición enviados por el microcontrolador
unsigned short int posD;	// 16 bits de cuenta

// Variables para las posición del motor izquierdo
unsigned char posI_H, posI_L;	// Datos para guardar los bytes de posición enviados por el microcontrolador
unsigned short int posI;	// 16 bits de cuenta

// Datos para guardar lo correspondiente con respecto al ángulo del balancín.
unsigned char Ax_H, Ax_L;
signed short int Ax_raw;

unsigned char Gy_H, Gy_L;
signed short int Gy_raw;

float Ax;	// Almacenamiento de la aceleración en X
float Gy;	// Alamcenamiento de la velocidad angular con respecto Y

// Datos para cálculo del ángulo
float accel_ang_x;
float angle, angle_prev;
float phi = 0.0;

// Controlador PD de posición (Maestro)
float e_phi[2] = {0.0, 0.0};
float P_phi, D_phi, I_phi;			// Proporcional, Derivativo , Control PD
float u_phi = 0;					// Esfuerzo de control del controlador maestro (rad/s)

// Controladores PI de velocidad (esclavos)
unsigned char pwmI = 0, pwmD = 0;                            // Dato para almacenar el valor para el ciclo de trabajo de cada PWMx
float omegaD = 0.0;
float omegaI = 0.0;

// Datos para el motor derecho
float omegadD;   // Velocidad deseada Derecho
float Posicion_rad_derecho[2] = {0.0, 0.0};
float eD[2] = {_PI, 0.0};
float uD = 0.0;

float P_omegaD = 0.0, I_omegaD = 0.0;			// Proporcional, Integral , Control PI

// Datos para el motor izquierdo
float omegadI;   // Velocidad deseada Izquierda
float Posicion_rad_izquierdo[2] = {0.0, 0.0};
float eI[2] = {0.0, 0.0};
float uI = 0.0;                               // Datos para almacenar el esfuerzo de control de cada motor.

float P_omegaI = 0.0, I_omegaI = 0.0; 	// Proporcional, Integral , Control PI

unsigned char Sensores;
float Kvi = 1.0, Kvd = 1.0; // Ganancias de cada motor
bool  sI, sD, sP = false;

char op = '0';

using namespace std;
int main()
{
    HANDLE h; /*handler, sera el descriptor del puerto*/
    DCB dcb; /*estructura de configuracion*/
    DWORD dwEventMask; /*mascara de eventos*/
    FILE *fp;
    
    //if((fp=fopen("PruebaPlanoInclinado.txt","w+"))==NULL)
    if((fp=fopen("DatosBalancin.txt","w+"))==NULL)
    //if((fp=fopen("Data-Balancin-Balanceado.txt","w+"))==NULL)
    //if((fp=fopen("Data-Balancin-Seguirvuelta.txt","w+"))==NULL)
    //if((fp=fopen("Data-Balancin-Seguirlinea.txt","w+"))==NULL)
	{
	  printf("No se puede abrir el archivo.\n");
	  exit(1);
	}    
        
    /*abrimos el puerto*/
    h=CreateFile("COM5",GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
    
    if(h == INVALID_HANDLE_VALUE) 
	{
         /*ocurrio un error al intentar abrir el puerto*/
    }
             
	/*obtenemos la configuracion actual*/
	if(!GetCommState(h, &dcb)) 
	{
         /*error: no se puede obtener la configuracion*/
    }
         
    /*Configuramos el puerto*/
	dcb.BaudRate = 57600;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = TRUE;
         
    /* Establecemos la nueva configuracion */
    if(!SetCommState(h, &dcb)) 
	{
        /* Error al configurar el puerto */
    }
                  
                  
    DWORD n;
    char enviar;
    int recibido;
                             
    /* Para que WaitCommEvent espere el evento RXCHAR */
    SetCommMask(h, EV_RXCHAR);
    while(1) 
	{
		
    	recibido=0;
        /* Enviamos... */
                  
        /* De la llamada a WaitCommEvent solo se retorna cuando ocurra51.		 * el evento seteado con SetCommMask */
        // WaitCommEvent(h, &dwEventMask, NULL);
        /* Recibimos algun dato!*/
        
        while( 1 ){
        	
        	ReadFile(h, &recibido, 1/* leemos un byte */, &n, NULL);
        	if(!n)
            	break;
            else				// Recepción de los datos enviados por el microcontrolador.
			{	
				if(kbhit())
				{
					op = getch();
				}

				if(op == 'o'){
					phi_d = phi_horizontal;
					op = '0';
				}
				if(op == 'p'){
					phi_d = alpha_p;
					op = '0';
				}
				
				if(op== '1')
				{
					kpP -= 0.1;
					op='0';
				}
				if(op== '2')
				{
					kpP += 0.1;
					op='0';
				}
				if(op== '3')
				{
					kdP -= 0.01;
					op='0';
				}
				
				if(op== '4')
				{
					kdP += 0.01;
					op='0';
				}
				if(op== '5')
				{
					kiP -= 1.0;
					op='0';
				}
				if(op== '6')
				{
					kiP += 1.0;
					op='0';
				}
				if(op== '7')
				{
					phi_d -= 0.001;
					
					op='0';
				}
				if(op== '8')
				{
					phi_d += 0.001;
					op='0';
				}				
				
							
				if( flag_COM != 0){		// Iteración para cambio de estado en la recepción de datos.
					flag_COM ++;
				}
				
				if( (recibido == 0x01) && (flag_COM == 0) ){	// Recepción del dato de identificación para comenzar la recepción de datos.
					flag_COM = 1;
				}
				
				if( flag_COM == 2 ){	// Recepción de los 8 bits más significativos de la cuenta del encoder del motor derecho.								
					posI_H = recibido;
					recibido = 0;
				}
				
				if( flag_COM == 3){		// Recepción de los 8 bits menos significativos de la cuenta del encoder del motor derecho.
					posI_L = recibido;
					recibido = 0;
				}
				
				if( flag_COM == 4){		// Recepción de los 8 bits más significativos de la cuenta del encoder del motor izquierdo.
					posD_H = recibido;
					recibido = 0;
				}
				
				if( flag_COM == 5){		// Recepción de los 8 bits menos significativos de la cuenta del encoder del motor izquierdo.
					posD_L = recibido;
					recibido = 0;
				}
				
				if( flag_COM == 6){		// Recepción de los 8 bits más significativos de la aceleración en x
					Ax_H = recibido;
					recibido = 0;
				}
				
				if( flag_COM == 7){		// Recepción de los 8 bits menos significativos de la aceleración en x
					Ax_L = recibido;
					recibido = 0;
				}
				
				if( flag_COM == 8){	// Recepción de los 8 bits más significativos de la velocidad angular en y
					Gy_H = recibido;
					recibido = 0;
				}
				
				if( flag_COM == 9){
					Gy_L = recibido;
					recibido = 0;
				}
					
				if( flag_COM == 10){	// Recepción de los 8 bits relacionados con los sensores
					Sensores = recibido;
					recibido = 0;
					
					// REALIZAR CONVERSIÓN DE DATOS
					
					// Convertir las posiciones a 16 bits
					posI = (posI_H<<8) + posI_L;
					posD = (posD_H<<8) + posD_L;	
					
					
					Ax_raw = (signed short int) (((Ax_H<<8) + Ax_L) + Ax_o);		// Obtención de las aceleraciones en cada eje					
					Gy_raw = (signed short int) (((Gy_H<<8) + Gy_L) + Gy_o);		// Obtención de la velocidad angular en el eje Y
					
					Ax = (90*((float)Ax_raw))/16384.0;					
					Gy = ((float)Gy_raw)/131.0;											
	
					angle = 0.98*( angle_prev + (-Gy)*T ) + 0.02*Ax;
    				angle_prev = angle;
    				
    				phi = angle*(_PI/-180.0);    // Conversión del ángulo obtenido a radianes con la posición actual del balancín.
    				
    				
    				/* CONTROL DE POSICIÖN ANGULAR */
    
				    e_phi[1] = e_phi[0];
				    
				    e_phi[0] = phi_d - phi;     // Obtención del error de posición;
				    
				    P_phi = kpP*e_phi[0];
				    D_phi = (kdP*(e_phi[0]-e_phi[1]))/T;
				    
				    if( (I_phi < VELNM ) && ( I_phi > (-VELNM)) ){
				        I_phi = I_phi + (kiP*T*e_phi[0]);
				    }else{
				        if( I_phi >= VELNM)
				            I_phi = 0.90*VELNM;
				        
				        if( I_phi <= (-VELNM))
				            I_phi = -0.90*(VELNM);
				    }
				    
				    u_phi = P_phi + D_phi + I_phi + omega_a;      // Esfuerzo de control del controlador PD (rad/s)
				    
				    if(u_phi > VELNM)
				        u_phi = VELNM;
				    
				    if(u_phi < (-VELNM))
				        u_phi = (-VELNM);
				    
				    /* COMPORTAMIENTO DE ACUERDO A LOS SENSORES */
				    /*
					if( U_pos > 0 ){
					    if( Sensores == 0 ){
					    	sI = false;
					    	sD = false;
					    	Kvd = 1.0;
					    	Kvi = 1.0;
						}else if(Sensores == 1){
							sI = true;
					    	sD = false;
							Kvd = 0.0;
					    	Kvi = 1.8;
						}else if(Sensores == 2){
							sI = false;
					    	sD = true;
							Kvd = 1.8;
					    	Kvi = 0.0;
						}else if(Sensores == 3){
							sI = true;
					    	sD = true;
							Kvd = 1.0;
					    	Kvi = 1.0;
						}else if(Sensores == 4){
					    	sI = false;
					    	sD = false;
							sP = true;
							PHI_D = ALPHA_P;	
						}else if(Sensores == 5){
					    	sI = true;
					    	sD = false;
							sP = true;
							PHI_D = ALPHA_P;
						}else if(Sensores == 6){
					    	sI = false;
					    	sD = true;
							sP = true;
							PHI_D = ALPHA_P;	
						}	
						}else if(Sensores == 7){
					    	sI = true;
					    	sD = true;
							sP = true;
							PHI_D = ALPHA_P;
					}else{
							Kvd = 1.0;
					    	Kvi = 1.0;
					}
				    */

					/* ASIGNAR LAS VELOCIDADES DESEADAS EN LOS MOTORES */				    		
				    omegadD = (u_phi)*Kvd;
    				omegadI = (u_phi)*Kvi;
				    
				    /*
    				if(VEL_DD > VELNM)
				        U_pos = VELNM;
				    if(VEL_DD < (-VELNM))
				        U_pos = (-VELNM);
    				
    				if(VEL_DI > VELNM)
				        U_pos = VELNM;
				    if(VEL_DI < (-VELNM))
				        U_pos = (-VELNM);
    				*/  
					    		
					/* CONTROL MOTOR IZQUIERDO */
        
				    eI[1] = eI[0];
				    Posicion_rad_izquierdo[1] = Posicion_rad_izquierdo[0];
				    
				    Posicion_rad_izquierdo[0] = ESC_RAD*((float) posI);
				    
				    if( fabs(Posicion_rad_izquierdo[0] - Posicion_rad_izquierdo[1]) < 10)
				        omegaI = (Posicion_rad_izquierdo[0] - Posicion_rad_izquierdo[1])/T; // Obtener la velocidad actual de la rueda izquierda
				    
				    eI[0] = omegadI - omegaI;	// Obtención del error de velocidad en el motor izquierdo
				    
				    P_omegaI = kpI*eI[0];	// Parte proporcional
				    
				    if( (I_omegaI < VNM) && (I_omegaI > (-VNM)) ){
				        I_omegaI = I_omegaI + (kiI*T*eI[0]);
				    }else{
				        if( I_omegaI >= VNM)
				            I_omegaI = 0.95*VNM;
				        
				        if( I_omegaI <= (-VNM))
				            I_omegaI = -0.95*(VNM);
				    }
				    
				    // Obtención del esfuerzo de control para el motor izquierdo
				    uI = P_omegaI + I_omegaI;
				    
				    if(uI >= VNM)
				        uI = VNM;
				    
				    if(uI <= (-VNM))
				        uI = (-VNM);
				    
				    pwmI = (unsigned char)((fabs(uI)*ESC_PWM));
				    
				    if( uI < 0 ){
				    	pwmI =  pwmI +128;	// Agregar bit de signo para indicar que el esfuerzo es negativo
					}
					
					/* CONTROL MOTOR DERECHO */
					
				    eD[1] = eD[0];
				    Posicion_rad_derecho[1] = Posicion_rad_derecho[0];
				    
				    Posicion_rad_derecho[0] = ESC_RAD*((float)posD);
				    
				    if( fabs(Posicion_rad_derecho[0] - Posicion_rad_derecho[1]) < 10 )
				        omegaD = (Posicion_rad_derecho[0] - Posicion_rad_derecho[1])/T;	// Obtener la velocidad actual de la rueda derecha
				    
				    eD[0] = omegadD - omegaD;
				    
				    P_omegaD = kpD*eD[0];	// Parte proporcional
				    
				    if( (I_omegaD < VNM) && (I_omegaD > (-VNM)) ){	// Parte Integral
				        I_omegaD = I_omegaD + kiD*T*eD[0];
				    }else{
				        if( I_omegaD >= VNM)
				            I_omegaD = 0.95*VNM;
				        
				        if( I_omegaD <= (-VNM))
				            I_omegaD = -0.95*(VNM);
				    }
				    
				    // Obtención del esfuerzo de control para el motor izquierdo
				    uD = P_omegaD + I_omegaD;
				    
				    if(uD >= VNM)
				        uD = VNM;
				    
				    if(uD <= (-VNM))
				        uD = (-VNM);
				    
				    pwmD = (unsigned char)((fabs(uD)*ESC_PWM));
				    
				    if( uD < 0 ){
				    	pwmD += 128;	// Agregar bit de signo para indicar que el esfuerzo es negativo
					}
					
					/* ENVIO DE DATOS */
					
					
					enviar=0xAA; // Enviar byte de identificación para el microcontrolador
                  	if(!WriteFile(h, &enviar/*puntero al buffer*/, 1/* 1 byte*/, &n, NULL)){
                    	/* Error al enviar */ 
                	}
                    
                    //usleep(250);					
                    
					enviar= pwmI;	// Enviar valor del PWM1 (Motor izquierdo)
                  	if(!WriteFile(h, &enviar/*puntero al buffer*/, 1/* 1 byte*/, &n, NULL)){
                    	/* Error al enviar */ 
                    }
                    //usleep(250);
                    
                    enviar= pwmD;	// Enviar el valor del PWM2 (Motor derecho)
					
                  	if(!WriteFile(h, &enviar/*puntero al buffer*/, 1/* 1 byte*/, &n, NULL)){
                    	/* Error al enviar */ 
                    }
					
					
					
					// Imprimir los datos para graficar
					//printf( "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%i\t%i\t%i\r\n", t, phi_d, phi, omegadI,omegaI, omegadD, omegaD, uI, uD, sP, sD, sI );
					//printf("%i\t%i\r\n",posD, posI);
					
					// Sección para imprimir datos de prueba
					printf("%0.3f\t%0.3f\t%0.3f\t%0.3f\n\r",kpP, kdP, kiP, phi_d); ///// Gganancias
					//prinft("%i\t%i\r\n", posD, posI);
					
					// Imprimir datos enviados por el uC
					//printf("Ax:%i\tAy:%i\tAz:%i\tGy:%i\r\n",Ax_raw, Ay_raw, Az_raw, Gy_raw);
					
					//printf("%0.3f\t\n", PHI_D);
					
					//printf("%0.3f\t%0.3f\t%0.3f\r\n",t, accel_ang_x, angle);
					//fprintf(fp,"%0.3f\t%0.3f\t%0.3f\r\n",t, accel_ang_x, angle);
					
					//t, PHI_D, PHI, VEL_DI, velocidad_izquierdo, VEL_DD, velocidad_derecho, U_vel_mizquierdo, U_vel_mderecho, Sensores
					
					/*escribir los datos en el archivo .txt*/
					fprintf(fp, "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%i\t%i\t%i\r\n", t, phi_d, phi, omegadI, omegaI, omegadD, omegaD, uI, uD,  sP, sD, sI);
					
					// Sección para guardar datos de pruebas
        			//fprintf(fp,"%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",t,PHI_D, PHI, U_pos, Error_pos[0],VEL_D,velocidad_derecho, U_vel_mderecho, Error_vel_derecho[0] , velocidad_izquierdo, U_vel_mizquierdo,Error_vel_izquierdo[0]);
					//fprintf(fp,"%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%u\n",t,PHI_D, PHI, U_pos, Error_pos[0],VEL_DD,velocidad_derecho, U_vel_mderecho, Error_vel_derecho[0] , VEL_DI,velocidad_izquierdo, U_vel_mizquierdo,Error_vel_izquierdo[0], Sensores);
					
					
					        			
					flag_COM = 0;
           			t += T; 	// Aumento en el tiempo                	   
				
				}
			}
        	
        	
		}
        

    }//CIERRE WHILE()
fclose(fp);             
return 0;
}
