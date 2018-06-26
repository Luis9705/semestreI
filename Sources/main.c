#include "derivative.h" /* include peripheral declarations */

//--------------------------------------------------CONSTANTES-----------------------------------------------------

enum{				//Numeración para la máquina de estados
	WAIT_READY,		//Crea constantes numeradas, empezando desde 0
	WAIT_IP,
	WAIT_OK,
	CONF_MUX,
	CONF_SERVER,
	SHOW_IP,
	WAIT_HEADER,
	WAIT_CHANNEL,
	WAIT_BYTES,
	WAIT_INFO,
	WAIT_SEND,
	WAIT_SEND_OK,
	WAIT_CONNECTION
};
enum{
	RED,
	GREEN,
	BLUE,
	WHITE,
	YELLOW,
	PINK,
	CYAN,
	ON,
	OFF,
	RESET,
	TOOGLE,
	TX_BUSY,
	TX_READY,
	MENSAJE
};
enum{				//Numeración para los mensajes
	READY,		
	WIFI_GOT_IP,
	OK,
	ERROR,
	BUSY,
	SEND_OK,
	ARROW,			//>
	AT,
	AT_CWMODE,
	AT_CWJAP,
	AT_CIPSTA_DEF,
	AT_CIPMUX,
	AT_CIPSERVER,
	AT_CIFSR,
	AT_CIPSEND,
	AT_CIPCLOSE,
	AT_CIPSTART,
	GET,
	IPD,
	COMA,
	DOS_PUNTOS,			//":"
	PUNTO,
	ESPACIO,
	MENSAJE_RECIBIDO,
	CANAL_RECIBIDO,
	ENTER,
	BYTES_RECIBIDOS,
	LED_ROJO_ON,
	LED_AZUL_ON,
	LED_VERDE_ON,
	LED_BLANCO_ON,
	LED_AMARILLO_ON,
	LED_OFF,
	FOCO_ON,
	FOCO_OFF,
	COMANDO_NO_RECONOCIDO,
	PAGINA_WEB1,
	PAGINA_WEB2,
	PAGINA_WEB3,
	PAGINA_WEB4,
	QUERY1,
	QUERY2,
	QUERY3,
	QUERY4
};

//--------------------------------------------------MENSAJES-----------------------------------------------------

unsigned char mensaje[60][120] ={
		"ready\r\n",				
		"WIFI GOT IP\r\n",			
		"OK\r\n",					
		"ERROR",				
		"busy\r\n",					
		"SEND OK\r\n",				
		">",					
		"AT\r\n",					
		"AT+CWMODE=3\r\n",			
		"AT+CWJAP=\"LJ\",\"12345678\"",
		"AT+CIPSTA_DEF=\"192.168.43.70\",\"192.168.43.1\",\"255.255.255.0\"",
		"AT+CIPMUX=1\r\n",			
		"AT+CIPSERVER=1,80\r\n",	
		"AT+CIFSR\r\n",				
		"AT+CIPSEND=",				
		"AT+CIPCLOSE=",
		"AT+CIPSTART=0,\"TCP\",\"172.16.101.132\",80\r\n",
		":GET /",					
		"+IPD,",					
		",",						
		":",						
		".",						
		" ",						
		"MENSAJE RECIBIDO: ",		
		"CANAL RECIBIDO: ",			
		"\r\n",						
		"BYTES RECIBIDOS: ",		
		"LED ROJO ON\r\n",				
		"LED AZUL ON\r\n",
		"LED VERDE ON\r\n",
		"LED BLANCO ON\r\n",
		"LED AMARILLO ON\r\n",
		"LED OFF\r\n",
		"FOCO ON",
		"FOCO OFF",
		"COMANDO NO RECONOCIDO\r\n",
		"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: Close\r\n\r\n<html><head><title>ESP8266</title><body><p>",
		"</p><p>",
		"</p><p>",
		"</p></body></html>",
		"GET /SemestreI/psrLamp.php?idLamp=default&IP=172.16.101.131&MAC=dc:4f:22:10:f6:a8&",
		"fechaAct=now()&zonaUbic=Guadalajara&Status=",
		"&Volt=",
		" HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n"
		//"<html><head><title>ESP8266 LED Control</title></head><body><script src=\"https://code.jquery.com/jquery-2.1.4.min.js\"></script><header size=32>ESP8266 - K64F Equipo 1 connexión</header><button id=\"1\" class=\"led\">Light On</button><button id=\"0\" class=\"led\">Light Off</button><button id=\"R\" class=\"led\">LED Red</button><button id=\"G\" class=\"led\">LED Green</button><button id=\"B\" class=\"led\">LED Blue</button><button id=\"O\" class=\"led\">LED Off</button><p id=\"LightStatus\">",
		//"</p><p id=\"LEDStatus\">",
		//"</p><script type=\"text/javascript\">$(document).ready(function(){$(\".led\").click(function(){var status = $(this).attr('id');switch (status) {case '1':$.get(\"http://172.16.101.131/1!\");break;case '0':$.get(\"http://172.16.101.131/0!\");break;case 'R':$.get(\"http://172.16.101.131/R!\");break;case 'G':$.get(\"http://172.16.101.131/G!\");break;case 'B':$.get(\"http://172.16.101.131/B!\");break;case 'O':$.get(\"http://172.16.101.131/O!\");break;default:break;}});});</script></body></html>"
};
//--------------------------------------------------VARIABLES GLOBALES-----------------------------------------------------

unsigned char temp;					//Arreglo donde se guardará información medoral
unsigned char mensaje_enviado[800];
unsigned char buffer[800];
unsigned char canal_recibido[2];
unsigned char bytes_recibidos[4];
unsigned char informacion_recibida[40];	//Arreglo multidimencional donde se guardarán los tres datos importantes de la comunicación: Canal, Datos recibidos y el mensaje
unsigned char* pointer_UART0;
unsigned char* pointer_UART1;
unsigned char count_parser, count_parser2;					//Variable para llevar la cuenta de la comparación del parser
unsigned int count_msg;					//Variable para llevar la cuenta para almacenar los datos recibidos
unsigned int byte_size;						
unsigned char tx_status;					//Variable que indica si se está mandando datos 
unsigned char actualState;					//Variable para la máquina de estados
unsigned int medicion, medicion2;
unsigned char voltaje[6];
unsigned char lamp_status;
unsigned int sensor1_ref = 40000;
unsigned int sensor2_ref = 20000;
//--------------------------------------------------DECLARACIÓN DE FUNCIONES-----------------------------------------------------

void vUART_init(void);
void copy_AtoB(unsigned char *arrayA, unsigned char *arrayB);
unsigned char* integer_to_array(unsigned int dato,unsigned char *array);
unsigned int array_to_integer(unsigned char *array);
void clear_array(unsigned char *array);
unsigned int count_array(unsigned char *array);
void setLamp(unsigned char estado);
void setLEDColor(unsigned char color);
void setESP8266(unsigned char estado);
void UART0_send_msg(unsigned char msg_ID);
void UART0_send_msg_address(unsigned char* msg_address);
void UART1_send(unsigned char* msg);
void UART1_send_msg(unsigned char msg_ID);
void UART1_send_msg_address(unsigned char* msg_address);
unsigned char UART1_parser(unsigned char msg_ID);
unsigned char UART1_parser2(unsigned char msg_ID);
void join_String(unsigned char * String1, unsigned char * String2);
unsigned char* format(unsigned char msg_ID, unsigned char* buff );
unsigned int convertVoltage(unsigned int med);
void vUART_send_Console(unsigned char dato);
void vUART_send_msg_Console (unsigned char *a);

//--------------------------------------------------MAIN-----------------------------------------------------

int main(void)
{
	vUART_init();				//Inicializar entradas y salidas
	setESP8266(OFF);	
	setLamp(OFF);
	setLEDColor(OFF);			//Iniciar con todas las salidas apagadas
	setESP8266(ON);			//Resetear el módulo Wifi
	tx_status = TX_READY;
	actualState = WAIT_IP;		//Inicio de la máquina de estados
	count_parser = 0;
	count_parser2 = 0;
	count_msg = 0;
	//UART1_send_msg(AT);			//Mandar el primer mensaje
	
	while(1){
								//Loop infinito
	}
	return 0;
}

//--------------------------------------------------ISR´s-----------------------------------------------------

void UART0_Status_IRQHandler(){
	if(UART0_S1&(1<<7)){				//Revisar si se puede escribir en la UART0
		UART0_D=*(pointer_UART0++);
		if(*pointer_UART0==0){
				tx_status = TX_READY; //Indica que ya terminó de mandar el mensaje
				UART0_C2&=~(1<<7);	  //Deshabilitar interrupción
			}
	}
}
void UART1_Status_IRQHandler(){
	if(tx_status == TX_BUSY){			//Solamente puede entrar a esta función si se mandó algo 
	if(UART1_S1&(1<<7)){				//Revisar si se puede escribir en la UART1
		UART1_D=*(pointer_UART1++);
		//UART0_D=*(pointer_UART1++);
		if(*pointer_UART1==0){
				tx_status = TX_READY;	//Indica que ya terminó de mandar el mensaje
				UART1_C2&=~(1<<7);		//Deshabilitar interrupción
			}
	}
	}
	if(UART1_S1&(1<<5)){
		do{}while(tx_status == TX_BUSY);		//Esperar por si todavía se está mandando algo
		
		temp = UART1_D;					//Guardar medoralmente el dato
		
		switch(actualState){
		
		case WAIT_READY:
			if(UART1_parser(READY) == 1){		//Esperar a "ready" y mandar un "AT"
				actualState = WAIT_OK;
				UART1_send_msg(AT);
			}
			break;
		case WAIT_IP:
			if(UART1_parser(WIFI_GOT_IP) == 1){		//Esperar a "ready" y mandar un "AT"
				actualState = WAIT_OK;
				UART1_send_msg(AT);
			}
			break;
		case WAIT_OK:
			if(UART1_parser(OK) == 1){			//Esperar un "OK" y mandar "AT+CIPMUX=..."
				actualState = CONF_MUX;
				UART1_send_msg(AT_CIPMUX);
			}
			break;
		case CONF_MUX:		
			if(UART1_parser(OK) == 1){			//Esperar un "OK" y mandar un "AT+CIPSERVER=..."
				actualState = CONF_SERVER;
				UART1_send_msg(AT_CIPSERVER);
			}
			break;
		case CONF_SERVER:						//Esperar un "OK"
			if(UART1_parser(OK) == 1){
				actualState = SHOW_IP; //SHOW_IP;
				UART1_send_msg(AT_CIFSR);
			}
			break;
		case SHOW_IP:						//Esperar un "OK"
			if(UART1_parser(OK) == 1){
				actualState = WAIT_HEADER;	//Este comando arroja un error
			}
			break;
		case WAIT_HEADER:						//Esperar un "+IPD,"
			if(UART1_parser(IPD) == 1){
				actualState = WAIT_CHANNEL;
				count_msg = 0;
			}
			break;
		case WAIT_CHANNEL:						//Almacena el canal recibido (máximo 2 caracteres) hasta que encuentra una coma
			if(UART1_parser(COMA) == 1){
				canal_recibido[count_msg] = 0;
				count_msg = 0;
				actualState = WAIT_BYTES;
			}
			else canal_recibido[count_msg++] = temp;
			break;
		case WAIT_BYTES:											//Almacena los bytes recibidos (máximo 4 caracteres) hasta que encuentra dos puntos
			if(UART1_parser(DOS_PUNTOS) == 1){
				canal_recibido[count_msg] = 0;
				byte_size = array_to_integer(&bytes_recibidos[0]); //Convierte el arreglo de bytes_recibidos a entero
				count_msg = 0;
				actualState = WAIT_INFO;
			}
			else bytes_recibidos[count_msg++] = temp;
			break;
		case WAIT_INFO:
			if(count_msg < 40) informacion_recibida[count_msg] = temp; 	//Solamente almacena los primeros 40 caracteres recibidos
			if(count_msg < byte_size) count_msg++;					//Deja pasar los demás datos, pero espera a que todos hayan sido recibidos
			if (count_msg >= byte_size){
				
				//---------------------Hacer lo que queramos con el mensaje aquí-------------
				byte_size = 0;
				count_msg = 0;
				actualState = WAIT_SEND;
				copy_AtoB(format(PAGINA_WEB1,buffer), mensaje_enviado);		//Juntar el html con toda la infromación en un solo string
				UART1_send_msg_address(format(AT_CIPSEND, buffer));			//Mandar AT´CIPSEND=0,x =
				
			}
			break;
		case WAIT_SEND:						//Esperar un ">" para mandar un mensaje
			if(UART1_parser(ARROW) == 1){
				//setLamp(TOOGLE);
				
				//UART1_D = bytes_recibidos[0];
				//UART1_send_msg(MENSAJE_RECIBIDO);
				UART1_send(mensaje_enviado);	//Mandar el mensaje

				
				actualState = WAIT_SEND_OK;
			}
			break;
		case WAIT_SEND_OK:						//Esperar un "SEND OK" para terminar la comunicacion
			if(UART1_parser(SEND_OK) == 1){
				//setLamp(TOOGLE);

				UART1_send_msg_address(format(AT_CIPCLOSE, buffer));

				actualState = WAIT_HEADER;
				clear_array(canal_recibido);
				clear_array(bytes_recibidos);
				clear_array(informacion_recibida);
			}
			break;
		case WAIT_CONNECTION:
			if(UART1_parser(OK) == 1){
				count_msg = 0;
				actualState = WAIT_HEADER;
				clear_array(canal_recibido);
				canal_recibido[0]='0';
				copy_AtoB(format(QUERY1,buffer), mensaje_enviado);
				vUART_send_msg_Console(mensaje_enviado);
				//UART1_send_msg_address(format(AT_CIPSEND, buffer));
				
				UART1_send_msg_address(format(AT_CIPCLOSE, buffer));
				/*actualState = WAIT_SEND;
				copy_AtoB(format(QUERY1,buffer), mensaje_enviado);		//Juntar el html con toda la infromación en un solo string
				UART1_send_msg_address(format(AT_CIPSEND, buffer));			//Mandar AT´CIPSEND=0,x =
				*/
			}
			if(UART1_parser2(ERROR) == 1){
				count_msg = 0;
				actualState = WAIT_HEADER;
			}
			break;
		default:
			break;		
		}
		UART0_D = temp;					//Enviar el dato a la consola		
	}
}
void LPTimer_IRQHandler(void){
	LPTMR0_CSR|=(1<<7);					//Apagar bandera del TIMER
	ADC1_SC1A=(1<<6)+18;				//Activar conversión del canal 18 y activar interrupción
			
}

void ADC1_IRQHandler(void){
	ADC1_SC1A=(1<<7)+0x1F;	//Turn off COCO	and stop conversion
	medicion = ADC1_RA;			//RA is the data output
	if(medicion > sensor1_ref) {
		setLamp(OFF);
		lamp_status = FOCO_OFF;
	}
	else{
		setLamp(ON);
		lamp_status = FOCO_ON;
	}
		
	ADC0_SC1A=(1<<6)+19; //pag 121 el pin está conectado a ADC0_DM0
	
	
	
}


void ADC0_IRQHandler(void){
	ADC0_SC1A=(1<<7)+0x1F;	//Turn off COCO	and stop conversion
	medicion2 = ADC0_RA;			//RA is the data output
	
	if(medicion2 > sensor2_ref && lamp_status == FOCO_OFF) lamp_status = ERROR;
	if(medicion2 < sensor2_ref && lamp_status == FOCO_ON) lamp_status = ERROR;
	setLEDColor(TOOGLE);
	
	UART1_send_msg(AT_CIPSTART);
	actualState = WAIT_CONNECTION;
	
}

//--------------------------------------------------FUNCIONES PRINCIPALES-----------------------------------------------------
void vUART_send_Console(unsigned char dato){
	do{}while(!(UART0_S1& 0x80));
	UART0_D=dato; //mandar 'A'
}
void vUART_send_msg_Console (unsigned char *a){
	unsigned char i = 0;
	do{
		vUART_send_Console(a[i++]); 
		//i++;
	}while(a[i] != 0);
}


void UART0_send_msg(unsigned char msg_ID){
	do{}while(tx_status == TX_BUSY);
	tx_status = TX_BUSY;
	if(msg_ID == MENSAJE){
		pointer_UART0=&informacion_recibida[0];
	}
	else pointer_UART0=&mensaje[msg_ID][0];				//Copia la dirección del arreglo al puntero global
	UART0_C2|=1<<7; 								//Enable interruption for UART1_Tx
	
}
void UART0_send_msg_address(unsigned char* msg_address){
	//pointer_UART0=&mensaje[msg_ID][0];
	do{}while(tx_status == TX_BUSY);
	tx_status = TX_BUSY;
	pointer_UART0=msg_address;				//Copia la dirección del arreglo al puntero global
	UART0_C2|=1<<7;
	//UART0_C2|=1<<7; 								//Enable interruption for UART1_Tx
}
void UART1_send_msg(unsigned char msg_ID){
	//pointer_UART0=&mensaje[msg_ID][0];
	do{}while(tx_status == TX_BUSY);
	tx_status = TX_BUSY;
	pointer_UART1=&mensaje[msg_ID][0];				//Copia la dirección del arreglo al puntero global
	UART1_C2|=1<<7;
	//UART0_C2|=1<<7; 								//Enable interruption for UART1_Tx
}
void UART1_send_msg_address(unsigned char* msg_address){
	//pointer_UART0=&mensaje[msg_ID][0];
	do{}while(tx_status == TX_BUSY);
	tx_status = TX_BUSY;
	pointer_UART1=msg_address;				//Copia la dirección del arreglo al puntero global
	UART1_C2|=1<<7;
	//UART0_C2|=1<<7; 								//Enable interruption for UART1_Tx
}
unsigned char UART1_parser(unsigned char msg_ID){
	if(temp == mensaje[msg_ID][count_parser]) count_parser++; //Si el nuevo dato es igual a la letra siguiente, aumenta la variable global
	else count_parser = 0;											//Si no, vuelve a iniciar

	if(mensaje[msg_ID][count_parser] == 0){
		count_parser = 0;
		return 1;													//Regresa un 1 si encontró la palabra
	}
	else return 0;													//Si no, regresa un 0
}

unsigned char UART1_parser2(unsigned char msg_ID){
	if(temp == mensaje[msg_ID][count_parser2]) count_parser2++; //Si el nuevo dato es igual a la letra siguiente, aumenta la variable global
	else count_parser2 = 0;											//Si no, vuelve a iniciar

	if(mensaje[msg_ID][count_parser2] == 0){
		count_parser2 = 0;
		return 1;													//Regresa un 1 si encontró la palabra
	}
	else return 0;													//Si no, regresa un 0
}

void UART1_send(unsigned char* msg){
	unsigned char i = 0;
	do{
		do{}while(!(UART1_S1& 0x80));
		UART1_D=msg[i]; //mandar 'A'
		i++;
	}while(msg[i] != 0);
}

//--------------------------------------------------FUNCIONES SECUNDARIAS-----------------------------------------------------

void vUART_init(void)
{
	//Configurar UART  
    SIM_SCGC4=0x00000C00; //Hab clk UART0 y UART1
    UART0_BDH=0;
    UART0_BDL=11;
    UART1_BDH=0;
    UART1_BDL=11;
    //UART0_C1=0;
    UART0_C2=12;           // bit 3: Hab Tx, bit 2: Hab Rx, Habilitar interrupción del Reciever
    UART1_C2=12 + (1<<5); // bit 3: Hab Tx, bit 2: Hab Rx, Habilitar interrupción del Reciever
    
    NVICICER0=(1<<(31%32));		//Configurar NVIC para UART0
    NVICISER0|=(1<<(31%32));
    
    NVICICER1=(1<<(33%32));		//Configurar NVIC para UART1
    NVICISER1|=(1<<(33%32));
    
    //Configurar pines
    SIM_SCGC5=0x00002C00 + 1; //Hab clk PORTB (PB16 y 17 son Rx y Tx) y PORTC
    PORTB_PCR16=0x00000300; //Hab clk PB16 Rx
    PORTB_PCR17=0x00000300; //Hab clk PB17 Tx
    PORTC_PCR3=0x00000300; //Hab clk PC3 Rx
    PORTC_PCR4=0x00000300; //Hab clk PC4 Tx
 
    
    PORTB_PCR21 = 0x00000100;	//Seleccionar pin como GPIO 
    PORTB_PCR22 = 0x00000100;
    PORTC_PCR2 =  0x00000100;
    PORTC_PCR5 = 0x00000100;
    PORTC_PCR7 = 0x00000100;
    PORTE_PCR26 = 0x00000100;
    GPIOB_PDDR = 0x00600000;//Dirección del pin (ouput)
    GPIOE_PDDR = 0x04000000;
    GPIOC_PDDR = 0x000000A4;
    
    //Configurar LPTMR
    LPTMR0_PSR = 0x05; //Configurar LPTMR
    LPTMR0_CSR = (1<<6) + 1;; //Habilitar LPTMR e Interrupción
    LPTMR0_CMR = 2000;
    
	NVICICER1=(1<<(58%32));		//Configure NVIC for LPTMR0
	NVICISER1|=(1<<(58%32));	//Enable NVIC
	
	
	//Configurar ADC1
	SIM_SCGC3|= 1<<27;			//clock ADC1
	ADC1_CFG1=0x0C;				//Conversión de 16 bits
	NVICICER2=(1<<(73%32));		//Configure NVIC for ADC1
	NVICISER2|=(1<<(73%32));	//Enable NVIC
	
	//Configurar ADC0
	SIM_SCGC6|= 1<<27;			//clock ADC1
	ADC0_CFG1=0x0C;				//Conversión de 16 bits
	NVICICER1=(1<<(39%32));		//Configure NVIC for ADC1
	NVICISER1|=(1<<(39%32));	//Enable NVIC
	
	
}
void copy_AtoB(unsigned char *arrayA, unsigned char *arrayB){		//Copia la información del arreglo A al arreglo B
	clear_array(arrayB);
	int i = 0;
	do{
		arrayB[i]=arrayA[i];
	}while(arrayA[++i] != 0);
}
unsigned char* integer_to_array(unsigned int dato,unsigned char *array){ //Convierte de decimal a un arreglo
	clear_array(array);
	unsigned char  digit;
    unsigned char i = 0;
        if (dato == 0)					//Si el dato es 0, pone automaticamente 0
        {
            array[i++] = '0';
            array[i] = 0;
            return array;
        }
        if(dato / 10 == 0){				//Si el dato es menor a 9, pone automaticamente el dígito
            array[i++] = dato + '0';
            array[i] = 0;
            return array;
        }
        do{
            digit = dato % 10; 			//Multiplica el valor anterior por 10
            dato /= 10;			
            //Divide el dato entre 10 
            array[i++] = digit + '0';	//Suma el valor actual más el valor anterior
            }while(dato != 0);
        array[i] = 0;
        unsigned char inicio, fin, c;
        fin = i-1;
        inicio = 0;
        do{								//Invierte el orden del arreglo
            c = array[inicio];
            array[inicio] = array[fin];
            array[fin] = c;
        }while(++inicio < --fin);
        return array;
    }
unsigned int convertVoltage(unsigned int med){
	unsigned int dato = (med * 3300)/65500;
	return dato;	
}
unsigned int array_to_integer(unsigned char *array){
	
	unsigned int val = 0;
	unsigned char i = 0;
	
	do{
		val *= 10;
	    val += array[i] - '0';
	}while(array[++i] != 0);
	return val;
}
void clear_array(unsigned char *array){							//Limpia el arreglo recibido
	unsigned char i=0;
	do{
		array[i] = 0;
	}while(array[++i] != 0);
}
unsigned int count_array(unsigned char *array){					//Recibe un Arreglo de caracteres y cuenta la cantidad de datos
	
	unsigned int i = 0;
	do{}while(array[++i] != 0);
	return i;
	
}
void setLamp(unsigned char estado){
	if(estado ==  OFF) {
		GPIOC_PDOR &= ~(1 << 2);	//Apagar relay (Encender bit 2)
		GPIOC_PDOR |= 1 << 7;
	}
	if(estado ==  ON){
		GPIOC_PDOR |= 1 << 2;
		GPIOC_PDOR &= ~(1 << 7);
	}
	if(estado == TOOGLE){
		GPIOC_PTOR |= (1 << 2); //TOOGLE
		GPIOC_PTOR |= (1 << 7);
	}
	
}
void setESP8266(unsigned char estado){
	if(estado ==  OFF) GPIOC_PDOR &= ~(1 << 5); 	//Apagar relay (Encender bit 2)
	if(estado ==  ON) GPIOC_PDOR |= 1 << 5;
	if(estado == RESET){
		GPIOC_PDOR &= ~(1 << 5); 
		GPIOC_PDOR |= 1 << 5;		//Resetear el ESP8266
	}
}
void setLEDColor(unsigned char color){
	switch(color){
	case RED:
		GPIOB_PDOR = 0x00200000;
		GPIOE_PDOR = 0x04000000;
		break;
	case BLUE:
		GPIOB_PDOR = 0x00400000;
		GPIOE_PDOR = 0x04000000;
		break;
	case GREEN:
		GPIOB_PDOR = 0x00600000;
		GPIOE_PDOR = 0x00000000;
		break;
	case WHITE:
		GPIOB_PDOR = 0x00000000;
		GPIOE_PDOR = 0x00000000;
		break;
	case YELLOW:
		GPIOB_PDOR = 0x00200000;
		GPIOE_PDOR = 0x00000000;
		break;
	case OFF:
		GPIOB_PDOR = 0x00600000;
		GPIOE_PDOR = 0x04000000;
		break;
	case PINK:
		GPIOB_PDOR = 0x00000000;
		GPIOE_PDOR = 0x04000000;
		break;
	case CYAN:
		GPIOB_PDOR = 0x00400000;
		GPIOE_PDOR = 0x00000000;
		break;
	case TOOGLE:
		GPIOB_PTOR |= 1<<21;
		break;
	default: //OFF
		GPIOB_PDOR = 0x00600000;
		GPIOE_PDOR = 0x04000000;
		break;
	}
	// 31 30 29 28 || 27 26 25 24 || 23 22 21 20 || 19 18 17 16 || 15 14 13 12 || 11 10 09 08 || 07 06 05 04 || 03 02 01 00
	//Led Red --- PTB22
	//Led Blue -- PTB21
	//Led Green - PTE26
}


void join_String(unsigned char * String1, unsigned char * String2){
	unsigned char i = 0;
	unsigned char j = 0;
	do{}while(String1[++i] != 0);
	do{String1[i++] = String2[j];}while(String2[++j] != 0);
	String1[i] = 0;
	}

unsigned char* format(unsigned char msg_ID, unsigned char* buff ){
	clear_array(buff);
	if(msg_ID == AT_CIPSEND){
		copy_AtoB(mensaje[AT_CIPSEND], buff);
		join_String(buff, canal_recibido);
		join_String(buff, mensaje[COMA]);
		join_String(buff,integer_to_array(count_array(mensaje_enviado), bytes_recibidos));
		join_String(buff, mensaje[ENTER]);
	}
	if(msg_ID == AT_CIPCLOSE){
		copy_AtoB(mensaje[AT_CIPCLOSE], buff);
		join_String(buff, canal_recibido);
		join_String(buff, mensaje[ENTER]);
	}
	if(msg_ID == PAGINA_WEB1){
		copy_AtoB(mensaje[PAGINA_WEB1], buff);
		join_String(buff, integer_to_array(medicion, voltaje));
		join_String(buff, mensaje[PAGINA_WEB2]);
		join_String(buff, integer_to_array(medicion2, voltaje));
		join_String(buff, mensaje[PAGINA_WEB3]);
		join_String(buff, mensaje[lamp_status]);
		join_String(buff, mensaje[PAGINA_WEB4]);
	}
	if(msg_ID == QUERY1){
		copy_AtoB(mensaje[QUERY1], buff);
		copy_AtoB(mensaje[QUERY2], buff);
		join_String(buff, mensaje[lamp_status]);
		join_String(buff, mensaje[QUERY3]);
		join_String(buff, integer_to_array(medicion, voltaje));
		join_String(buff, mensaje[QUERY4]);
	}
	return buff;
}

