#include <18F65J94.h>
#device ADC=8

#FUSES WDT                    //No Watch Dog Timer
#FUSES WDT1024                   //Watch Dog Timer uses 1:128 Postscale
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES FRC

#use delay(internal=32M)
//!//#use delay(crystal=32000000)
#build (reset=0x400, interrupt=0x408)
#org 0,0x3FF{}

#pin_select U4RX = PIN_D0
#pin_select U4TX = PIN_D1

#pin_select U2RX = PIN_D2
#pin_select U2TX = PIN_D3

//#pin_select INT1 = PIN_B1 // OK
//!#pin_select INT1 = PIN_B1 // UP
//!#pin_select INT2 = PIN_B2 // DOWN
//!#pin_select INT3 = PIN_B3 // LEFT
//#pin_select P6A = PIN_E6

#use rs232(baud=9600,parity=N,UART2, bits=8,stream=syrus,timeout=20) // 
#use rs232(baud=9600,parity=N,UART4, bits=8,stream=accesory,timeout=20) // 

#use i2c(Master,I2C2,FAST,RESTART_WDT)

#include <string.h>
#include <stdlib.h>
#include <external_eeprom.h>
#include <7seg_dp_4dig_V5.h>
#include <MCP4541.h>
#include <test.h>
//#include <hex_loader.h>

#define in_0 PIN_C0 // input 0 quinta rueda
#define in_1 PIN_C1 // input 1 puertas
#define in_2 PIN_C2 // input 2 retro del tamper, activa contador y despues de desbordarse activa salida 0
#define in_3 PIN_C3 // input 3 monitorea la apertura del cofre
#define in_4 PIN_C5 // input 4 entrada de voltaje
#define out_0 PIN_C6 // activacion de la valvula
#define out_1 PIN_C7 // salida permanente mientras MCU este energizado
#define led PIN_A4
#define ok PIN_B0
#define up PIN_B1
#define down PIN_B2
#define left PIN_B3
#define right PIN_B4
#define cancel PIN_B5

// direcciones de memoria de la eeprom
#define password_1_address 0 // OK
#define password_2_address 2 // OK
#define password_3_address 4 // OK
#define password_4_address 6 // OK
#define status_word_address 8 // OK
#define input_inverted_address 9
#define tries_address 10
#define blocked_keypad_address 11
#define test_addres 20
#define test_copilot_door_flag_address 22
#define security_enabled_flag_address 24
#define an_out_time_address 26
#define umbral_debounce_copilot_door_address 28
#define out_0_time_address 30
#define input4_enable_address 32
#define input3_enable_address 34
#define type_holland_address 36
#define v3_time_address 38
#define v3_enable_address 40
#define password_5_address 42 // OK
#define password_6_address 44 // OK
#define password_7_address 46 // OK
#define password_8_address 48 // OK
#define in_0_address 50
#define bpc2_address 52
#define in_1_address 56
#define in_3_address 58
#define bpp_flag_option_address 60
#define bpc1_address 62
#define exe_bpc1_flag_address 64
#define exe_bpc2_flag_address 66
#define exe_bpp_flag_address 68
#define bpc1_overtime_time_address 70
#define bpc2_overtime_time_address 72
#define bpc1_time_address 74
#define bpc2_time_address 76
#define bpp_time_address 78
#define exe_monitor1_flag_address 80
#define exe_monitor3_flag_address 82
#define out0_option_address 84
#define out0_cmd_mode_address 86
#define out0_cmd_mode_last_stat_address 88
#define wrong_pass_address_1 90
#define wrong_pass_address_2 92
#define wrong_pass_address_3 94
#define wrong_pass_address_4 96
#define wrong_pass_address_5 98
#define temporal_out0_off_flag_address 100
#define temporal_out0_off_transition_address 102
#define out_0_activated_address 104
#define password_master_address 106
#define password_master_used_address 108
#define serial_address_flag 124
#define serial_address 200
#define block_by_bad_pass_used_address 250
#define acc_detected_address 252
#define acc_lost_counter_address 254
#define acc_lost_max_mgs_counter_address 256
#define acc_lost_long_msg_timer_address 258
//#define test_data 0xFFFF

int16 display_number; // Numero que sera desplegado en la pantalla.
int digit_position=3; // posicion del punto decimal
int16 u,d,c,m;
char rst_micro[]="*rst#";
short data_buffer=false; // Bandera que indica si hay datos en el buffer
short hex_loader_flag = false;
char syrus_buffer[1023]; // bufer donde se alamcenan los datos entrantes al puerto serial
int16 index=0; // indice que indica en que nivel de la pila se esta guardando el dato en el puerto serial

/*variables para grabar cadenas grandes en eeprom*/
short hex_data_buffer = false;
char length;
int16 syrus_buffer_index;
int16 hex_eemprom_address = 1000;
int16 hex_eemprom_index = 0;
int16 next_eeprom_page = 50;
int16 total_arrows;



//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////        DEFINICION DE FUNCIONES DEL PRGRAMA           /////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void scan_cmd(); // Cuando hay una cadena posible valida en el buffer, se analiza en esta funcion si pertenece a algun comando valido
void scan_update();
void send_to_buffer_syrus(char c); // Recibe el caracter que se lee por el puerto serial y lo almacena en un buffer para su posterior procesamiento
void make_digits(); // Toma una variable de 16bit y la convierte en unidades, decenas, centenas y millares para desplegarla en los displays
void init_interrupts(); // Activa las interrupciones del programa
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////         VARIABLES DEL PRGRAMA           ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////        INTERRUPCIONES DEL PRGRAMA           ///////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

#INT_TIMER0
void  RTCC_isr(void) 
{
   restart_wdt(); 
   display_number++;
   set_timer0(3036);
}

#INT_TIMER2 
void  TIMER2_isr(void) 
{
   //output_toggle(led);
   restart_wdt(); 
   make_digits();
   display_7seg(display_number,digit_position);
}



#int_rda2 // Syrus
void serial_isr2() 
{
   char c;

   restart_wdt(); 
   if(kbhit(syrus))
   {
      c=fgetc(syrus);
         {
            send_to_buffer_syrus(c);
         }
   }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////        FUNCIONES DEL PRGRAMA           ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////




void scan_cmd()
{
  int16 length=0;
  int16 i=0;
  int j=0;
  int letter_index;
  int password_test_nib1;
  int password_test_nib0;
  int16 password_master_counter;
  char char_password[10];
  char char_password_master[10];
  char letter_pass[17];
  int16 password_test;
  int16 password_test_counter;
  int16 password_test_master;

   restart_wdt(); 
  if(data_buffer==true)
  {
    disable_interrupts(global);
    strlwr (syrus_buffer);

    while(syrus_buffer[i]!='#')
    {
      i++;
      length++;
    }
    length++;  

    if((syrus_buffer[0]=='*')&&(syrus_buffer[length-1]=='#'))
    {         
      if(!strncmp(rst_micro, syrus_buffer, length))
      {
        fprintf(syrus,"\r\nMensaje recibido");  
        hex_loader_flag = true;
      }
    }

    index=0;
    data_buffer=false;
    enable_interrupts(global);  
  }  

  if(hex_data_buffer==true) // Si hay un dato en buffer que puede ser un paquete de programacionñ
  {
    disable_interrupts(global); // deshabilita las interrupciones para dedicarse a grabar en la eeprom
    //fprintf(syrus, "00\r\n");
    while(syrus_buffer[i]!='&')
    {
      i++;
      length++;
    }
    length++;  
    //fprintf(syrus, "01\r\n");
    syrus_buffer_index = 1;
    hex_eemprom_index = 0;
    total_arrows = 0;
    if((syrus_buffer[0]=='*')&&(syrus_buffer[length-1]=='&'))
    {      
      //fprintf(syrus, "02\r\n");
      while(syrus_buffer[syrus_buffer_index]==':')
      {
        //fprintf(syrus, "03\r\n");
        restart_wdt(); 
        do{
          //fprintf(syrus, "04\r\n");
          restart_wdt(); 
          //hex_buffer[j] = syrus_buffer[syrus_buffer_index];
          write_ext_eeprom(hex_eemprom_address+hex_eemprom_index,syrus_buffer[syrus_buffer_index]);   
          syrus_buffer_index++;
          hex_eemprom_index++;
        }while((syrus_buffer[syrus_buffer_index]!=':')&&(syrus_buffer[syrus_buffer_index+1]!='&'));
        hex_eemprom_address += next_eeprom_page;
        total_arrows++;
        hex_eemprom_index = 0;
        fprintf(syrus, "L%04Lu OK\r\n", total_arrows);
      }
    }
    enable_interrupts(global);  
    hex_data_buffer = false;
    index=0;
  } 
}

void send_to_buffer_syrus(char c)
{ 
   restart_wdt(); 
   syrus_buffer[index]=c; // Se alamcena el caracter entrante en el ultimo nivel de la pila del buffe
//!   if(index>=syrus_buffer) 
//!   {
//!      index = 0;
//!   }
//!   else
//!   {
      if(syrus_buffer[index]=='#') // si el caracter entrante es '#' se considera final de cadena y se analiza el bufer
      {
         data_buffer=true; // activando la bandera de 'Dato listo en buffer'
      }
      else if(syrus_buffer[index]=='&')
      {
        hex_data_buffer=true;
      }
      else
      {     
         index++;   
      }
//!   }   
}


void make_digits()
{
   restart_wdt(); 
   m = display_number / 1000;
   c = (display_number -(m*1000))/100;
   d = (display_number - (m*1000 + c*100))/10;
   u = display_number -(m*1000 + c*100 + d*10 );
}

void init_interrupts()
{
   restart_wdt(); 
   enable_interrupts(INT_RTCC);
   enable_interrupts(INT_TIMER2);
   enable_interrupts(INT_RDA2);
   enable_interrupts(GLOBAL);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//!////////////////////////////////        PROGRAMA PRINCIPAL           /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////


void main()
{
   setup_timer_0(T0_INTERNAL|RTCC_DIV_128);      //1.0 s overflow
   setup_timer_2(T2_DIV_BY_16,100,16);      // 1 ms overflow
   delay_ms(2010);
   restart_wdt();
   init_interrupts();
   display_number = 5000;
   turn_on_7seg();
   while(true)
   {
      // Monitorea si hay comandos en el buffer del puerto serial para ser analizados
      scan_cmd();
      
      scan_update();
   }
}


