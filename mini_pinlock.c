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
//#include <vpc.h>
#include <test.h>
//#include <MCP79410_2.h>

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



//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////        DEFINICION DE FUNCIONES DEL PRGRAMA           /////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void system_reset_tries(); // Resetea los intentos fallidos por contraseÃ±a erronea
void system_reset(); // Realiza un reset por software
void load_status_word(); // Carga los valores guardados en la eeprom externa
void test_input(); // Monitorea el estado de la entrada 0
void scan_cmd(); // Cuando hay una cadena posible valida en el buffer, se analiza en esta funcion si pertenece a algun comando valido
void send_to_buffer_syrus(char c); // Recibe el caracter que se lee por el puerto serial y lo almacena en un buffer para su posterior procesamiento
void unlock_gps(); // secuencia de desbloquedo para el gps. Activa la salida  0 de pinlock en la secuencia adecuada 
void test_ok(); // Monitorea si se intenta usar una contraseÃ±a
void test_keypad(); // monitorea el teclado continuamente
void start_debounce_time(); // impide dobles entradas al presionar un boton mecanico
void system_display_sleep(); // Contador para apagar la pantalla por falta de actividad
void display_error(); // Despliega en la pantalla el mensaje error
void display_ok(); // Despliega en la pantalla el mensaje ok
void display_block_timer(); // Muestra en la pantalla un contador regresivo cunado el teclado esta bloqueado
void make_digits(); // Toma una variable de 16bit y la convierte en unidades, decenas, centenas y millares para desplegarla en los displays
void init_pinlock(); // Carga los valores iniciales a las variables volatiles
void init_interrupts(); // Activa las interrupciones del programa
void test_input_2(); // Esta entrada monitorea la retro del tamper
void test_input_3(); // Esta entrada monitorea la deteccion de apertura del cofre
void test_input_4(); // Esta entrada monitorea la deteccion de ignicion o algun sensor que en estado activo tenga como seÃ±al un voltaje positivo
void bpc1();// Si la puerta del iloto esta abierta por mas de 15 minutos se estara enviando el voltaje de bloqueo cada 5 minutos.
void bpc2(); // Si la puerta del copiloto esta abierta por mas de 15 minutos se estara enviando el voltaje de bloqueo cada 5 minutos.
void bpp(); // si no se detecto apertura de puerta en el piloto X tiempo antes de haber encendido el motor entonces envia bloqueo
void send_to_buffer_accesory(char d);
void scan_cmd2();
void unlock_cargolock();
void lock_cargolock();
void send_cmd_00();
void test_input_1();
void lock_by_door();
void unlock_by_door();
void execute_v0();
void execute_v1();
void execute_v2();
void v3_by_ign_off(); // funcion para enviar voltaje de bloqueo por apagado de motor
void execute_v3(); // Voltaje V3 que envia por el pot digital 1.25V
void disable_all_security(); // Definicion de la funcion que ejecuta el V4
void execute_v4(); // Funcion que envia el voltaje V4 a travez del potenciometro digital
void map_eeprom();
void acc_monitor();
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////         VARIABLES DEL PRGRAMA           ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
const char init_msg[]="*901,"; // codigo de reinicio del dispositivo
const char msg_inicio[]="4.4.2.8"; // Mensaje de inicio
const char keep_alive_syrus[]="*115#";
const char msg_engache[]="*146#"; // Mensaje de quinta enganchada
const char msg_desengache[]="*147#"; // Mensaje de quinta desenganchada
const char msg_poll_enganche[]="*148,1#"; // Respuesta de poleo a quinta ganchada
const char msg_poll_desenganche[]="*148,0#"; // Respuesya de poleo a quinta desenganchada
const char invert_msg_answer[]="*149,1#"; // Respuesta a inversion de entrada de la quinta 
const char no_invert_msg_answer[]="*149,0#"; // Respuesta a no-inversion de la entrada de la quinta rueda
const char activate_security_doors_resp[]="*206#";// 
const char deactivate_security_doors_resp[]="*207#";//
const char msg_input1_act[]="*200#"; // Mensaje de entrada 1 activada por cambio de estado
const char msg_input1_deact[]="*201#"; // Mensaje de entrada 1 desactivada por cambio de estado
const char msg_poll_input1_act[]="*200#"; // Mensaje en respuesta a peticion si la entrada 1 esta activada
const char msg_poll_input1_deact[]="*201#"; // Mensaje en respuesta a peticion si la entrada 1 esta desactivada
const char msg_input3_act[]="*202#"; // Mensaje de entrada 3 activada
const char msg_input3_deact[]="*203#"; // Mensaje de entrada 3 desactivada
const char msg_poll_input3_act[]="*202#"; // Respuesta de poleo a entrada 3
const char msg_poll_input3_deact[]="*203#"; // Respuesya de poleo a entrada 3
const char msg_input4_act[]="*204#"; // Mensaje de entrada 4 activada - Motor encendido
const char msg_input4_deact[]="*205#"; // Mensaje de entrada 4 desactivada
const char msg_poll_input4_act[]="*204#"; // Respuesta de poleo a entrada 4
const char msg_poll_input4_deact[]="*205#"; // Respuesya de poleo a entrada 4
const char block_by_keypad[]="*193#"; // Bloqueo por teclado
const char enable_bpp_routine_msg_resp[]="*194#"; // Respuesta al comando para habilitar la funcion de Bloqueo Por Puenteo
const char disable_bpp_routine_msg_resp[]="*195#"; // Respuesta al comando para habilitar la funcion de Bloqueo Por Puenteo
const char bpp_send_msg[]="*221#"; // Bloqueo por puenteo detectado
const char bpc1_send_msg[]="*220#"; // Bloqueo por corte en puerta de piloto ejecutado
const char disable_bpc1_routine_msg_resp[]="*219#";// Respuesta comando aceptado para desactivar el Bloqueo por corte de puerta de piloto
const char enable_bpc1_routine_msg_resp[]="*218#"; // Respuesta comando aceptado para activar Bloqueo por corte de puerta de piloto
const char enable_bpc2_routine_msg_resp[]="*184#";
const char ans_2_activate_ign_transition[]="*210#"; // respuesta al comando de activacion de bloqueo por apagado de motor
const char ans_2_deactivate_ign_transition[]="*216#"; // respuesta al comando de desactivacion de bloqueo por apagado de motor
const char monitor_in3_enable_resp[]="*208#"; // Respuesta a la activacion del monitoreo de la entrada 3
const char monitor_in3_disable_resp[]="*209#"; // Respuesta a la cancelacion del monitoreo de la entrada 3
const char ios_state_resp[]="*222,";
const char cmd_disable_all_security_cmd[]="*213#"; // respuesta al comando para enviar el voltaje V4 y deshabilitar la seguridad en el GPS
const char bpc2_send_msg[]="*183#"; // Puerta abierta por mas de 15 minutos y sigue abierta, este mensaje se mandara casa 5 minutos
const char disable_bpc2_routine_msg_resp[]="*185#";
const char enable_exe_bpc1_resp[]="*223#"; // Envio de voltaje de bloqueo por bpc1 habilitado
const char disable_exe_bpc1_resp[]="*224#"; // Envio de voltaje de bloqueo por bpc1 deshabilitado
const char enable_exe_bpc2_resp[]="*225#"; // Envio de voltaje de bloqueo por bpc2 habilitado
const char disable_exe_bpc2_resp[]="*226#"; // Envio de voltaje de bloqueo por bpc2 deshabilitado
const char enable_exe_bpp_resp[]="*227#"; // Envio de voltaje de bloqueo por bpp habilitado
const char disable_exe_bpp_resp[]="*228#"; // Envio de voltaje de bloqueo por bpp deshabilitado
const char bpc1_tmrs_saved[]="*229,"; // Timers relacionados al bloqueo por corte en piloto grabados
const char bpc2_tmrs_saved[]="*230,"; // Timers relacionados al bloqueo por corte en copiloto grabados
const char bpp_tmr_saved[]="*231,"; // Timers relacionados al bloqueo por puenteo en piloto grabados
const char enable_exe_monitor1_resp[]="*232#"; // Respuesta a la activacion del envio de voltaje de bloqueo por apertura de puerta de piloto
const char disable_exe_monitor1_resp[]="*233#"; // Respuesta a la desactivacion del envio de voltaje de bloqueo por apertura de puerta de piloto
const char enable_exe_monitor3_resp[]="*234#"; // Respuesta a la activacion del envio de voltaje de bloqueo por apertura de puerta de copiloto
const char disable_exe_monitor3_resp[]="*235#"; // Respuesta a la desactivacion del envio de voltaje de bloqueo por apertura de puerta de copiloto
const char out0_option_activated[]="*247#"; // Respuesta a la activacion de la funcionalidad del segundo bloqueo del shield x3
const char out0_option_deactivated[]="*248#";// Respuesta a la desactivacion de la funcionalidad del segundo bloqueo del shield x3
const char msg_input1_act_2[]="*249#"; // Puerta abierta piloto - Funcionalidad de Bloqueo por apertura no activado
const char msg_input1_deact_2[]="*250#"; // Puerta cerrada piloto - Funcionalidad de Bloqueo por apertura no activado
const char msg_input3_act_2[]="*251#"; // Puerta abierta copiloto - Funcionalidad de Bloqueo por apertura no activado
const char msg_input3_deact_2[]="*252#"; // Puerta cerrada copiloto - Funcionalidad de Bloqueo por apertura no activado
const char msg_out0_on[]="*253#"; // Bloqueo secundario activado de manera forzada
const char msg_out0_off[]="*254#"; // Bloqueo secundario desactivado de manera forzada
const char msg_out0_cmd_mode_off[]="*255#"; // Salir de modo comando para el bloqueo secundario
const char cmd_error[]="*114,error#"; // respuesta de comando serial no identificado
const char wrong_pass_typed[]="*256,"; // Contraseñas introducidas y que no son correctas
const char cmd_pass_error[]="*257,error#"; // respues de contraseÃ±as no leidas correctamente
const char temporal_out0_off_resp[]="*258#"; // Desactivacion momentanea del paro de motor secundario controlado por el pinlock
const char out0_paused_once[]="*259#"; // Este comando se envia en lugar de haber activado el paro secundario. Solo se envia cuando la opcion de pausa esta activada
const char qry_master_pass_resp[]="*260,"; // Contrase�a maestra solicitada 
const char set_serial_resp[]="*261,"; // respuesta de confirmacion de numero de serie grabado en el dispositivo
const char bad_pass_used[]="*268,"; //Contrase�a ingresada incorrecta
const char waiting_confirmation[]="*269,"; // Numero que se quedo en la pantalla sin confirmar con la tecla OK
const char qry_display_number_resp[]="*270,"; // XXXX es la contrase�a que actualmente esta desplegada en la pantalla
const char block_by_bad_pass_used_enable_rep[]="*271#";
const char block_by_bad_pass_used_disable_rep[]="*272#";
const char read_options0_resp[]="*273,";
const char acc_0_disconnected[]="*275#";
const char remove_accesory_resp[]="*276#";
const char footer='#';

// COMANDOS QUE INTERPRETA EL CONTROLADOR DE MOTOR , SE INTEGRAN EN LA FUNCION scan_cmd() 
char blocking_box[]="*fw_lock#";
char unblocking_box[]="*fw_unlock#";
char define_model[]="*set_model,"; 
char qry_model[]="*qry_model#"; 
char qry_ios_acc[]="*qry_ios_dev01#";
char get_feedback_cmd[]="*get_feedback#";
char autolock_enable[]="*autolock,1#";
char autolock_disable[]="*autolock,0#";
char set_time_2_activate_autolock[]="*set_autolock=";
char set_lock_time_1_cmd[]="*set_lock_time_1=";
char get_interlock_status_msg[]="*get_interlock#";
//nuevos
char get_status[]="*qry_interlock_status#";
char qry_version[]="*qry_acc_version#";
char force_blocking_box[]="*forcelock#";
char set_offset_0[]="*set_offset_0#";
char set_offset_100[]="*set_offset_100#";
char inc_mov[]="*im#";
char dec_mov[]="*dm#";
char set_max_stroke_move[]="*set_max_move#";
char remmove_max_stroke_move[]="*remove_max_move#";
char read_acc0_options0[]="*acc0_rm0#";

// COMANDOS O RESPUESTAS QUE ENVIA EL CONTROLADOR DE MOTOR,  SE INTEGRAN EN LA FUNCION scan_cmd2()
//!const char version[]="*900,2.2.1.1#"; // Mensaje de inicio que indica la version del firmware
char accesory_178_msg[]="*178#";
char accesory_179_msg[]="*179#";
char accesory_180_msg[]="*180#";
char lock_ok[]="*236,"; // Bloqueo mayor al 90% confirmado
char unlock_ok[]="*237,"; // Desbloqueo menor al 10% confirmado
char position_stroke_msg[]="*238,"; // Respuesta a solicitud de la posicion fisica del bloqueo del quinta rueda
char lock_error[]="*239,"; // Bloqueo menor a 90%, se considera hubo algun problema mecanico.
char unlock_error[]="*240,"; // Desbloqueo mayor al 10%, no se considera que este desbloqueado aun
char set_model_resp[]="*241,";
char qry_model_resp[]="*242,";
char send_model[]="*243,";
char qry_ios_resp[]="*244,";
char indutives_status[]="*245,";
char blocking_box_ack[]="*170,ack#";
char unblocking_box_ack[]="*171,ack#";
char keep_alive_accesory[]="*173#";
char accesory_190_msg[]="*190#";
char accesory_191_msg[]="*191#";
char autolock_enable_resp[]="*187#";
char autolock_disable_resp[]="*188#";
char accesory_189_msg[]="*189#"; // autobloqueo al detectar enganche
char accesory_181_msg[]="*181#";
char accesory_182_msg[]="*182#";
//nuevos
char get_status_resp[]="*246,";
char force_blocking_box_resp[]="*262#";
char set_offset_0_resp[]="*263#";
char set_offset_100_resp[]="*264#";
char set_max_stroke_move_resp[]="*265#";
char remove_max_stroke_move_resp[]="*266#";
char read_acc0_options0_resp[]="*274,";
char version[]="*900,";

char invert_msg[]="*inv_input0,1#"; // Comando para invertir la seÃ±al de la quinta rueda
char no_invert_msg[]="*inv_input0,0#"; //  Comando para no invertir la seÃ±al de la quinta rueda
char read_status[]="*rd_status_word#"; //  
char poll_gnd_input[]="*rd_input0#"; // Leer el estado de la quinta
char rst_blocktimer[]="*rsttmr#"; //  REiniciar el bloqueo del teclado
char pass_request[]="*rqps#";
char rst_micro[]="*rst#";
char pass_received_pinlock[]="*111,"; //*111,
char pass_received_cargolock[]="*211,"; //*111,
char block_keypad; //  esta variable indica si el keypad esta o no bloqueado
char syrus_buffer[150]; // bufer donde se alamcenan los datos entrantes al puerto serial
char accesory_buffer[150]; // bufer donde se alamcenan los datos entrantes al puerto serial
char set_time_2_activate_autolock_resp[]="*277,";
char set_lock_time_1_cmd_resp[]="*locktime_after_sensor=";
char accesory_192_0_msg[]="*192,0#";
char accesory_192_1_msg[]="*192,1#";
char accesory_192_2_msg[]="*192,2#";
char accesory_192_3_msg[]="*192,3#";
char lock_msg_error[]="*fw_lock_error#";
char lock_msg_running[]="*fw_lock_running#";
char out_0_dly[]="*out0_dly=";
char inc_analog_time[]="*iat#";
char dec_analog_time[]="*dat#"; 
char activate_security_doors[]="*monitor_in1,1#";//
char deactivate_security_doors[]="*monitor_in1,0#";//
char poll_input1[] = "*rd_input1#";
char poll_input_3[]="*rd_input3#";
char monitor_in3_enable[]="*monitor_in3,1#"; // Activa el monitoreo de la entrada 3
char monitor_in3_disable[]="*monitor_in3,0#"; // Desactiva el monitoreo de la entrada 3
char poll_input4[]="*rd_input4#"; // comando para solicitud del estado de la entrada 4
char activate_moto_wheel[]="*moto_wheel_holland,1#";
char deactivate_moto_wheel[]="*moto_wheel_holland,0#";
char activate_moto_wheel_answ[]="*type_holland,1#";
char deactivate_moto_wheel_answ[]="*type_holland,0#";
char run_test_cmd[]="*run_test#";
char activate_ign_transition[]="*ignition_cutoff,1#"; // comnando para activar el envio de voltaje de bloqueo por motor apagado
char deactivate_ign_transition[]="*ignition_cutoff,0#"; // comando para desactivar el envio de voltaje de bloquero por motor apagado
char set_v3_time[]="*set_v3_time="; // longitud 12, comando para modificar el retardo en enviar el voltaje de bloqueo despues de motor apagado
char qry_ios[]="*qry_ios#"; // comando para solicitar estado de las entradas y salidas
char disable_all_security_cmd[]="*disable_all#"; // comando para enviar el voltaje V4 y deshabilitar la seguridad en el GPS
char enable_bpc2_routine_msg[]="*monitor_bpc2,1#";
char disable_bpc2_routine_msg[]="*monitor_bpc2,0#";
char enable_bpp_routine_msg[]="*monitor_bpp,1#"; // Comando para habilitar el Boquepo por puenteo (que hayan roto el pin de puerta)
char disable_bpp_routine_msg[]="*monitor_bpp,0#"; // Comando para deshabilitar el Boquepo por puenteo (que hayan roto el pin de puerta)
char enable_bpc1_routine_msg[]="*monitor_bpc1,1#";  // Activar bloqueo por corte de puerta en piloto
char disable_bpc1_routine_msg[]="*monitor_bpc1,0#"; // Desactivar bloqueo por corte de puerta en piloto
char get_bin_counters[]="*get_rtc#";
char enable_exe_bpc1[]="*exe_bpc1,1#"; // Comando para habilitar el voltaje de bloqueo por bpc1 
char disable_exe_bpc1[]="*exe_bpc1,0#"; // Comando para deshabilitar el voltaje de bloqueo por bpc1 
char enable_exe_bpc2[]="*exe_bpc2,1#"; // Comando para habilitar el voltaje de bloqueo por bpc12
char disable_exe_bpc2[]="*exe_bpc2,0#"; // Comando para deshabilitar el voltaje de bloqueo por bpc2
char enable_exe_bpp[]="*exe_bpp,1#"; // Comando para habilitar el voltaje de bloqueo por bpp
char disable_exe_bpp[]="*exe_bpp,0#"; // Comando para deshabilitar el voltaje de bloqueo por bpp
char edit_tmr_bpc1[]="*tmr_bpc1,"; // comando para cambiar los tiempos relacionados al Bloqueo Por Corte en la puerta del piloto
char edit_tmr_bpc2[]="*tmr_bpc2,"; // comando para cambiar los tiempos relacionados al Bloqueo Por Corte en la puerta del copiloto
char edit_tmr_pbb[]="*tmr_bpp,"; // comando para cambiar el tiempo relacionado al bloqueo por Puenteo de Puerta del piloto
char enable_exe_monitor1[]="*exe_monitor1,1#"; // Comando para habilitar el voltaje de bloqueo por apertura de puerta en piloto
char disable_exe_monitor1[]="*exe_monitor1,0#"; // Comando para deshabilitar el voltaje de bloqueo por apertura de puerta en piloto
char enable_exe_monitor3[]="*exe_monitor3,1#"; // Comando para habilitar el voltaje de bloqueo por apertura de puerta en piloto
char disable_exe_monitor3[]="*exe_monitor3,0#"; // Comando para deshabilitar el voltaje de bloqueo por apertura de puerta en piloto
char enable_out0_option[]="*auto_out0,1#"; // Comando para habilitar la funcion del segundo bloqueo del shield x3
char disable_out0_option[]="*auto_out0,0#"; // Comando para deshabilitar la funcion del segundo bloqueo del shield x3
char out0_on[]="*set_out0,1#"; // Comando para forzar a activar la salida para el segundo bloqueo del shield x2
char out0_off[]="*set_out0,0#"; // Comando para forzar a desactivar la salida para el segundo bloqueo del shield x2
char out0_cmd_mode_off[]="*set_out0,default#"; // Sale del modo comando cuando se esta forzando un estado de la salida del paro secundario
char temporal_out0_off[]="*set_out0,pause#"; // desactiva o inhabilita momentaneamente el paro secundario del pinlock. Se habilita nuevamente si detecta la entrada #2 del pinlock desactivada
char qry_master_pass[]="*qrymtrpss#"; // Comando para solicitar la contrase�a maestra
char set_serial[]="*set_serial:"; // Comando para definir el numero de serie en los pinlock
char qry_display_number[]="*qdn#";
char block_by_bad_pass_used_enable[]="*wpass_cutoff,1#";
char block_by_bad_pass_used_disable[]="*wpass_cutoff,0#";
char read_options0[]="*rm0#";
char remove_accesory[]="*remove_acc#";
char serial_number[15];
char acc_string[5];

const int input0_debouce = 10;
const int input1_debouce = 2; // Limite del contador para el retardo por rebote mecanico

short input0_state; // Estado de la entrada digital 0
short last_input_state; // Estado anterior al actual de la entrada 0
short poll_input0=false; // Indica si se solicito un polea de la entrada 0
short input_inverted; // Bandera que indica si la entrada 0 se considerara como invertida
short release_button=true; // Bit que permite volver a detectar un boton del teclao presionado
short test_ok_flag=false; // Confirma que la contraseÃ±a en la pantalla se usara
short password_1_used; // Bandera de contraseÃ±a 1 usada
short password_2_used; // Bandera de contraseÃ±a 2 usada
short password_3_used; // Bandera de contraseÃ±a 3 usada
short password_4_used; // Bandera de contraseÃ±a 4 usada
short password_5_used; // Bandera de contraseÃ±a 1 usada
short password_6_used; // Bandera de contraseÃ±a 2 usada
short password_7_used; // Bandera de contraseÃ±a 3 usada
short password_8_used; // Bandera de contraseÃ±a 4 usada
short password_master_used; // Bandera de contraseÃ±a maestra usada
short data_buffer=false; // Bandera que indica si hay datos en el buffer
short data_buffer2=false; // Bandera que indica si hay datos en el buffer
short response_lock_flag = true;
short send_lock_msg = false;
short lock_msg_running_flag = false;
short input3_enable; // bandera que permite o no el leer la entrada
short input3_state; // estado actual de la entrada 3
short last_input3_state; // estado anterior de la entrada 3
short poll_input3 = false; // Indica si se solicito un polea de la entrada 3
short input1_state; // Estado actual de la entrada
short last_input1_state; // Ultimo estado de la entrada, se compara con la actual para verificar si hubo un cambio
short poll_input1_flag = false; // Si es verdadera respondera al poleo de la entrada. Se activa desde comando serial
short test_copilot_door_flag; // activa o desactiva la funcion especial de la entrada
short out_0_activated;
short type_holland;
short input4_state; // estado actual de la entrada 4
short last_input4_state; // estado anterior de la entrada 4
short poll_input4_flag = false; // Indica si se solicito un poleo de la entrada 4
short input4_adc_enable; // Esta variable habilita o desabilita la funcion para enviar voltaje analogico por bloqueo
short enable_v3; // bandera para iniciar el contador que activarÃ¡ el voltaje V3 de bloqueo por apagado de motor
short disable_all_security_flag; // Bandera que habilita o no la ejecuciion de la funcion para enviar el voltaje V4
short bpc2_overtime = false;
short bpc2_overtime_option;
short bpc1_overtime = false; // 
short bpc1_overtime_option;
short bpp_flag_option; // bandera que controla el acceso a la rutina de bloqueo por puenteo
short bpp_flag = false; // bandera que apunta a la funcion de puebteo por bloqueo
short exe_bpc1_flag; // bandera para enviar o no el voltaje de bloqueo desde el pinlock hacia el syrus en la funcion bpc1
short exe_bpc2_flag; // bandera para enviar o no el voltaje de bloqueo desde el pinlock hacia el syrus en la funcion bpc2
short exe_bpp_flag; // bandera para enviar o no el voltaje de bloqueo desde el pinlock hacia el syrus en la funcion bpp
short exe_monitor1_flag; // bandera para enviar o no el voltaje de bloqueo desde el pinlock hacia el syrus en la funcion de apertura de puerta de piloto
short exe_monitor3_flag; // bandera para enviar o no el voltaje de bloqueo desde el pinlock hacia el syrus en la funcion de apertura de puerta de copiloto
short out0_option; // Bandera para activiar o no la salida 0 que depende de la entrada 2. Activa el paro secundario a ignicion
short out0_cmd_mode_flag;  // Bandera que indica si esta o no activado el modo comando para el bloqueo secundario del shield x3
short out0_cmd_mode_last_stat_flag;  // Bandera que indica en que estado se cargara la salida 0 en cado de estar activado el modo comando de la salida 0
short temporal_out0_off_flag;// = false;
short temporal_out0_off_transition;// = true;
short set_serial_flag; // bandera que indica que ya esta grabado un numero serial en el pinlock
short block_by_bad_pass_used_flag;
short acc_detected;
short acc_connected = false;

int input1_debouce_counter; // Contador para el retardo por rebote mecanico
int temp_doors_var;
int index2=0; // indice que indica en que nivel de la pila se esta guardando el dato en el puerto serial
int index=0; // indice que indica en que nivel de la pila se esta guardando el dato en el puerto serial
int last_input_state_backup;
int gpvar; // variable de uso general
int input0_debouce_counter;
int used_password_id_pinlock=112; //  ID de contraseÃ±a usada
int used_password_id_cargolock=212; //  ID de contraseÃ±a usada
int digit_position=3; // posicion del punto decimal
int i_timer1; // Timer del retardo para no detectar dobles botones presionados (debounce)
int tries; // Esta variable incrementa cada vez que ingresan una contraseÃ±a de manera incorrecta
//int attempts_allowed=3; // numero de intentos permitidos antes de bloquear el teclado por ingresar erroneamente una contraseÃ±a pantalla 0
int status_word; // 
int input4_debouce = 2; // retardo en segundos para volver a detectar la entrada
int input4_debouce_counter; // incrementar en timer0, para que no se dispare constantemente
int input3_debouce_counter; // incrementar en timer0, para que no se dispare constantemente
int input3_debouce = 2; // retardo en segundos para volver a detectar la entrada
int acc_string_index;
int acc_lost_counter;
int acc_lost_max_counter = 10;

int16 long_gpvar; // variable de uso general
int16 aux_counter; // variable para uso general
int16 general_reset_timer; // Contador para el reset periodico del programa
int16 general_reset = 7200; // Contador general para el reset del programa
int16 block_time_0 =300; // tiempo por el cual permanecera bloqueada la pantalla cuando se ingresan las contraseÃ±as erroneamente varias veces
int16 block_time_1 =600; // tiempo por el cual permanecera bloqueada la pantalla cuando se ingresan las contraseÃ±as erroneamente varias veces
int16 block_time_2 =900; // tiempo por el cual permanecera bloqueada la pantalla cuando se ingresan las contraseÃ±as erroneamente varias veces
int16 display_off_timer; // Timer para apagar el display cuando no se esta usando
int16 display_off_time = 15; // Tiempo para apagar el display cuando no se eesta usando
int16 display_number; // Numero que sera desplegado en la pantalla
int16 u,d,c,m;
int16 block_timer=0; // Variable que lleva el conteo que indica cuanto tiempo estara bloqueado el teclado
int16 reset_tries_timer=0; // Variable que lleva el conteo de cuanto tiempo falta para el los intentos fallidos en contraseÃ±a no bloqueen el teclado
int16 reset_tries=10800; // Tiempo para que se reinicie el contador de intentos fallidos en la contraseÃ±a
int16 password_1; // ContraseÃ±a 1
int16 password_2; // ContraseÃ±a 2
int16 password_3; // ContraseÃ±a 3
int16 password_4; // ContraseÃ±a 4
int16 password_5; // ContraseÃ±a 1
int16 password_6; // ContraseÃ±a 2
int16 password_7; // ContraseÃ±a 3
int16 password_8; // ContraseÃ±a 4  
int16 password_master; // Contraseña maestra
int16 keep_alive_accesory_counter;
int16 lock_msg_counter = 0;
int16 lock_max_msg_counter = 6;
int16 response_lock_timer;
int16 response_lock_time = 20;
int16 out_0_time; // retardo para inciar la activacion de la salida 0
int16 out_0_timer; // contador para el retardo de la activacion de la salida 0 
int16 debounce_copilot_door;
int16 umbral_debounce_copilot_door;
int16 an_out_time;
int16 v3_timer; // Almacena el tiempo incremental de segundos para enviar el voltaje de bloqueo despues de apagar el motor
int16 v3_time;// variable que almacena el retardo en segundos entre el apagado de motor y el envio del voltaje de bloqueo
int16 bpc2_timer; // Contador incremental
int16 bpc2_overtime_timer;
int16 bpc1_timer; // Contador incremental
int16 bpc1_overtime_timer;
int16 bpc1_overtime_time; // Intervalo de alertas de manipulacion de puerta de piloto por corte de sensor
int16 bpc2_overtime_time; // Intervalo de alertas de manipulacion de puerta de copiloto por corte de sensor
int16 bpc1_time; // Tiempo maximo para que la puerta permanezca abierta antes de considerar una posible manipulacion
int16 bpc2_time; // Tiempo maximo para que la puerta permanezca abierta antes de considerar una posible manipulacion
int16 bpp_time; // Tiempo entre cambio de estado de puerta y encendido de motor maximo para considerar un bloqueo por puenteo
int16 waiting_ok_timer;
int16 waiting_ok_time = 10;
int16 acc_connected_timer = 0;
int16 acc_connected_time = 60;
int16 acc_lost_max_mgs_counter;
int16 acc_lost_max_mgs_count = 10;
int16 acc_lost_long_msg_time = 21600;
int16 acc_lost_long_msg_timer;
int16 wrong_pass[6];

int32 rtc_engine_off; // Contador binario de cuanto tiempo a transcurrido desde que se apago el motor
int32 rtc_door; // Contador de respaldo de cuanto tiempo ha pasado desde que se apago el motor
int32 rtc_door_backup; // Contador binario cuanto tiempo ha pasado desde que se apago el motor hasta que se abrio/cerro una puerta
int32 options0;
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////        INTERRUPCIONES DEL PRGRAMA           ///////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

#INT_TIMER0
void  RTCC_isr(void) 
{
   restart_wdt(); 
   general_reset_timer++;    
   reset_tries_timer++;
   block_timer--;
   display_off_timer++;
   input0_debouce_counter++;
   input1_debouce_counter++;
   input3_debouce_counter++;
   input4_debouce_counter++;
   out_0_timer++;
   bpc2_timer++;
   bpc2_overtime_timer++;
   v3_timer++; // timer incremental del envio del voltaje de bloqueo por apagado de motor
   response_lock_timer++;
   rtc_engine_off++;
   rtc_door++;
   bpc1_timer++;
   bpc1_overtime_timer++;
   acc_connected_timer++;
   //acc_lost_max_mgs_counter++;
   acc_lost_long_msg_timer++;
   set_timer0(3036);
}

#INT_TIMER1
void  TIMER1_isr(void) 
{ 
   restart_wdt(); 
   i_timer1++;
   if(i_timer1>=5)
   {
      release_button=true;
      disable_interrupts(INT_TIMER1);
   }
   //output_toggle(led);
}

#INT_TIMER2 
void  TIMER2_isr(void) 
{
   //output_toggle(led);
   restart_wdt(); 
   debounce_copilot_door++;
   make_digits();
   display_7seg(display_number,digit_position);
}

/*
#INT_TIMER4
void  TIMER4_isr(void) 
{
   output_toggle(buzzer);
}*/

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

#int_rda4 // Controlador de motor
void serial_isr4() 
{
   char d;

   restart_wdt(); 
   if(kbhit(accesory))
   {
      d=fgetc(accesory);
         {
            send_to_buffer_accesory(d);
         }
   }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////        FUNCIONES DEL PRGRAMA           ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void system_reset_tries()
{
   restart_wdt(); 
   if(reset_tries_timer>=reset_tries)
   {
     tries=0;
     //update_eeprom_flag = true;
     write_ext_eeprom(tries_address, tries);
     reset_tries_timer=0;
   }
}


void system_reset()
{
   restart_wdt(); 
   if(general_reset_timer >= general_reset)
   {
      //fprintf(syrus,"%s", keep_alive_syrus);
      write_int16_ext_eeprom(acc_lost_long_msg_timer_address, acc_lost_long_msg_timer);
      fprintf(accesory,"%s",rst_micro);  
      delay_ms(1000);
      reset_cpu();
   } 
}

void load_status_word() // Carga el historial de las contraseÃ±as que se han usado
{
   int temp_pass_var;

    restart_wdt(); 
    input_inverted =  read_ext_eeprom(input_inverted_address);     
    status_word = read_ext_eeprom(status_word_address);  
    password_1_used =  bit_test(status_word,0);
    password_2_used =  bit_test(status_word,1);
    password_3_used =  bit_test(status_word,2);
    password_4_used =  bit_test(status_word,3);
    password_5_used =  bit_test(status_word,4);
    password_6_used =  bit_test(status_word,5);
    password_7_used =  bit_test(status_word,6);
    password_8_used =  bit_test(status_word,7);
    
    temp_pass_var = read_ext_eeprom(password_master_used_address);  
    if(temp_pass_var=='s')
    {
       password_master_used = true;
    }
    else
    {
       password_master_used = false;
    }

    password_1 = read_int16_ext_eeprom(password_1_address);
    password_2 = read_int16_ext_eeprom(password_2_address);
    password_3 = read_int16_ext_eeprom(password_3_address);
    password_4 = read_int16_ext_eeprom(password_4_address); 
    password_5 = read_int16_ext_eeprom(password_5_address);
    password_6 = read_int16_ext_eeprom(password_6_address);
    password_7 = read_int16_ext_eeprom(password_7_address);
    password_8 = read_int16_ext_eeprom(password_8_address);
    password_master = read_int16_ext_eeprom(password_master_address);
}

void test_input()
{
   restart_wdt(); 
   if(input0_debouce_counter>=input0_debouce)
   {
      /*
         Variables y definiciones requeridas:
         #define ign PIN_??*/
     
      if(input(in_0)) // Entrada  activada
      {
         input0_state = true;
      }
      else
      {
         input0_state = false;
      }
      
      // Si hubo un cambio de estado en la entrada
      if((poll_input0==1)||(input0_state!=last_input_state)) 
      {
         //fprintf(syrus,"cambio entrada, %u,%u,%u\r\n", input0_state, last_input_state, poll_input0);
         if(input0_state)
         {
            last_input_state = true;
            write_ext_eeprom(in_0_address, 0xFF);
         }
         else
         {
            last_input_state = false;
            write_ext_eeprom(in_0_address, 0x00);
         }
   
         if(input0_state) // Entrada  activada
         {
            if(!input_inverted) // Entrada no invertida
            {
               if(poll_input0)
               {
                  fprintf(syrus,"%s", msg_poll_enganche);
               }
               else
               {
                  fprintf(syrus,"%s", msg_engache);
               }
            }
           else // Entrada invertida
            {
               if(poll_input0)
               {
                  fprintf(syrus,"%s", msg_poll_desenganche);
               }
               else
               {
                  fprintf(syrus,"%s", msg_desengache);
               }
            }   
         }
         else  // entrada no activada
         {
            if(!input_inverted) // Entrada no invertida
            {
               if(poll_input0)
               {
                  fprintf(syrus,"%s", msg_poll_desenganche);
               }
               else
               {
                  fprintf(syrus,"%s", msg_desengache);
               }
            }
           else // Entrada invertida
            {
               if(poll_input0)
               {
                  fprintf(syrus,"%s",msg_poll_enganche);
               }
               else
               {
                  fprintf(syrus,"%s",msg_engache);
               }
            } 
         }
         poll_input0 = false;      
      }      
      input0_debouce_counter = 0;
   }
}   

void scan_cmd()
{
  short valid_cmd = false;
  short valid_pass_cmd = false;
  short valid_cmd_aux = false;
  short valid_data=true;
  int length=0;
  int i=0;
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

      for(aux_counter = 0; aux_counter < 150; ++aux_counter)
      {
         accesory_buffer[aux_counter] = 0;
      }
      index2 = 0;
   
      //fprintf(syrus,"+%s+\r\n", syrus_buffer);
      //fprintf(syrus,"+%u+\r\n", length);
      //rst_cpu
      if((syrus_buffer[0]=='*')&&(syrus_buffer[length-1]=='#'))
      {
      
        valid_cmd = false;
        valid_pass_cmd = false;
        valid_cmd_aux = false;
         
        if(!strncmp(rst_micro, syrus_buffer, length))
        {
          fprintf(accesory,"%s",rst_micro);  
          for(aux_counter = 0; aux_counter < 16; ++aux_counter)
          {
            delay_ms(250);
            output_toggle(led);
          }
          fprintf(accesory,"%s",rst_micro);  
          delay_ms(250);
          write_int16_ext_eeprom(acc_lost_long_msg_timer_address, acc_lost_long_msg_timer);
          reset_cpu();
        }
        else if(!strncmp(run_test_cmd, syrus_buffer, length)) // Comando para correr pruebas en libreria test.h
        {
            int16 max_address = 256;
            int16 count_address;
            int16 map_data;
            int32 CRC;
   
            count_address = 0;
            do{
               write_ext_eeprom(count_address,0xFF);
               fprintf(syrus, "%03Lu\r", count_address);
               count_address++;
            }while(count_address<=max_address);
                 valid_cmd = true;
        }
        else if((syrus_buffer[1]=='d')&&(syrus_buffer[2]=='s')&&(syrus_buffer[3]=='d')&&(syrus_buffer[4]=='=')) // Comando: *dsd=XXXXX#, retardo para apertura de puertas
        {
          i=0;
          do{
            char_password[i] = syrus_buffer[i+5];
            i++;
          }while(syrus_buffer[i+5]!='#');

          if(i<5)
          {
            char_password[i] = 0x00;
            umbral_debounce_copilot_door = atol(char_password);
            write_int16_ext_eeprom(umbral_debounce_copilot_door_address, umbral_debounce_copilot_door);
            fprintf(syrus,"*dsd,%Lu#", umbral_debounce_copilot_door);
            valid_cmd = true;   
          }
          else
          {
            valid_cmd = false;
          }
        }
        else if((syrus_buffer[1]=='i')&&(syrus_buffer[2]=='n')&&(syrus_buffer[3]=='f')&&(syrus_buffer[4]=='o')) // Comando: *info#
        {
           valid_cmd = true;
        }
        else if(!strncmp(read_status, syrus_buffer, length))
        {
          fprintf(syrus,"*STS_WRD:%X#\r\n", status_word);
          valid_cmd = true;
        }
        else if(!strncmp (invert_msg, syrus_buffer, length)) // inveirte la deteccion de la entrada 0 del pinlock, dedicada a la quinta rueda
        {
          input_inverted = true;
          write_ext_eeprom(input_inverted_address,0xFF);
          fprintf(syrus,"%s", invert_msg_answer);
          valid_cmd = true;
        }
        else if(!strncmp (no_invert_msg, syrus_buffer, length))
        {
          input_inverted = false;
          write_ext_eeprom(input_inverted_address,0x00);
          fprintf(syrus,"%s", no_invert_msg_answer);
          valid_cmd = true;
        }
        else if(!strncmp(poll_gnd_input, syrus_buffer, length))
        {
          poll_input0 = true;
          valid_cmd = true;
        }
        else if(!strncmp(rst_blocktimer, syrus_buffer, length))
        {
          block_timer = 2;
          valid_cmd = true;
        }
        else if(!strncmp(pass_request, syrus_buffer, length))
        {
          fprintf(syrus,"*%LX,%LX,%LX,%LX,%LX,%LX,%LX,%LX#", ~password_1+1, ~password_2+1, ~password_3+1, ~password_4+1,~password_5+1, ~password_6+1, ~password_7+1, ~password_8+1);
          valid_cmd = true;
        }
//!        else if(!strncmp(save_eeprom, syrus_buffer, length))
//!        {
//!          fprintf(syrus,"\r\nsaved!!!\r\n");
//!          valid_cmd = true;
//!        }
        else if(!strncmp(blocking_box, syrus_buffer, length))
        {
          lock_cargolock();   
          valid_cmd = true;   
        }
        else if(!strncmp(unblocking_box, syrus_buffer, length))
        {
          fprintf(accesory,"%s", unblocking_box);
          valid_cmd = true;
        }
        else if(!strncmp(autolock_enable, syrus_buffer, length))
        {
          fprintf(accesory,"%s", autolock_enable);
          valid_cmd = true;
        }
        else if(!strncmp(autolock_disable, syrus_buffer, length))
        {
          fprintf(accesory,"%s", autolock_disable);
          valid_cmd = true;
        }  
        else if(!strncmp(activate_security_doors, syrus_buffer, length))
        {
          fprintf(syrus,"%s", activate_security_doors_resp);
          test_copilot_door_flag = true;
          write_ext_eeprom(test_copilot_door_flag_address, 's');      
          valid_cmd = true;
        }
        else if(!strncmp(deactivate_security_doors, syrus_buffer, length))
        {
          fprintf(syrus,"%s", deactivate_security_doors_resp);
          test_copilot_door_flag = false; // Se desactiva la lectura de la puerta del piloto
          write_ext_eeprom(test_copilot_door_flag_address, 'n');    
          input3_enable = false; // Tambien se desactiva la lectura de la puerta del piloto por seguridad
          write_ext_eeprom(input3_enable_address, 'n');  
          valid_cmd = true;
        }
        else if(!strncmp(inc_analog_time, syrus_buffer, length))
        {
          an_out_time+=1000;
          write_int16_ext_eeprom(an_out_time_address, an_out_time);
          fprintf(syrus,"*%Lu#", an_out_time/1000);
          valid_cmd = true;
        }
        else if(!strncmp(dec_analog_time, syrus_buffer, length))
        {
          an_out_time-=1000;
          write_int16_ext_eeprom(an_out_time_address, an_out_time);
          fprintf(syrus,"*%Lu#", an_out_time/1000);
          valid_cmd = true;
        }
        else if(!strncmp(out_0_dly, syrus_buffer, 10)) // Retardo para activacion de la salida 2 despues de activar la entrada 2
        {
          i=0;
          do{
            char_password[i] = syrus_buffer[i+10];
            i++;
          }while(syrus_buffer[i+10]!='#');
          if(i<6)
          {
            char_password[i] = 0x00;
            out_0_time = atol(char_password);
            write_int16_ext_eeprom(out_0_time_address, out_0_time);
            fprintf(syrus,"*dly_out_0,%Lu seg#", out_0_time);
            out_0_timer=0;
            valid_cmd = true;   
          }
        }
        else if(!strncmp(poll_input1, syrus_buffer, length))
        {
          poll_input1_flag = true;
          valid_cmd = true;
        }
        else if(!strncmp(poll_input_3, syrus_buffer, length))
        {
          poll_input3 = true;
          valid_cmd = true;
        }
        else if(!strncmp(poll_input4, syrus_buffer, length))
        {
          poll_input4_flag = true;
          valid_cmd = true;
        }
        else if(!strncmp(monitor_in3_enable, syrus_buffer, length))
        {
          fprintf(syrus,"%s", monitor_in3_enable_resp);
          input3_enable = true;
          write_ext_eeprom(input3_enable_address, 's');      
          valid_cmd = true;
        }
        else if(!strncmp(monitor_in3_disable, syrus_buffer, length))
        {
          fprintf(syrus,"%s", monitor_in3_disable_resp);
          input3_enable = false;
          write_ext_eeprom(input3_enable_address, 'n');      
          valid_cmd = true;
        }       
        else if(!strncmp(activate_moto_wheel, syrus_buffer, length))
        {
          input_inverted = true;
          type_holland = true,
          write_ext_eeprom(input_inverted_address,0xFF);
          write_ext_eeprom(type_holland_address,'s');
          fprintf(syrus, "%s", activate_moto_wheel_answ);
          valid_cmd = true;   
        }
        else if(!strncmp(deactivate_moto_wheel, syrus_buffer, length))
        {
          type_holland = false,
          write_ext_eeprom(type_holland_address,'n');
          fprintf(syrus, "%s", deactivate_moto_wheel_answ);
          valid_cmd = true;   
        }
        else if(!strncmp(activate_ign_transition, syrus_buffer, length)) // Habilita envio de voltaje para bloquear por apagado de motor
        {
          input4_adc_enable = true;
          write_ext_eeprom(v3_enable_address, 's');
          fprintf(syrus,"%s", ans_2_activate_ign_transition);
          valid_cmd = true;
        }
        else if(!strncmp(deactivate_ign_transition, syrus_buffer, length)) // Deshabilitar envio de voltaje para bloqueo por apagado de motor
        {
          input4_adc_enable = false;
          write_ext_eeprom(v3_enable_address, 'n');
          fprintf(syrus,"%s", ans_2_deactivate_ign_transition);
          valid_cmd = true;
        }
        else if(!strncmp(set_v3_time, syrus_buffer, 12)) // cambiar el tiempo del retardo para el envio de voltaje para bloqueo por apagado de motor
        {
          i=0;
          do{
            char_password[i] = syrus_buffer[i+13];
            i++;
          }while(syrus_buffer[i+13]!='#');
          if(i<5)
          {
            char_password[i] = 0x00;
            v3_time = atol(char_password);
            v3_timer=0;
            write_int16_ext_eeprom(v3_time_address, v3_time);
            fprintf(syrus,"*v3=%Lu#",v3_time);
            valid_cmd = true;   
          }
        }  
        else if(!strncmp(qry_ios, syrus_buffer, length)) // Solicita el estado actual de las entradas y salidas
        {
          i = 0;
          if(input_state(in_0))
          {i+=1;}
          if(input_state(in_1))
          {i+=2;}
          if(input_state(in_2))
          {i+=4;}
          if(input_state(in_3))
          {i+=8;}
          if(input_state(in_4))
          {i+=16;}
          if(input_state(out_0))
          {i+=32;}
          if(input_state(out_1))
          {i+=64;}
          fprintf(syrus,"%s%u#", ios_state_resp,i);
          valid_cmd = true;
        }
        else if(!strncmp(disable_all_security_cmd, syrus_buffer, length)) // Habilita envio de voltaje para deshabilitar el sistema de seguridad
        {
          fprintf(syrus,"%s", cmd_disable_all_security_cmd);
          disable_all_security_flag = true;
          valid_cmd = true;
        }
        else if(!strncmp(enable_bpc2_routine_msg, syrus_buffer, length))
        {
          fprintf(syrus,"%s", enable_bpc2_routine_msg_resp); // 
          bpc2_overtime_option = true;
          write_ext_eeprom(bpc2_address,'s');
          valid_cmd = true;
        }
        else if(!strncmp(disable_bpc2_routine_msg, syrus_buffer, length))
        {
          fprintf(syrus,"%s", disable_bpc2_routine_msg_resp); // 
          bpc2_overtime_option = false;
          write_ext_eeprom(bpc2_address,'n');
          valid_cmd = true;
        }
        else if(!strncmp(enable_bpp_routine_msg, syrus_buffer, length))
        {
            fprintf(syrus,"%s", enable_bpp_routine_msg_resp); // 
            bpp_flag_option = true; // la bandera se coloca arriba para permitir el acceso a esa rutina
            write_ext_eeprom(bpp_flag_option_address,'s'); // se graba en la eeprom para recordar la funcion al reiniciar el sistema
            valid_cmd = true; // Comando valido para no enviar mensjae de error
        }
        else if(!strncmp(disable_bpp_routine_msg, syrus_buffer, length))
        {
            fprintf(syrus,"%s", disable_bpp_routine_msg_resp); // respuesta de desactivacion del bloqueo por apertura de copiloto
            bpp_flag_option = false; // la bandera se coloca abajo para no permitir el acceso a esa rutina
            write_ext_eeprom(bpp_flag_option_address,'n');
            valid_cmd = true; // Comando valido para no enviar mensjae de error
        }
        else if(!strncmp(enable_bpc1_routine_msg, syrus_buffer, length))
        {
            fprintf(syrus,"%s", enable_bpc1_routine_msg_resp); // 
            bpc1_overtime_option = true;
            write_ext_eeprom(bpc1_address,'s');
            valid_cmd = true;
        }
        else if(!strncmp(disable_bpc1_routine_msg, syrus_buffer, length))
        {
            fprintf(syrus,"%s", disable_bpc1_routine_msg_resp); // respuesta de desactivacion del bloqueo por apertura de copiloto
            bpc1_overtime_option = false;
            write_ext_eeprom(bpc1_address,'n');
            valid_cmd = true;
        }
        else if(!strncmp(set_time_2_activate_autolock, syrus_buffer, 13)) // cambiar el tiempo del retardo para el autobloqueo 
        {  
          fprintf(accesory,"%s",syrus_buffer);
          valid_cmd = true;   
        } 
        else if(!strncmp(set_lock_time_1_cmd, syrus_buffer, 16)) // cambiar el tiempo del retardo para el bloqueo despues de detectar el sensor inductivo.
        {  
          fprintf(accesory,"%s",syrus_buffer);
          valid_cmd = true;   
        }
        else if(!strncmp(get_interlock_status_msg, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",get_interlock_status_msg);
          valid_cmd = true;   
        }
        else if(!strncmp(get_status, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",get_status);
          valid_cmd = true;   
        }
        else if(!strncmp(qry_version, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",qry_version);
          valid_cmd = true;   
        }
        else if(!strncmp(force_blocking_box, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",force_blocking_box);
          valid_cmd = true;   
        }
        else if(!strncmp(set_offset_0, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",set_offset_0);
          valid_cmd = true;   
        }
        else if(!strncmp(set_offset_100, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",set_offset_100);
          valid_cmd = true;   
        }
        else if(!strncmp(inc_mov, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",inc_mov);
          valid_cmd = true;   
        }
        else if(!strncmp(dec_mov, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",dec_mov);
          valid_cmd = true;   
        }
        else if(!strncmp(set_max_stroke_move, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",set_max_stroke_move);
          valid_cmd = true;   
        }
        else if(!strncmp(remmove_max_stroke_move, syrus_buffer, length)) // Solicita el estado del enganche/desenganche y estado del bloqueo/desbloqueo
        {  
          fprintf(accesory,"%s",remmove_max_stroke_move);
          valid_cmd = true;   
        }
        else if(!strncmp(read_acc0_options0, syrus_buffer, length)) 
        {  
          fprintf(accesory,"%s",read_acc0_options0);
          valid_cmd = true;   
        }
        else if(!strncmp(get_bin_counters, syrus_buffer, length)) // Solicita el valor de los contadores binarios del tiempo desde que se apago el motor o desde que se inicio el programa
        {  
          fprintf(syrus,"Motor: %Lu - Puerta: %Lu - Respaldo: %Lu\r\n",rtc_engine_off, rtc_door, rtc_door_backup);
          valid_cmd = true;   
        }
        
        else if(!strncmp(enable_exe_bpc1, syrus_buffer, length))
        {
           exe_bpc1_flag = true;
           write_ext_eeprom(exe_bpc1_flag_address, 's');
           fprintf(syrus,"%s",enable_exe_bpc1_resp);
           valid_cmd = true;
        }
        else if(!strncmp(disable_exe_bpc1, syrus_buffer, length))
        {
           exe_bpc1_flag = false;
           write_ext_eeprom(exe_bpc1_flag_address, 'n');
           fprintf(syrus,"%s",disable_exe_bpc1_resp);
           valid_cmd = true;
        }
        else if(!strncmp(enable_exe_bpc2, syrus_buffer, length))
        {
           exe_bpc2_flag = true;
           write_ext_eeprom(exe_bpc2_flag_address, 's');
           fprintf(syrus,"%s",enable_exe_bpc2_resp);
           valid_cmd = true;
        }
        else if(!strncmp(disable_exe_bpc2, syrus_buffer, length))
        {
           exe_bpc2_flag = false;
           write_ext_eeprom(exe_bpc2_flag_address, 'n');
           fprintf(syrus,"%s",disable_exe_bpc2_resp);
           valid_cmd = true;
        }
        else if(!strncmp(enable_exe_bpp, syrus_buffer, length))
        {
           exe_bpp_flag = true;
           write_ext_eeprom(exe_bpp_flag_address, 's');
           fprintf(syrus,"%s",enable_exe_bpp_resp);
           valid_cmd = true;
        }
        else if(!strncmp(disable_exe_bpp, syrus_buffer, length))
        {
           exe_bpp_flag = false;
           write_ext_eeprom(exe_bpp_flag_address, 'n');
           fprintf(syrus,"%s",disable_exe_bpp_resp);
           valid_cmd = true;
        }
        else if(!strncmp(edit_tmr_bpc1, syrus_buffer, 9)) // cambiar los tiempos relacionados a bpc1
        {
            //fprintf(syrus,"puntero 0\r\n");
            //fprintf(syrus,"%s\r\n",syrus_buffer);
           // Se busca si en la cadena existe una coma que delimite el valor de los dos contadores que se pretende grabar  
           i=0;
           do{
             char_password[i] = syrus_buffer[i+10];
             i++;
           }while((syrus_buffer[i+10]!='#')&&(syrus_buffer[i+10]!=','));
            //fprintf(syrus,"puntero 1\r\n");
             // Si existe una coma entre los valores entonces se evaluan los datos para verificar si son correctos
           if((syrus_buffer[i+10]==','))
           {
             for (j = 0; j <= 6; ++j)
             {
                char_password[j] = 0;
             }  
             j=0;  
             i++;
             do{
               char_password[j] = syrus_buffer[i+10];
               i++;
               j++;
             }while(syrus_buffer[i+10]!='#');
             //fprintf(syrus,"puntero 2\r\n");
             if(j<=5)
             {
               char_password[j] = 0x00;
               bpc1_overtime_time = atol(char_password);
               //fprintf(syrus,"%Lu\r\n", bpc1_overtime_time);
               write_int16_ext_eeprom(bpc1_overtime_time_address, bpc1_overtime_time); 
         
               i=0;
               do{
                 char_password[i] = syrus_buffer[i+10];
                 i++;
               }while(syrus_buffer[i+10]!=',');
               //fprintf(syrus,"puntero 3\r\n");
               if(i<=5)
               {
                 char_password[i] = 0x00;
                 bpc1_time = atol(char_password);
                 write_int16_ext_eeprom(bpc1_time_address, bpc1_time); 
                  //const char bpc1_tmrs_saved[]="*229,";
                  //const char bpc2_tmrs_saved[]="*230,";
                  //const char footer='#';
                 fprintf(syrus,"%s%Lu,%Lu%c", bpc1_tmrs_saved,bpc1_time,bpc1_overtime_time,footer);
                 //fprintf(syrus,"T1 = %Lu y T2 = %Lu\r\n", bpc1_time, bpc1_overtime_time);
                 valid_cmd = true;  
               }
             }
             //fprintf(syrus,"puntero 4\r\n");
           }
           //fprintf(syrus,"puntero 5\r\n");
        }
        else if(!strncmp(edit_tmr_pbb, syrus_buffer, 8)) // cambiar los tiempos relacionados a bpp
        {
           // Se busca si en la cadena existe una coma que delimite el valor de los dos contadores que se pretende grabar  
           i=0;
           do{
             char_password[i] = syrus_buffer[i+9];
             i++;
           }while(syrus_buffer[i+9]!='#');
           char_password[i] = 0x00;
           bpp_time = atol(char_password);
           write_int16_ext_eeprom(bpp_time_address, bpp_time); 
           fprintf(syrus,"%s%Lu%c", bpp_tmr_saved,bpp_time,footer);
           valid_cmd = true;
        }
        else if(!strncmp(edit_tmr_bpc2, syrus_buffer, 9)) // cambiar los tiempos relacionados a bpc2
        {
           // Se busca si en la cadena existe una coma que delimite el valor de los dos contadores que se pretende grabar  
           i=0;
           do{
             char_password[i] = syrus_buffer[i+10];
             i++;
           }while((syrus_buffer[i+10]!='#')&&(syrus_buffer[i+10]!=','));
             // Si existe una coma entre los valores entonces se evaluan los datos para verificar si son correctos
           if((syrus_buffer[i+10]==','))
           {
             for (j = 0; j <= 6; ++j)
             {
                char_password[j] = 0;
             }  
             j=0;  
             i++;
             do{
               char_password[j] = syrus_buffer[i+10];
               i++;
               j++;
             }while(syrus_buffer[i+10]!='#');
             if(j<=5)
             {
               char_password[j] = 0x00;
               bpc2_overtime_time = atol(char_password);
               write_int16_ext_eeprom(bpc2_overtime_time_address, bpc2_overtime_time); 
         
               i=0;
               do{
                 char_password[i] = syrus_buffer[i+10];
                 i++;
               }while(syrus_buffer[i+10]!=',');
               if(i<=5)
               {
                 char_password[i] = 0x00;
                 bpc2_time = atol(char_password);
                 write_int16_ext_eeprom(bpc2_time_address, bpc2_time); 
                 fprintf(syrus,"%s%Lu,%Lu%c", bpc2_tmrs_saved,bpc2_time,bpc2_overtime_time,footer);
                 //fprintf(syrus,"T1 = %Lu y T2 = %Lu\r\n", bpc2_time, bpc2_overtime_time);
                 valid_cmd = true;  
               }
             }
           }
        }
        else if(!strncmp(enable_exe_monitor1, syrus_buffer, length))
        {
           exe_monitor1_flag = true;
           write_ext_eeprom(exe_monitor1_flag_address, 's');
           fprintf(syrus,"%s",enable_exe_monitor1_resp);
           valid_cmd = true;
        }
        else if(!strncmp(disable_exe_monitor1, syrus_buffer, length))
        {
           exe_monitor1_flag = false;
           write_ext_eeprom(exe_monitor1_flag_address, 'n');
           fprintf(syrus,"%s",disable_exe_monitor1_resp);
           valid_cmd = true;
        }
        else if(!strncmp(enable_exe_monitor3, syrus_buffer, length))
        {
           exe_monitor3_flag = true;
           write_ext_eeprom(exe_monitor3_flag_address, 's');
           fprintf(syrus,"%s",enable_exe_monitor3_resp);
           valid_cmd = true;
        }
        else if(!strncmp(disable_exe_monitor3, syrus_buffer, length))
        {
           exe_monitor3_flag = false;
           write_ext_eeprom(exe_monitor3_flag_address, 'n');
           fprintf(syrus,"%s",disable_exe_monitor3_resp);
           valid_cmd = true;
        }
        else if(!strncmp(get_feedback_cmd, syrus_buffer, length))
        {
          fprintf(accesory,"%s", get_feedback_cmd);
          valid_cmd = true;
        }
        else if(!strncmp(define_model, syrus_buffer, 10)) // 
        {  
          fprintf(accesory,"%s",syrus_buffer);
          valid_cmd = true;   
        }
        else if(!strncmp(qry_model, syrus_buffer, length)) // 
        {  
          fprintf(accesory,"%s",syrus_buffer);
          valid_cmd = true;   
        }
        else if(!strncmp(qry_ios_acc, syrus_buffer, length)) // 
        {  
          fprintf(accesory,"%s",syrus_buffer);
          valid_cmd = true;   
        }
        else if(!strncmp(enable_out0_option, syrus_buffer, length)) // Activa la funcionalidad del bloqueo secundario en el shield x3
        {
           out0_option = true;
           write_ext_eeprom(out0_option_address, 's');
           fprintf(syrus,"%s",out0_option_activated);
           valid_cmd = true;
        }
        else if(!strncmp(disable_out0_option, syrus_buffer, length)) // desactiva la funcionalidad del bloqueo secundario en el shield x3
        {
           out0_option = false;
           write_ext_eeprom(out0_option_address, 'n');
           fprintf(syrus,"%s",out0_option_deactivated);
           valid_cmd = true;
        }
        else if(!strncmp(out0_on, syrus_buffer, length))
        {
           out0_cmd_mode_flag = true;
           write_ext_eeprom(out0_cmd_mode_address, 's');

           output_high(out_0);
           out0_cmd_mode_last_stat_flag = true;
           write_ext_eeprom(out0_cmd_mode_last_stat_address, 's');

           fprintf(syrus,"%s",msg_out0_on);
           valid_cmd = true;
        }
        else if(!strncmp(out0_off, syrus_buffer, length))
        {
           out0_cmd_mode_flag = true;
           write_ext_eeprom(out0_cmd_mode_address, 's');

           output_low(out_0);
           out0_cmd_mode_last_stat_flag = false;
           write_ext_eeprom(out0_cmd_mode_last_stat_address, 'n');

           fprintf(syrus,"%s",msg_out0_off);
           valid_cmd = true;
        }
        else if(!strncmp(out0_cmd_mode_off, syrus_buffer, length))
        {
           out0_cmd_mode_flag = false;
           write_ext_eeprom(out0_cmd_mode_address, 'n');
           fprintf(syrus,"%s",msg_out0_cmd_mode_off);
           valid_cmd = true;
        }
        else if(!strncmp(temporal_out0_off, syrus_buffer, length)) // desactiva o inhabilita momentaneamente el paro secundario del pinlock. Se habilita nuevamente si detecta la entrada #2 del pinlock desactivada
        {
            temporal_out0_off_flag = true; // activa la bandera para pausar el paro secundario que controla el pinlock
            write_ext_eeprom(temporal_out0_off_flag_address, 's');
            output_low(out_0);  
            out_0_activated = false;
            write_ext_eeprom(out_0_activated_address, 'n');
            fprintf(syrus,"%s", temporal_out0_off_resp); // respuesta de confirmacion de la desactivacion temporal del bloqueo secundario
            valid_cmd = true;
        }
        else if(!strncmp(qry_master_pass, syrus_buffer, length)) // Comando para preguntar la contrase�a maestra
        {
           fprintf(syrus,"%s%04Lu%c", qry_master_pass_resp, ~password_master+1, footer);    
           valid_cmd = true;     
        }
        else if(!strncmp(set_serial, syrus_buffer, 11))
        {
           //*set_serial:IL29071900987#
           // La longitud correcta debe ser 13
           i=0;
           for (i = 0; i < 13; ++i)
           {
              serial_number[i] = syrus_buffer[12+i]; 
              write_ext_eeprom(serial_address+i, serial_number[i]);
           }
           write_ext_eeprom(serial_address+i, 0x00);
           valid_data = true;
           i=0;
           while((valid_data)&&(i<13))
           {
               if((((serial_number[i]>='0')&&(serial_number[i]<='9'))||((serial_number[i]>='a')&&(serial_number[i]<='z'))))
               {
                  valid_data = true;
               }
               else
               {
                  valid_data = false;
               }
               i++;
           }
           
           /*
            valid_data nos indicara si el numero de serie solo esta dado por numeros y letras
           */
           if(valid_data)
           {
               set_serial_flag = true;
               write_ext_eeprom(serial_address_flag, 's');
               //fprintf(syrus,"%s%s,%Lu%c",set_serial_resp,serial_number,strlen(serial_number),footer);
               fprintf(syrus,"%s%s%c",set_serial_resp,serial_number,footer);
               valid_cmd = true;
           }
        }
        else if(!strncmp(block_by_bad_pass_used_enable, syrus_buffer, length))
        {
          block_by_bad_pass_used_flag = true;
          fprintf(syrus, "%s", block_by_bad_pass_used_enable_rep);
          write_ext_eeprom(block_by_bad_pass_used_address, 's');
          valid_cmd = true;
        }
        else if(!strncmp(block_by_bad_pass_used_disable, syrus_buffer, length))
        {
          block_by_bad_pass_used_flag = false;
          fprintf(syrus, "%s", block_by_bad_pass_used_disable_rep);
          write_ext_eeprom(block_by_bad_pass_used_address, 'n');
          valid_cmd = true;
        }
        else if(!strncmp(qry_display_number, syrus_buffer, length))
        {
           fprintf(syrus, "%s%04Lu%c", qry_display_number_resp,display_number,footer);
           valid_cmd = true;
        }
        else if(!strncmp(read_options0, syrus_buffer, length))
        {
           options0 = 0;
           
           if(input_inverted)
              {bit_set(options0,0);}
           else
              {bit_clear(options0,0);}

           if(test_copilot_door_flag)
              {bit_set(options0,1);}
           else
              {bit_clear(options0,1);}

           if(input3_enable)
              {bit_set(options0,2);}
           else
              {bit_clear(options0,2);}

           if(exe_monitor1_flag)
              {bit_set(options0,3);}
           else
              {bit_clear(options0,3);}

           if(exe_monitor3_flag)
              {bit_set(options0,4);}
           else
              {bit_clear(options0,4);}

           if(input4_adc_enable)
              {bit_set(options0,5);}
           else
              {bit_clear(options0,5);}

           if(bpc2_overtime_option)
              {bit_set(options0,6);}
           else
              {bit_clear(options0,6);}

           if(bpc1_overtime_option)
              {bit_set(options0,7);}
           else
              {bit_clear(options0,7);}

           if(bpp_flag_option)
              {bit_set(options0,8);}
           else
              {bit_clear(options0,8);}

           if(exe_bpc2_flag)
              {bit_set(options0,9);}
           else
              {bit_clear(options0,9);}

           if(exe_bpc1_flag)
              {bit_set(options0,10);}
           else
              {bit_clear(options0,10);}

           if(exe_bpp_flag)
              {bit_set(options0,11);}
           else
              {bit_clear(options0,11);}

           if(out0_option)
              {bit_set(options0,12);}
           else
              {bit_clear(options0,12);}

           if(out0_cmd_mode_flag)
              {bit_set(options0,13);}
           else
              {bit_clear(options0,13);}

           if(temporal_out0_off_flag)
              {bit_set(options0,14);}
           else
              {bit_clear(options0,14);}

           if(block_by_bad_pass_used_flag)
              {bit_set(options0,15);}
           else
              {bit_clear(options0,15);}
           fprintf(syrus, "%s%Lu%c", read_options0_resp,options0,footer);
           valid_cmd = true;
        }
        else if(!strncmp(remove_accesory, syrus_buffer, length))
        {
            acc_detected = false;
            write_ext_eeprom(acc_detected_address,'n');
            fprintf(syrus, "%s", remove_accesory_resp);
            valid_cmd = true;
        }
        else if(syrus_buffer[1]==0xFF)
        {
          valid_cmd_aux = true;
          valid_cmd = true;
          switch (syrus_buffer[2])  // Se selecciona que registro se va a escribir
          {
            case 0x00: // Se cargan las cuatro nuevas contraseÃ±as que seran utilizadas -  *\FF\001111222233334444#
            i=3;
            letter_index = 0;
            do{ // Se verifica primeramente que la contraseÃ±a solo contenga numeros
              if((syrus_buffer[i]>='q')&&(syrus_buffer[i]<='z'))
              {
                  letter_pass[letter_index] = syrus_buffer[i];
                  switch (syrus_buffer[i])
                  {
                     case 'z':
                     syrus_buffer[i] = '0';
                     break;   
         
                     case 'y':
                     syrus_buffer[i] = '1';
                     break;
         
                     case 'x':
                     syrus_buffer[i] = '2';
                     break;
         
                     case 'w':
                     syrus_buffer[i] = '3';
                     break;
         
                     case 'v':
                     syrus_buffer[i] = '4';
                     break;
         
                     case 'u':
                     syrus_buffer[i] = '5';
                     break;
         
                     case 't':
                     syrus_buffer[i] = '6';
                     break;
         
                     case 's':
                     syrus_buffer[i] = '7';
                     break;
         
                     case 'r':
                     syrus_buffer[i] = '8';
                     break;
         
                     case 'q':
                     syrus_buffer[i] = '9';
                     break;
                  }
                  valid_data=true;
              }
              else
              {
                  valid_data=false;     
              }
              i++;   
              letter_index++;
            }while((i!=19)&&(valid_data));
            letter_pass[letter_index] = 0x00;
            if(valid_data)
            {
              char_password[0]=syrus_buffer[3]; // millares   
              char_password[1]=syrus_buffer[4]; // centenas*
              char_password[2]=syrus_buffer[5]; // decenas
              char_password[3]=syrus_buffer[6]; // unidades
              char_password[4]=0x00;
              password_1 = atol(char_password);
              write_int16_ext_eeprom(password_1_address,password_1);

              char_password[0]=syrus_buffer[7]; // millares
              char_password[1]=syrus_buffer[8]; // centenas
              char_password[2]=syrus_buffer[9]; // decenas
              char_password[3]=syrus_buffer[10]; // unidades*
              char_password[4]=0x00;
              password_2 = atol(char_password);
              write_int16_ext_eeprom(password_2_address,password_2);

              char_password[0]=syrus_buffer[11]; // millares*   
              char_password[1]=syrus_buffer[12]; // centenas
              char_password[2]=syrus_buffer[13]; // decenas
              char_password[3]=syrus_buffer[14]; // unidades
              char_password[4]=0x00;
              password_3 = atol(char_password);
              write_int16_ext_eeprom(password_3_address,password_3);

              char_password[0]=syrus_buffer[15]; // millares   
              char_password[1]=syrus_buffer[16]; // centenas
              char_password[2]=syrus_buffer[17]; // decenas*
              char_password[3]=syrus_buffer[18]; // unidades
              char_password[4]=0x00;
              password_4 = atol(char_password);
              write_int16_ext_eeprom(password_4_address,password_4);

              password_1_used=false;
              password_2_used=false;
              password_3_used=false;
              password_4_used=false;
              bit_clear(status_word,0);
              bit_clear(status_word,1);
              bit_clear(status_word,2);
              bit_clear(status_word,3);                
              write_ext_eeprom(status_word_address, status_word);

//!              fprintf(syrus,"%s",pass_received_pinlock); // *111,
//!              fprintf(syrus, "%04Lu", password_1);
//!              fprintf(syrus, "%04Lu", password_2);
//!              fprintf(syrus, "%04Lu", password_3);
//!              fprintf(syrus, "%04Lu", password_4);
//!              fprintf(syrus,"#");
              
              //fprintf(syrus,"%s%s%c",pass_received_pinlock,letter_pass,footer);
              
               // Aqui se comienza con el calculo de la contraseña maestra tomando un digito que cada una de las contraseñas iniciales
               char_password_master[4]=0x00;
               char_password_master[3]=syrus_buffer[10]; // unidades*
               char_password_master[2]=syrus_buffer[17]; // decenas*
               char_password_master[1]=syrus_buffer[4]; // centenas*
               char_password_master[0]=syrus_buffer[11]; // millares*   
               password_test_master = atol(char_password_master);
               password_test_nib0 = make8(password_test_master,0);
               password_test_nib1 = make8(password_test_master,1);
               swap(password_test_nib0); 
               swap(password_test_nib1); 
               password_test_master = (make16(password_test_nib1,password_test_nib0))+40321;   
               password_test_master = ~password_test_master;          
               if((password_test_master==0)||(password_test_master>9999))
               {
                  while((password_test_master==0)||(password_test_master>9999))
                  {                 
                     password_test_nib0 = make8(password_test_master,0);
                     password_test_nib1 = make8(password_test_master,1);
                     swap(password_test_nib0); 
                     swap(password_test_nib1); 
                     password_test_master = (make16(password_test_nib1,password_test_nib0))+40321;     
                     password_test_master = ~password_test_master;                                 
                  }
               }                     
               password_master = password_test_master;
               write_int16_ext_eeprom(password_master_address,password_master),
               password_master_used = false;
               write_ext_eeprom(password_master_used_address,'n');
               //fprintf(syrus,"\r\n*Master:%Lu#",password_master);
               //fprintf(syrus,"%s%s%c",pass_received_pinlock,letter_pass,footer);
               fprintf(syrus,"%s%s%c",pass_received_pinlock,letter_pass,footer);
               valid_pass_cmd = true;
               general_reset_timer=0;
            }
            break;

            case 0x01: // Se cargan las cuatro nuevas contraseÃ±as que seran utilizadas -  *\FF\001111222233334444#
            i=3;
            letter_index = 0;
            do{ // Se verifica primeramente que la contraseÃ±a solo contenga numeros
              if((syrus_buffer[i]>='q')&&(syrus_buffer[i]<='z'))
               {
                  letter_pass[letter_index] = syrus_buffer[i];
                  switch (syrus_buffer[i])
                  {
                     case 'z':
                     syrus_buffer[i] = '0';
                     break;   
         
                     case 'y':
                     syrus_buffer[i] = '1';
                     break;
         
                     case 'x':
                     syrus_buffer[i] = '2';
                     break;
         
                     case 'w':
                     syrus_buffer[i] = '3';
                     break;
         
                     case 'v':
                     syrus_buffer[i] = '4';
                     break;
         
                     case 'u':
                     syrus_buffer[i] = '5';
                     break;
         
                     case 't':
                     syrus_buffer[i] = '6';
                     break;
         
                     case 's':
                     syrus_buffer[i] = '7';
                     break;
         
                     case 'r':
                     syrus_buffer[i] = '8';
                     break;
         
                     case 'q':
                     syrus_buffer[i] = '9';
                     break;
                  }
                  valid_data=true;
               }
              else
              {valid_data=false;}
              i++;   
              letter_index++;
            }while((i!=19)&&(valid_data));
            letter_pass[letter_index] = 0x00;
            if(valid_data)
            {
              char_password[0]=syrus_buffer[3];   
              char_password[1]=syrus_buffer[4];
              char_password[2]=syrus_buffer[5];
              char_password[3]=syrus_buffer[6];
              char_password[4]=0x00;
              password_5 = atol(char_password);
              write_int16_ext_eeprom(password_5_address,password_5);

              char_password[0]=syrus_buffer[7];   
              char_password[1]=syrus_buffer[8];
              char_password[2]=syrus_buffer[9];
              char_password[3]=syrus_buffer[10];
              char_password[4]=0x00;
              password_6 = atol(char_password);
              write_int16_ext_eeprom(password_6_address,password_6);

              char_password[0]=syrus_buffer[11];   
              char_password[1]=syrus_buffer[12];
              char_password[2]=syrus_buffer[13];
              char_password[3]=syrus_buffer[14];
              char_password[4]=0x00;
              password_7 = atol(char_password);
              write_int16_ext_eeprom(password_7_address,password_7);

              char_password[0]=syrus_buffer[15];   
              char_password[1]=syrus_buffer[16];
              char_password[2]=syrus_buffer[17];
              char_password[3]=syrus_buffer[18];
              char_password[4]=0x00;
              password_8 = atol(char_password);
              write_int16_ext_eeprom(password_8_address,password_8);

              password_5_used=false;
              password_6_used=false;
              password_7_used=false;
              password_8_used=false;
              bit_clear(status_word,4);
              bit_clear(status_word,5);
              bit_clear(status_word,6);
              bit_clear(status_word,7);                
              write_ext_eeprom(status_word_address, status_word);

//!              fprintf(syrus,"%s",pass_received_cargolock); // *111,
//!              fprintf(syrus, "%04Lu", password_5);
//!              fprintf(syrus, "%04Lu", password_6);
//!              fprintf(syrus, "%04Lu", password_7);
//!              fprintf(syrus, "%04Lu", password_8);
//!              fprintf(syrus,"#");
              
              fprintf(syrus,"%s%s%c",pass_received_cargolock,letter_pass,footer);
              valid_pass_cmd = true;
              general_reset_timer=0;
            }
            break;   
          }
        }
        else if(syrus_buffer[1]==0xCC) // Aqui se ingresaran los comandos de lectura para el multiplexor
        {
            
        } 
      }
      
      if((!valid_cmd)&&(!valid_pass_cmd)&&(!valid_cmd_aux))
      {
        fprintf(syrus,"%s",cmd_error);
      }
      
      if((!valid_pass_cmd)&&(valid_cmd_aux)&&(valid_cmd))
      {
        fprintf(syrus,"%s",cmd_pass_error);
      }


      for (aux_counter = 0; aux_counter < 150; ++aux_counter)
      {
        syrus_buffer[aux_counter] = 0;
      }

      index=0;   
      data_buffer=false;
      enable_interrupts(global);  
   }  
}

void send_to_buffer_syrus(char c)
{ 
   restart_wdt(); 
   syrus_buffer[index]=c; // Se alamcena el caracter entrante en el ultimo nivel de la pila del buffe
   if(index>=syrus_buffer) 
   {
      index = 0;
   }
   else
   {
      if(syrus_buffer[index]=='#') // si el caracter entrante es '#' se considera final de cadena y se analiza el bufer
      {
         data_buffer=true; // activando la bandera de 'Dato listo en buffer'
      }
      else
      {     
         index++;   
      }
   }   
}

void unlock_gps()
{
  restart_wdt(); 
  unlock_by_door();
  write_ext_eeprom(security_enabled_flag_address, 'n');
  for (long_gpvar = 0; long_gpvar < 3500; ++long_gpvar)
  {
    restart_wdt();
    delay_ms(1);
  }
  //output_low(gnd_output);
}

void test_ok()
{ 
   short do_ok=false;
   short do_cancel=false;
   int16 gp_int16_var;
   
   restart_wdt(); 
   if(test_ok_flag==true)
   {
     start_debounce_time();
     disable_interrupts(global);
     gp_int16_var=0;
     turn_off_7seg();
     delay_ms(300);
     turn_on_7seg();
     do{
         restart_wdt(); 
        display_msg(2);
        if(!input(ok))
           {do_ok=true;}
        
        if(!input(cancel))
           {do_cancel=true;}
        gp_int16_var++;
        delay_ms(1);
        
     }while((gp_int16_var<=10000)&&(!do_ok)&&(!do_cancel));
     
     if(do_ok)
     {
        if((display_number==password_1)&&(!password_1_used))
        {
           display_ok();
           password_1_used=true;
           bit_set(status_word,0);
           write_ext_eeprom(status_word_address, status_word);
           fprintf(syrus, "*%u,%04Lu#", used_password_id_pinlock, password_1);
           password_1 = 10000;
           write_int16_ext_eeprom(password_1_address,password_1);
           //update_eeprom_flag = true;
           unlock_gps();
        }
        else if((display_number==password_2)&&(!password_2_used))
        {
           display_ok();
           password_2_used=true;
           bit_set(status_word,1);
           write_ext_eeprom(status_word_address, status_word);
           fprintf(syrus, "*%u,%04Lu#", used_password_id_pinlock, password_2);
           password_2 = 10000;
           write_int16_ext_eeprom(password_2_address,password_2);
           //update_eeprom_flag = true;
           unlock_gps();
        }
        else if((display_number==password_3)&&(!password_3_used))
        {
           display_ok();
           password_3_used=true;
           bit_set(status_word,2);
           write_ext_eeprom(status_word_address, status_word);
           //update_eeprom_flag = true;
           fprintf(syrus, "*%u,%04Lu#", used_password_id_pinlock, password_3);
           password_3 = 10000;    
           write_int16_ext_eeprom(password_3_address,password_3);
           unlock_gps();
        }
        else if((display_number==password_4)&&(!password_4_used))
        {
           display_ok();
           password_4_used=true;
           bit_set(status_word,3);
           write_ext_eeprom(status_word_address, status_word);
           //update_eeprom_flag = true;
           fprintf(syrus, "*%u,%04Lu#", used_password_id_pinlock, password_4);
           password_4 = 10000;    
           write_int16_ext_eeprom(password_4_address,password_4);
           unlock_gps();
        }
        else if((display_number==password_master)&&(!password_master_used)) // La  contraseÃ±a  maestra desbloquea todas las salidas ademas apaga el sistema de seguridad
        {
           display_ok();
           password_master_used=true;
           write_ext_eeprom(password_master_used_address, 's');
           fprintf(syrus, "*%u,%04Lu#", used_password_id_pinlock, password_master);
           password_master = 10000;   
           write_int16_ext_eeprom(password_master_address,password_master);
           disable_all_security_flag = true;
        }
        else if((display_number==password_5)&&(!password_5_used))
        {
           display_ok();
           password_5_used=true;
           bit_set(status_word,4);
           write_ext_eeprom(status_word_address, status_word);
           fprintf(syrus, "*%u,%04Lu#", used_password_id_cargolock, password_5);
           password_5 = 10000;
           write_int16_ext_eeprom(password_5_address,password_5);
           //update_eeprom_flag = true;
           unlock_cargolock();
        }
        else if((display_number==password_6)&&(!password_6_used))
        {
           display_ok();
           password_6_used=true;
           bit_set(status_word,5);
           write_ext_eeprom(status_word_address, status_word);
           fprintf(syrus, "*%u,%04Lu#", used_password_id_cargolock, password_6);
           password_6 = 10000;
           write_int16_ext_eeprom(password_6_address,password_6);
           //update_eeprom_flag = true;
           unlock_cargolock();
        }
        else if((display_number==password_7)&&(!password_7_used))
        {
           display_ok();
           password_7_used=true;
           bit_set(status_word,6);
           write_ext_eeprom(status_word_address, status_word);
           //update_eeprom_flag = true;
           fprintf(syrus, "*%u,%04Lu#", used_password_id_cargolock, password_7);
           password_7 = 10000;    
           write_int16_ext_eeprom(password_7_address,password_7);
           unlock_cargolock();
        }
        else if((display_number==password_8)&&(!password_8_used)) // La 4ta contraseÃ±a ahora es la maestra y desbloquea todas las salidas ademas apaga el sistema de seguridad
        {
           display_ok();
           password_8_used=true;
           bit_set(status_word,7);
           write_ext_eeprom(status_word_address, status_word);
           //update_eeprom_flag = true;
           fprintf(syrus, "*%u,%04Lu#", used_password_id_cargolock, password_8);
           password_8 = 10000;   
           write_int16_ext_eeprom(password_8_address,password_8);
           unlock_cargolock();
        }
        else if(display_number==0x00)
        {
            display_ok();
            fprintf(syrus, "%s", block_by_keypad);
            if(lock_msg_running_flag)
            {
               fprintf(syrus, "%s", lock_msg_running);
            }
            else
            {
               lock_cargolock();
            }
        }
        else // ni una contraseÃ±a ingresada en el teclado es valida, asi que no se comienzan los registro
        {
           fprintf(syrus, "%s%04Lu%c", bad_pass_used,display_number,footer);
           if(block_by_bad_pass_used_flag)
           {
               enable_v3 = true;   
               v3_timer = 0;
           }
           display_error();
           tries+=1;
           wrong_pass[tries] = display_number;
           write_ext_eeprom(tries_address, tries);
           switch (tries)
           {
               case 1:
               write_int16_ext_eeprom(wrong_pass_address_1,wrong_pass[tries]); 
               break;
               
               case 2:
               write_int16_ext_eeprom(wrong_pass_address_2,wrong_pass[tries]); 
               break;
               
               case 3:
               write_int16_ext_eeprom(wrong_pass_address_3,wrong_pass[tries]); 
               break;
               
               case 4:
               write_int16_ext_eeprom(wrong_pass_address_4,wrong_pass[tries]); 
               break;
               
               case 5:
               write_int16_ext_eeprom(wrong_pass_address_5,wrong_pass[tries]); 
               break;
               
               default:
               write_int16_ext_eeprom(wrong_pass_address_5,wrong_pass[tries]); 
               break;
           }
           
           if(tries>=6)
           {
              block_keypad='s';
              write_ext_eeprom(tries_address, tries);
              block_timer = block_time_2;
              fprintf(syrus,"%s%04Lu,,%c",wrong_pass_typed,wrong_pass[tries],footer);
           }
           else if(tries>=5)
           {
              block_keypad='s';
              write_ext_eeprom(tries_address, tries);
              block_timer = block_time_1;
              //fprintf(syrus,"Contraseña erronea: %04Lu,%04Lu,\r\n",wrong_pass[4],wrong_pass[5]);
              fprintf(syrus,"%s%04Lu,%04Lu,%c",wrong_pass_typed,wrong_pass[4],wrong_pass[5],footer);
           }
           else if (tries==3)
           {
              block_keypad='s';
              write_ext_eeprom(tries_address, tries);
              block_timer = block_time_0;
              //fprintf(syrus,"Contraseña erronea: %04Lu,%04Lu,%04Lu\r\n",wrong_pass[1],wrong_pass[2],wrong_pass[3]);
              fprintf(syrus,"%s%04Lu,%04Lu,%04Lu%c",wrong_pass_typed,wrong_pass[1],wrong_pass[2],wrong_pass[3],footer);
           }
           
        }
     }
     
     if(do_cancel)
     {
        //fprintf(syrus,"Oprimi CANCEL\r\n");
     }
     
     if(gp_int16_var>=10000)
     {
        fprintf(syrus, "%s%04Lu%c", waiting_confirmation,display_number,footer);
     }

     
     test_ok_flag=false;
     enable_interrupts(global);
   }
}

void test_keypad()
{
  restart_wdt(); 
  if(display_off_timer<display_off_time)
  {
    // Monitorea el boton RIGHT
    if(!input(right)&&(release_button)) 
    {
      start_debounce_time(); 
      if((digit_position<=3)&&(digit_position>=1))
      {
         digit_position--;
      }
      else
      {
         digit_position=3;
      }
    }

    // Monitorea el boton LEFT
    if(!input(left)&&(release_button)) 
    {
      start_debounce_time();
      if((digit_position>=0)&&(digit_position<=2))
      {
         digit_position++;
      }
      else
      {
         digit_position=0;
      }  
    }

    // Monitorea el boton ok      
    if(!input(ok)&&(release_button)) 
    {
       start_debounce_time(); 
       turn_on_7seg();
       test_ok_flag=true;
    }            

    // Monitorea el boton up     
    if(!input(up)&&(release_button)) 
    {
       switch (digit_position)
       {
          case 0:
          if(u<=8)
          {display_number++;}
          else
          {display_number-=9;}
          break;
          
          case 1:
          if(d<=8)
          {display_number+=10;}
          else
          {display_number-=90;}
          break;
          
          case 2:
          if(c<=8)
          {display_number+=100;}
          else
          {display_number-=900;}
          break;
          
          case 3:
          if(m<=8)
          {display_number+=1000;}
          else
          {display_number-=9000;}
          break;
       }
       start_debounce_time(); 
    }

    // Monitorea el boton down      
    if(!input(down)&&(release_button)) 
    {
       switch (digit_position)
       {
          case 0:
          if(u>=1)
          {display_number--;}
          else
          {display_number+=9;}
          break;
          
          case 1:
          if(d>=1)
          {display_number-=10;}
          else
          {display_number+=90;}
          break;
          
          case 2:
          if(c>=1)
          {display_number-=100;}
          else
          {display_number+=900;}
          break;
          
          case 3:
          if(m>=1)
          {display_number-=1000;}
          else
          {display_number+=9000;}
          break;
       }
       start_debounce_time(); 
    }
  }
  else
  {
    if(!input(right)||!input(left)||!input(up)||!input(down)||!input(ok)||!input(cancel)) 
    {
      start_debounce_time(); 
      display_off_timer = 0;
    }
  }
}

void start_debounce_time()
{
   restart_wdt(); 
   display_off_timer = 0;
   turn_on_7seg();
   set_timer1(0); 
   i_timer1 = 0;
   release_button=false;
   enable_interrupts(INT_TIMER1);
}

void system_display_sleep()
{
   restart_wdt(); 
   if(display_off_timer>=display_off_time)
   {
      turn_off_7seg();
   }
}

void display_error()
{
   restart_wdt(); 
   long_gpvar=0;
   do{
      restart_wdt();
      display_msg(0);
      long_gpvar++;
      delay_ms(1);
   }while(long_gpvar!=3000);   
   
   long_gpvar=0;
   do{
      restart_wdt();
      display_msg(3);
      long_gpvar++;
      delay_ms(1);
   }while(long_gpvar!=300);  
}

void display_ok()
{
   restart_wdt(); 
   long_gpvar=0;
   do{
      restart_wdt();
      display_msg(1);
      long_gpvar++;
      delay_ms(1);
   }while(long_gpvar!=3000);   
   
   long_gpvar=0;
   do{
      restart_wdt();
      display_msg(3);
      long_gpvar++;
      delay_ms(1);
   }while(long_gpvar!=300);     
   tries = 0;
}

void display_block_timer()
{
   restart_wdt(); 
   if(block_keypad=='s')
   {
      disable_interrupts(INT_TIMER2);
      do{
         display_7seg(block_timer,digit_position);
         digit_position++;
         if(digit_position>=4)
            {digit_position=0;}
         delay_ms(1);
      }while(block_timer>1);
      //update_eeprom_flag = True;     
      enable_interrupts(INT_TIMER2);
      test_ok_flag=false;
      block_keypad='n';
      write_ext_eeprom(blocked_keypad_address,block_keypad); 
      //tries=0;
      reset_tries_timer=0;
   }
}

void make_digits()
{
   restart_wdt(); 
   m = display_number / 1000;
   c = (display_number -(m*1000))/100;
   d = (display_number - (m*1000 + c*100))/10;
   u = display_number -(m*1000 + c*100 + d*10 );
}

void init_pinlock()
{
   restart_wdt(); 
   output_high(out_1); // mantiene activa la salida 1 mientras el pinlcok este ejecutandose, si se apaga o corta, esta salida pierte seÃ±al negativa
   //output_low(out_0); // mantiene desactivada la salida 0, despues de detectarse la entrada de tamper y cumplirse su timer, se activa la salida
   execute_v0();
   load_status_word();
   keep_alive_accesory_counter = 0;
   disable_all_security_flag = false;

   // Bandera para activar o desactivar el Bloqueo Por Puenteo del Copiloto
   temp_doors_var = read_ext_eeprom(bpc2_address);
   if(temp_doors_var=='s')
   {
     bpc2_overtime_option = true;
   }
   else
   {
     bpc2_overtime_option = false;
   }
      
   // Respaldo del ultimo estado de la entrada #0 = Sensor de Quinta   
   last_input_state_backup = read_ext_eeprom(in_0_address);
   switch (last_input_state_backup)
   {
      case 0x00:
      last_input_state = false;
      break;
      
      case 0xFF:
      last_input_state = true;
      break;
      
      default:
      if(input(in_0)) // Entrada  activada
      {
         last_input_state = true;
      }
      else
      {
         last_input_state = false;
      }
      break;
   }
   
   // Respaldo del ultimo estado de la entrada #1 = Piloto         
   last_input_state_backup = read_ext_eeprom(in_1_address);
   switch (last_input_state_backup)
   {
     case 0x00:
     last_input1_state = false;
     break;
     
     case 0xFF:
     last_input1_state = true;
     break;
     
     default:
     if(!input(in_1)) // Entrada  activada
     {
        last_input1_state = true;
     }
     else
     {
        last_input1_state = false;
     }
     break;
   }

     // Respaldo del ultimo estado de la entrada #3 = Copiloto
   last_input_state_backup = read_ext_eeprom(in_3_address);
   switch (last_input_state_backup)
   {
     case 0x00:
     last_input3_state = false;
     break;
     
     case 0xFF:
     last_input3_state = true;
     break;
     
     default:
     if(!input(in_3)) // Entrada  activada
     {
        last_input3_state = true;
     }
     else
     {
        last_input3_state = false;
     }
     break;
   }
      
   
   //fprintf(accesory,"%s%c", msg_inicio_accesory, 0xFE);
   index = 0;
   general_reset_timer = 0;
   input0_debouce_counter=0;
   display_number = 2099;
   gpvar = 0;
   rtc_engine_off = 0; // Contador binario del tiempo transcurrido desde que se apago el motor
   rtc_door = 0; // Contador binario que indica el tiempo transcurrido desde que se apago el motor hasta que hubo un cambio en el estado de las puertas
   rtc_door_backup = 0; // El contador de respaldo se inicia en 0 cada que se reinicia el programa
   
   // cargar el tiempo que las salidas analogicas permaneceran activas cuando se ejecuten
   an_out_time = read_int16_ext_eeprom(an_out_time_address);
   if(an_out_time==0xFFFF) // si es la primera carga de datos, por dafualt el tiempo sera de 10 segundos
   {
      an_out_time = 10000;
      write_int16_ext_eeprom(an_out_time_address, an_out_time);
   }
   
   tries = read_ext_eeprom(tries_address);
   if(tries==255)
   {
      tries = 0;
   }
   
   if(tries==0)
   {
      wrong_pass[1] = 0;
      wrong_pass[2] = 0;
      wrong_pass[3] = 0;
      wrong_pass[4] = 0;
      wrong_pass[5] = 0;
   }
   else
   {
      wrong_pass[1] = read_int16_ext_eeprom(wrong_pass_address_1);
      wrong_pass[2] = read_int16_ext_eeprom(wrong_pass_address_2);
      wrong_pass[3] = read_int16_ext_eeprom(wrong_pass_address_3);
      wrong_pass[4] = read_int16_ext_eeprom(wrong_pass_address_4);
      wrong_pass[5] = read_int16_ext_eeprom(wrong_pass_address_5);
   }
   
   block_keypad = read_ext_eeprom(blocked_keypad_address);
   if(block_keypad=='s') // 
   {
     if(tries>=6)
     {
        block_timer = block_time_2;
     }
     else if(tries>=5)
     {
        block_timer = block_time_1;
     }
     else if (tries>=3)
     {
        block_timer = block_time_0;
     }
   }
   
   // pregunta si la funcion de bloqueo por puerta de piloto abierta esta activada
   temp_doors_var = read_ext_eeprom(test_copilot_door_flag_address);
   if(temp_doors_var=='s')
   {
      test_copilot_door_flag = true;
   }
   else
   {
      test_copilot_door_flag = false;
   }
   
   // pregunta si la entrada 3 esta habilitada para estar leyendola Puerta de Copiloto
   temp_doors_var = read_ext_eeprom(input3_enable_address);
   if(temp_doors_var=='s')
   {
      input3_enable = true;
   }
   else
   {
      input3_enable = false;
   }
   
   // al iniciar verifica en la eeprom si el sistema de bloqueo por puertas estara activado al reiniciarse (manipulacion previa)
//!   temp_doors_var = read_ext_eeprom(security_enabled_flag_address);
//!   if(temp_doors_var=='s')
//!   {
//!      //security_enabled = true;
//!      //
//!      lock_by_door();
//!   }
//!   else
//!   {
//!      //security_enabled = false;
//!      unlock_by_door();
//!   }
   
   // verifica si la proteccion de quinta holland y traba quinta esta activada 
   temp_doors_var = read_ext_eeprom(type_holland_address);
   if((temp_doors_var==255)||(temp_doors_var=='s'))
   {
      type_holland = true;
   }
   else
   {
      type_holland = false;
   }

   // umbral para detectar la apertura de puertas - Esta parte del codigo no esta haciendo nada
   long_gpvar = read_int16_ext_eeprom(umbral_debounce_copilot_door_address);
   if(long_gpvar==0xFFFF)
   {
      umbral_debounce_copilot_door = 400;
   }
   else
   {
      umbral_debounce_copilot_door = long_gpvar;
   }
   
   // Se carga el tiempo que tarda en activarse la salida 0 despues de detectar entrada 1
   long_gpvar = read_int16_ext_eeprom(out_0_time_address);
   if(long_gpvar==0xFFFF)
   {
      out_0_time = 300;
   }
   else
   {
      out_0_time = long_gpvar;
   }
   
   long_gpvar = read_int16_ext_eeprom(v3_time_address); // Carga el tiempo de retardo despues de apagar el vehiculo para mandar voltaje de bloqueo
   if(long_gpvar==0xFFFF)
   {
      v3_time = 5;
      write_int16_ext_eeprom(v3_time_address, v3_time);
   }
   else
   {
      v3_time = long_gpvar;
   }
   
   temp_doors_var = read_ext_eeprom(v3_enable_address); // Carga la bandera que habilitara o no el envio del voltaje de bloqueo por apagado de motor
   if(temp_doors_var=='s')
   {
      input4_adc_enable = true;
   }
   else
   {
      input4_adc_enable = false;
   }
   
   temp_doors_var = read_ext_eeprom(bpp_flag_option_address); // se carga el estado de la bandera para el bloqueo por puenteo desde la eeprom al inciar el equipo
   if(temp_doors_var=='s') // Si es igual a 's' entonces la bandera es verdadera
   {
     bpp_flag_option = true;
   }
   else // Si no, la bandera es falsa y la opcion de BPP no estara habilitada
   {
     bpp_flag_option = false;
   }
   
   temp_doors_var = read_ext_eeprom(bpc1_address); // Se carga el estado de la bandera para el bloqueo por corte en puerta del piloto
   if(temp_doors_var=='s')
   {
     bpc1_overtime_option = true;
   }
   else
   {
     bpc1_overtime_option = false;
   }
   
   // Carga el valor inicial del envio de voltahe de bloqueo para funcion bpc1
   temp_doors_var = read_ext_eeprom(exe_bpc1_flag_address);
   if(temp_doors_var=='s'){
     exe_bpc1_flag = true;}
   else{
     exe_bpc1_flag = false;}
   
   // Carga el valor inicial del envio de voltahe de bloqueo para funcion bpc2
   temp_doors_var = read_ext_eeprom(exe_bpc2_flag_address);
   if(temp_doors_var=='s'){
     exe_bpc2_flag = true;}
   else{
     exe_bpc2_flag = false;}
   
   // Carga el valor inicial del envio de voltahe de bloqueo para funcion bpp
   temp_doors_var = read_ext_eeprom(exe_bpp_flag_address);
   if(temp_doors_var=='s'){
     exe_bpp_flag = true;}
   else{
     exe_bpp_flag = false;}
     
   long_gpvar = read_int16_ext_eeprom(bpc1_overtime_time_address);
   if(long_gpvar==0xFFFF)
   {
      bpc1_overtime_time = 300;
   }
   else
   {
      bpc1_overtime_time = long_gpvar;
   }

   long_gpvar = read_int16_ext_eeprom(bpc2_overtime_time_address);
   if(long_gpvar==0xFFFF)
   {
      bpc2_overtime_time = 300;
   }
   else
   {
      bpc2_overtime_time = long_gpvar;
   }

   long_gpvar = read_int16_ext_eeprom(bpc1_time_address);
   if(long_gpvar==0xFFFF)
   {
      bpc1_time = 900;
   }
   else
   {
      bpc1_time = long_gpvar;
   }

   long_gpvar = read_int16_ext_eeprom(bpc2_time_address);
   if(long_gpvar==0xFFFF)
   {
      bpc2_time = 900;
   }
   else
   {
      bpc2_time = long_gpvar;
   }

   long_gpvar = read_int16_ext_eeprom(bpp_time_address);
   if(long_gpvar==0xFFFF)
   {
      bpp_time = 900;
   }
   else
   {
      bpp_time = long_gpvar;
   }
   
   // Carga el valor inicial del envio de voltaje de bloqueo por apertura de puerta del piloto
   temp_doors_var = read_ext_eeprom(exe_monitor1_flag_address);
   if(temp_doors_var=='s'){
     exe_monitor1_flag = true;}
   else{
     exe_monitor1_flag = false;}
     
   // Carga el valor inicial del envio de voltaje de bloqueo por apertura de puerta del copiloto
   temp_doors_var = read_ext_eeprom(exe_monitor3_flag_address);
   if(temp_doors_var=='s'){
     exe_monitor3_flag = true;}
   else{
     exe_monitor3_flag = false;}

   // Carga la opcion para activar o no la funcionalidad del bloqueo secundario
   temp_doors_var = read_ext_eeprom(out0_option_address);
   if(temp_doors_var=='s'){
     out0_option = true;}
   else{
     out0_option = false;}  
  
  // Carga las baneras relacionadas al forzado de la activacion de la salida 0 que se usa como bloqueo secundario en los pinlock
  temp_doors_var = read_ext_eeprom(out0_cmd_mode_address);
   if(temp_doors_var=='s')
   {
      out0_cmd_mode_flag = true;
      temp_doors_var = read_ext_eeprom(out0_cmd_mode_last_stat_address);
      if(temp_doors_var=='s')
      {
         out0_cmd_mode_last_stat_flag = true;
         output_high(out_0);
      }
      else
      {
         out0_cmd_mode_last_stat_flag = false;
         output_low(out_0);
      }
   }
   else
   {
      out0_cmd_mode_flag = false;
   }
   // Carga los valores relacionados a la suspencion de la activacion de la salida 0 
   temp_doors_var = read_ext_eeprom(temporal_out0_off_flag_address);
   if(temp_doors_var=='s')
   {
      temporal_out0_off_flag = true;
   }
   else
   {
      temporal_out0_off_flag = false;
   } 
   
   // Carga la bandera de transicion de la suspencion de la activacion de la salida 0
   temp_doors_var = read_ext_eeprom(temporal_out0_off_transition_address);
   if((temp_doors_var!='s')&&(temp_doors_var!='n')) // si es la primera vez entonces por defaul = false
   {
      temporal_out0_off_transition = true;
   }
   else
   {
      if(temp_doors_var=='s')
      {
         temporal_out0_off_transition = true;
      }
      else
      {
         temporal_out0_off_transition = false;
      }
   }
   
   temp_doors_var = read_ext_eeprom(out_0_activated_address);
   if(temp_doors_var=='s')
   {
      out_0_activated = true;
      output_high(out_0);
   }
   else
   {
      out_0_activated = false;
   } 
   
   temp_doors_var = read_ext_eeprom(serial_address_flag);
   if(temp_doors_var=='s')
   {
     set_serial_flag = true;
     for (temp_doors_var = 0;temp_doors_var < 13; ++temp_doors_var)
     {
        serial_number[temp_doors_var] = read_ext_eeprom(serial_address+temp_doors_var);
     }
     serial_number[13] = 0x00;
     //fprintf(syrus,"*>>>%s%c",serial_number,footer);
   }
   else{
     set_serial_flag = false;}
   
   temp_doors_var = read_ext_eeprom(block_by_bad_pass_used_address);
   if(temp_doors_var=='s')
   {
         block_by_bad_pass_used_flag = true;
   }
   else
   {
         block_by_bad_pass_used_flag = false;   
   }
   
   temp_doors_var = read_ext_eeprom(acc_detected_address);
   if(temp_doors_var=='s')
   {
         acc_detected = true;
   }
   else
   {
         acc_detected = false;   
   }
   
   temp_doors_var = read_ext_eeprom(acc_lost_counter_address);
   if(temp_doors_var==0xFF)
   {
      acc_lost_counter = 0;
   }
   else
   {
      acc_lost_counter = temp_doors_var;
   }
   
   long_gpvar = read_int16_ext_eeprom(acc_lost_max_mgs_counter_address); // Carga el tiempo de retardo despues de apagar el vehiculo para mandar voltaje de bloqueo
   if(long_gpvar==0xFFFF)
   {
      acc_lost_max_mgs_counter = 0;
      write_int16_ext_eeprom(acc_lost_max_mgs_counter_address, acc_lost_max_mgs_counter);
   }
   else
   {
      acc_lost_max_mgs_counter = long_gpvar;
   }

   long_gpvar = read_int16_ext_eeprom(acc_lost_long_msg_timer_address); // Carga el tiempo de retardo despues de apagar el vehiculo para mandar voltaje de bloqueo
   if(long_gpvar==0xFFFF)
   {
      acc_lost_long_msg_timer = 0;
      write_int16_ext_eeprom(acc_lost_long_msg_timer_address, acc_lost_long_msg_timer);
   }
   else
   {
      acc_lost_long_msg_timer = long_gpvar;
   }
   
   display_off_timer = 0;
   fprintf(syrus,"%s%s,%s%c", init_msg,msg_inicio,serial_number,footer);
   init_interrupts();
}

void init_interrupts()
{
   restart_wdt(); 
   enable_interrupts(INT_RTCC);
   enable_interrupts(INT_TIMER1);
   enable_interrupts(INT_TIMER2);
   //enable_interrupts(INT_TIMER4);
   //enable_interrupts(INT_TIMER5);
   //enable_interrupts(INT_RDA);
   enable_interrupts(INT_RDA2);
   //enable_interrupts(INT_RDA3);
   enable_interrupts(INT_RDA4);
   enable_interrupts(GLOBAL);
}

void send_to_buffer_accesory(char d)
{ 
   restart_wdt(); 
   accesory_buffer[index2] = d; // Se alamcena el caracter entrante en el ultimo nivel de la pila del buffe
   
   if(index2>=accesory_buffer) 
   {
      index2 = 0;
   }
   else
   {
      if(accesory_buffer[index2]=='#') // si el caracter entrante es '#' se considera final de cadena y se analiza el bufer
      {
         data_buffer2=true; // activando la bandera de 'Dato listo en buffer'
      }
      else
      {     
         index2++;
//!         if(d!=0xFE)
//!         {
//!            index2++;  
//!         }   
      }
   }   
}

void scan_cmd2() // Escanear comandos en el puerto serial del controlador del motor
{
   short valid_cmd2=false;
   int length=0;
   int i=0;
   // char cadena_ejemplo[]="*Identificar_esta_trama#";

   restart_wdt(); 
   if(data_buffer2==true)
   {
      disable_interrupts(global);
      strlwr (accesory_buffer);

      while(accesory_buffer[i]!='#')
      {
         i++;
         length++;
      }
      length++;  

      if((accesory_buffer[0]=='*')&&(accesory_buffer[length-1]=='#'))
      {
         if(!strncmp(blocking_box_ack, accesory_buffer, length))
         {
             fprintf(syrus,"%s", blocking_box_ack); // Respues de confirmacion de ejecucion por parte del controlador de motor
             //send_lock_msg = false; // no permite enviar nuevamente el comando de bloqueo
             //response_lock_flag = true; // bandera que indica que ya se recibio la respuesta
             //lock_msg_running_flag = false;
             valid_cmd2 = true; // comando valido
         }         
         else if((accesory_buffer[1]==0x0A)&&(accesory_buffer[2]==0x02))
         {  
            //fprintf(syrus,"+Detectado+\r\n");
            acc_connected = true;
            acc_lost_counter = 0;
            acc_lost_max_mgs_counter = 0;
            write_ext_eeprom(acc_lost_counter_address,acc_lost_counter);
            write_int16_ext_eeprom(acc_lost_max_mgs_counter_address, acc_lost_max_mgs_counter);
            valid_cmd2 = true;
         }
         else if(!strncmp(unblocking_box_ack, accesory_buffer, length))
         {
            fprintf(syrus,"%s", unblocking_box_ack);
            //fprintf(accesory,"*171 ACK#\r\n");
            valid_cmd2 = true;
         }
         else if(!strncmp(keep_alive_accesory, accesory_buffer, length))
         {
//!            keep_alive_accesory_counter++;
//!            if(keep_alive_accesory_counter>=20)
//!            {
//!               keep_alive_accesory_counter = 0;
//!               fprintf(syrus,"%s", keep_alive_accesory);  
//!            }
            //fprintf(syrus,"%s", keep_alive_accesory);  
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_178_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_178_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_179_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_179_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_180_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_180_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_181_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_181_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_182_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_182_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_189_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_189_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_190_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_190_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_191_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_191_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_192_0_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_192_0_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_192_1_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_192_1_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_192_2_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_192_2_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(accesory_192_3_msg, accesory_buffer, length))
         {
            fprintf(syrus,"%s", accesory_192_3_msg);
            valid_cmd2 = true;
         }
         else if(!strncmp(autolock_enable_resp, accesory_buffer, length))
         {
            fprintf(syrus,"%s", autolock_enable_resp);
            valid_cmd2 = true;
         }
         else if(!strncmp(autolock_disable_resp, accesory_buffer, length))
         {
            fprintf(syrus,"%s", autolock_disable_resp);
            valid_cmd2 = true;
         }
         else if(!strncmp(set_time_2_activate_autolock_resp, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(set_lock_time_1_cmd_resp, accesory_buffer, 22))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(lock_ok, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(unlock_ok, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(position_stroke_msg, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(lock_error, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(unlock_error, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(set_model_resp, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(qry_model_resp, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(send_model, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(qry_ios_resp, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(indutives_status, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(get_status_resp, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else if(!strncmp(force_blocking_box_resp, accesory_buffer, length))
         {
            fprintf(syrus,"%s", force_blocking_box_resp);
            valid_cmd2 = true;
         }
         else if(!strncmp(set_offset_0_resp, accesory_buffer, length))
         {
            fprintf(syrus,"%s", set_offset_0_resp);
            valid_cmd2 = true;
         }
         else if(!strncmp(set_offset_100_resp, accesory_buffer, length))
         {
            fprintf(syrus,"%s", set_offset_100_resp);
            valid_cmd2 = true;
         }
         else if(!strncmp(set_max_stroke_move_resp, accesory_buffer, length))
         {
            fprintf(syrus,"%s", set_max_stroke_move_resp);
            valid_cmd2 = true;
         }
         else if(!strncmp(remove_max_stroke_move_resp, accesory_buffer, length))
         {
            fprintf(syrus,"%s", remove_max_stroke_move_resp);
            valid_cmd2 = true;
         }
         else if(!strncmp(version, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            acc_detected = true;
            //fprintf(syrus,"Accesorio detectado\r\n");
            write_ext_eeprom(acc_detected_address,'s');
            valid_cmd2 = true;
         }
         else if(!strncmp(read_acc0_options0_resp, accesory_buffer, 4))
         {
            i=0;
            do{
               fprintf(syrus,"%c", accesory_buffer[i]);
               i++;
            }while(i!=length);
            valid_cmd2 = true;
         }
         else
         {
            valid_cmd2 = false;       
         }
      }  

      if(!valid_cmd2)
      {
         //fprintf(accesory,"*114,error#%c", 0xFE);
      }
      else
      {
        /*
        dofigo opcional
        */
      }

      for (aux_counter = 0; aux_counter < 150; ++aux_counter)
      {
         accesory_buffer[aux_counter] = 0;
      }

      index2=0;   
      data_buffer2=false;
      clear_interrupt(INT_RDA4);
      enable_interrupts(global);  
   }  
}

// Voltaje de referencia, no tiene efecto
void execute_v0()
{
   restart_wdt(); 
   write_wiper(129);
}

// voltaje de desbloqueo
void execute_v1()
{
   restart_wdt(); 
   write_wiper(100);
   //delay_ms(an_out_time);
   for(long_gpvar=0; long_gpvar<=an_out_time; ++long_gpvar)
   {
      delay_ms(1);
      restart_wdt(); 
   }
   execute_v0();
}

// voltaje de bloqueo por puertas abiertas
void execute_v2()
{
   restart_wdt(); 
   write_wiper(36); // 2.25VCD // 
   //delay_ms(an_out_time);
   for(long_gpvar=0; long_gpvar<=an_out_time; ++long_gpvar)
   {
      delay_ms(1);
      restart_wdt(); 
   }
   execute_v0();
}

void execute_v3() // Voltaje V3 que envia por el pot digital 1.25V
{
   restart_wdt(); 
   write_wiper(76); // 1.25VCD, voltaje para mandar bloquear por apagado de motor
   //delay_ms(an_out_time); // Retardo que mantendra activa la seÃ±al analogica deseada
   for(long_gpvar=0; long_gpvar<=an_out_time; ++long_gpvar)
   {
      delay_ms(1);
      restart_wdt(); 
   }
   execute_v0();   // Este voltaje mantiene la salida del potenciometro en 0
}

void execute_v4() // Voltaje V4 que envia por el pot digital 2.75V
{
   restart_wdt(); 
   write_wiper(22); // 2.75VCD, voltaje para desbloquear y apagar el sistema de sseguridad de bloqueos del syrus
   //delay_ms(an_out_time); // Retardo que mantendra activa la seÃ±al analogica deseada
   for(long_gpvar=0; long_gpvar<=an_out_time; ++long_gpvar)
   {
      delay_ms(1);
      restart_wdt(); 
   }
   execute_v0();   // Este voltaje mantiene la salida del potenciometro en 0
}

void test_input_1() // Puerta de Piloto
{
   restart_wdt(); 
   if(test_copilot_door_flag) // si el uso de la funcion de puertas abiertas esta activado
   {
      if(input1_debouce_counter>=input1_debouce)
      {  
         if(!input(in_1)) // 
         {
            input1_state = true;
         }
         else
         {
            input1_state = false;
            bpc1_timer = 0;
            bpc1_overtime_timer = 0;
            bpc1_overtime = false; // detiene los envios de bloqueo por corte porque ya se detecto puerta cerrada
         }
         
         // Si hubo un cambio de estado en la entrada o una peticion de la misma
         if((poll_input1_flag==1)||(input1_state!=last_input1_state)) 
         {
            //fprintf(syrus,"cambio entrada, %u,%u,%u\r\n", input2_state, last_input2_state, poll_input2);
            if((input1_state!=last_input1_state))
            {
               rtc_door_backup = rtc_door;
            }
            
            if(input1_state)
            {
               last_input1_state = true;
               write_ext_eeprom(in_1_address, 0xFF);
            }
            else
            {
               last_input1_state = false;
               write_ext_eeprom(in_1_address, 0x00);
            }
            

            if(input1_state) // Entrada  activada - Puerta de piloto abierta
            {
                 if(!poll_input1_flag) // Respuesta a cambio de estado
                 {
                   if(exe_monitor1_flag)
               {
                  fprintf(syrus,"%s", msg_input1_act); // Envia mensaje de entrada activada
                  lock_by_door(); // Bloquea por medio de voltaja analogico
               }
               else
               {
                  fprintf(syrus,"%s", msg_input1_act_2); // Envia mensaje de entrada activada - Funcion de bloqueo no activada
               }

                  }
                  else // Respuesta a peticion
                  {
                        if(exe_monitor1_flag)
                  {
                     fprintf(syrus,"%s", msg_poll_input1_act); // Envia mensaje de entrada activada
                  }
                  else
                  {
                     fprintf(syrus,"%s", msg_input1_act_2); // Envia mensaje de entrada activada - Funcion de bloqueo no activada
                  }
                  }
            }
            else  // entrada no activada - Puerta de piloto cerrada
            {
                  if(!poll_input1_flag) // Respuesta a cambio de estado
                  {
                        if(exe_monitor1_flag)
                  {
                     fprintf(syrus,"%s", msg_input1_deact); // Envia mensaje de entrada activada
                  }
                  else
                  {
                     fprintf(syrus,"%s", msg_input1_deact_2); // Envia mensaje de entrada activada - Funcion de bloqueo no activada
                  }
                  }
                  else // respuesta a peticion
                  {
                        if(exe_monitor1_flag)
                  {
                     fprintf(syrus,"%s", msg_poll_input1_deact); // Envia mensaje de entrada activada
                  }
                  else
                  {
                     fprintf(syrus,"%s", msg_input1_deact_2); // Envia mensaje de entrada activada - Funcion de bloqueo no activada
                  }
                  }
            }
            poll_input1_flag = false;      
         }      
         input1_debouce_counter = 0;
      }
   }   
}

// accion que bloqueara el acelerador por accion de aperturas de puertas
void lock_by_door()
{
  //output_high(aux_output);
  // Ejecuta el voltaje de bloqueo: 1.5<V2<2.0
  //fprintf(syrus,"V2\r\n");
  restart_wdt(); 
  execute_v2();
  write_ext_eeprom(security_enabled_flag_address, 's');
}

// accion que desbloqueara el acelerador por ingresar una contraseÃ±a correcta
void unlock_by_door()
{
  //output_low(aux_output);
  // Ejecuta el voltaje de desbloqueo: 0.5<V1<1.0
  restart_wdt(); 
  execute_v1(); // desbloqueo de puerta por uso de contrase�as
  write_ext_eeprom(security_enabled_flag_address, 'n');
}


// Verifica si la entrada 2 fue activada y si lo fue, comienza con un contador para activar la salida 0
// Para version shield TYT 
void test_input_2() 
{
   restart_wdt(); 
   if(out0_cmd_mode_flag==false)
   {
      if(input(in_2)) // Si la entrada dos es aterrizada
      {
        // Si: (la salida 0 no ha sido activada & ya se cumplio el retardo & esta activada la funcion de paro secundario)
        // Actualizacion >>
        // Si: (la salida 0 no ha sido activada & ya se cumplio el retardo & esta activada la funcion de paro secundario & la funcion temporal esta desactivada) 
         if((!out_0_activated)&&(out_0_timer>=out_0_time)&&(out0_option)&&(!temporal_out0_off_flag)) 
         {
            output_high(out_0);
            out_0_activated = true;
            write_ext_eeprom(out_0_activated_address, 's');
         }
         if((!out_0_activated)&&(out_0_timer>=out_0_time)&&(out0_option)&&(temporal_out0_off_flag)&&(temporal_out0_off_transition)) 
         {
            temporal_out0_off_transition = false;
            write_ext_eeprom(temporal_out0_off_transition_address, 'n');
            fprintf(syrus,"%s", out0_paused_once); // Se envia este comando en lugar de actvas la salida 0
         }
      }
      else // Entrada 2 flotando
      {
         output_low(out_0);    
         out_0_activated = false;
         write_ext_eeprom(out_0_activated_address, 'n');
         if(!temporal_out0_off_transition)
         {
            temporal_out0_off_flag = false;
            write_ext_eeprom(temporal_out0_off_flag_address, 'n');
            temporal_out0_off_transition = true;
            write_ext_eeprom(temporal_out0_off_transition_address, 's');
         }
         out_0_timer = 0;
      }
   }
}


void test_input_3() // Monitorea un cambio en el estado de la entrada 3, en este caso, el copiloto
{
   restart_wdt(); 
   if((input3_debouce_counter>=input3_debouce)&&(input3_enable==true))
   {     
      if(!input(in_3)) // Entrada 3 activa - Puerto copiloto abierta
      {
         input3_state = true;
      } 
      else // Puerto copiloto cerrada
      {
         input3_state = false;
         bpc2_timer = 0;
         bpc2_overtime_timer = 0;
         bpc2_overtime = false;
      }
      
      // Si hubo un cambio de estado en la entrada o una peticion de la misma
      if((poll_input3==1)||(input3_state!=last_input3_state)) 
      {
         //fprintf(syrus,"cambio entrada, %u,%u,%u\r\n", input2_state, last_input2_state, poll_input2);
        if(input3_state)
        {
            last_input3_state = true;
            write_ext_eeprom(in_3_address, 0xFF);
        }
        else
        {
            last_input3_state = false;
            write_ext_eeprom(in_3_address, 0x00);
        }

        if(input3_state) // Entrada  activada - Puerta de piloto abierta
        {
              if(!poll_input3) // Respuesta a cambio de estado
              {
                if(exe_monitor3_flag)
            {
               fprintf(syrus,"%s", msg_input3_act); // Envia mensaje de entrada activada
               lock_by_door(); // Bloquea por medio de voltaja analogico
            }
            else
            {
               fprintf(syrus,"%s", msg_input3_act_2); // Envia mensaje de entrada activada - Funcion de bloqueo no activada
            }

              }
              else // Respuesta a peticion
              {
                    if(exe_monitor1_flag)
               {
                  fprintf(syrus,"%s", msg_poll_input3_act); // Envia mensaje de entrada activada
               }
               else
               {
                  fprintf(syrus,"%s", msg_input3_act_2); // Envia mensaje de entrada activada - Funcion de bloqueo no activada
               }
              }
        }
        else  // entrada no activada - Puerta de piloto cerrada
        {
              if(!poll_input3) // Respuesta a cambio de estado
              {
                    if(exe_monitor1_flag)
               {
                  fprintf(syrus,"%s", msg_input3_deact); // Envia mensaje de entrada activada
               }
               else
               {
                  fprintf(syrus,"%s", msg_input3_deact_2); // Envia mensaje de entrada activada - Funcion de bloqueo no activada
               }
              }
              else // respuesta a peticion
              {
                    if(exe_monitor1_flag)
               {
                  fprintf(syrus,"%s", msg_poll_input3_deact); // Envia mensaje de entrada activada
               }
               else
               {
                  fprintf(syrus,"%s", msg_input3_deact_2); // Envia mensaje de entrada activada - Funcion de bloqueo no activada
               }
              }
        } 
        poll_input3 = false;      
      }      
      input3_debouce_counter = 0;
   }
}   

void test_input_4() // Monitorea un cambio en el estado de la entrada 4, en este caso la ignicion
{
   restart_wdt(); 
   if(input4_debouce_counter>=input4_debouce)
   {
     
      if(!input(in_4)) // Entrada 4 inactiva
      {
         input4_state = false;
      }
      else
      {
         input4_state = true;
      }
      
      // Si hubo un cambio de estado en la entrada o una peticion de la misma
      if((poll_input4_flag==1)||(input4_state!=last_input4_state)) 
      {
         //fprintf(syrus,"cambio entrada, %u,%u,%u\r\n", input2_state, last_input2_state, poll_input2);
         if(input4_state)
         {
            last_input4_state = true;
         }
         else
         {
            last_input4_state = false;
         }
         
   
         if(input4_state) // Entrada  activada - Motor encendido
         {
               // insertar en deteccion de encendido de motor
               //fprintf(syrus, "T. Motor apagado: %Lu / T. Backup: %Lu\r\n", rtc_engine_off, rtc_door_backup);
               if((bpp_flag_option)&&((rtc_engine_off-rtc_door_backup)>=(bpp_time)))
               {
                 bpp_flag = true;  
               }
               rtc_engine_off = 0; // Contador binario del tiempo transcurrido desde que se apago el motor
               rtc_door = 0; // Contador binario que indica el tiempo transcurrido desde que se apago el motor hasta que hubo un cambio en el estado de las puertas
               rtc_door_backup = 0;
               
               if(!poll_input4_flag)
               {
                  fprintf(syrus,"%s", msg_input4_act);
               }
               else
               {
                  fprintf(syrus,"%s", msg_poll_input4_act);
               }
         }
         else  // entrada no activada - Significa que apagaron el motor
         {
               if(!poll_input4_flag)
               {
                  fprintf(syrus,"%s", msg_input4_deact);
               }
               else
               {
                  fprintf(syrus,"%s", msg_poll_input4_deact);
               }
               
               if(input4_adc_enable) // habilita e inicia el contador para mandar el voltaje de bloqueo por apagado de motor
               {
                  enable_v3 = true;   
                  v3_timer = 0;
               }
         }
         poll_input4_flag = false;      
      }      
      input4_debouce_counter = 0;
   }
}   

void v3_by_ign_off() // Funcion que envio de voltaje de bloqueo por motor apagado
{
   restart_wdt(); 
   if((enable_v3)&&(v3_timer>=v3_time)) // si ya se apago el motor y paso el tiempo de retardo se activa el voltaje que mandara el bloqueo por apagado de motor
   {
      execute_v3();
      enable_v3 = false; // se cancela el reingreso hasta que vuelva a encenderse y apagarse la salida
   }
}

void disable_all_security() // Desabilita el sistema de seguridad del shield
{
   restart_wdt(); 
   if(disable_all_security_flag)
   {
      execute_v4();
      disable_all_security_flag = false;
   }
}

void unlock_cargolock()
{
   restart_wdt(); 
   fprintf(accesory, "%s",unblocking_box);
}

void lock_cargolock()
{
   restart_wdt(); 
   fprintf(accesory, "%s",blocking_box);   
}
//!void lock_cargolock()
//!{
//!   fprintf(accesory, "%s",blocking_box);
//!}

void send_cmd_00() // envia el comando para expandir el motor con el controlador
{
   /*   condicion 1: Se solicita por que se envio un comando MDT
      condicion 2: No ha habido respuesta del controlador y ademas ya se cumplio el tiempo minimo de espera programado
   */
   restart_wdt(); 
   if((send_lock_msg)||((response_lock_flag==false)&&(response_lock_timer>=response_lock_time)))
   {
      //fprintf(accesory,"\r\n%u,%u,%Lu,%Lu,%Lu\r\n", send_lock_msg, response_lock_flag,response_lock_timer,response_lock_time, lock_msg_counter);
      send_lock_msg = false;   // Indica que ya se envio el primero intento al controlador 
      lock_msg_running_flag = true; // Indica que se esta ejecutando el comando y sus respectivos intentos
      response_lock_flag = false; // bandera que determinara que el comando fue ejecutado
      response_lock_timer = 0; // Inicia el conteo del timer que limitara el tiempo que se procesara el comando
      lock_msg_counter++; // Contador de intentos para ejecutar el comando de bloqueo 
      //enable_interrupts(INT_RDA4); // Habilito interrupcion para escuchar al accesorio
      fprintf(accesory,"%s", blocking_box); // Comando que interpreta el controlador del motor
   }   

   /* Si se exedieron los intentos y no hubo respues de parte del controlador de motor se deja de enviar y avisa por puerto serial*/
   if((lock_msg_counter>=lock_max_msg_counter)&&(!response_lock_flag))
   {
      send_lock_msg = false; // no permite enviar nuevamente el comando de bloqueo
      response_lock_flag = true; // bandera que indica que ya se recibio la respuesta o que no llego y se acabaron las oportunidades
      lock_msg_counter = 0;
      lock_msg_running_flag = false;
      //disable_interrupts(INT_RDA4); // Como no llego respuesta, se deja de escuchar el puerto serial
      fprintf(syrus, "%s", lock_msg_error); // mensaje de error, no se confirmo que se haya ejecutado el comando
   }
}

void bpc1() // Bloqueo por corte puerta de piloto
{
   restart_wdt(); 
   if((bpc1_overtime_option)&&(test_copilot_door_flag))
   {
      if((bpc1_timer>=bpc1_time)&&(!bpc1_overtime)) // si ya pasaron mas de 15 minutos con la puerta abierta
      {
         //fprintf(syrus,"Puerta Piloto abierta por mas de X minutos\r\n");   
         bpc1_timer = 0;
         bpc1_overtime_timer = 0;
         bpc1_overtime = true;
      }

      if((bpc1_overtime)&&(bpc1_overtime_timer>=bpc1_overtime_time))
      {
         bpc1_overtime_timer = 0;
         fprintf(syrus,"%s", bpc1_send_msg);
         if(exe_bpc1_flag)
         {
            delay_ms(1500);
            lock_by_door();
         }
      }
   }
}

void bpc2() // Bloqueo por corte puerta de copiloto
{
   restart_wdt(); 
   if((bpc2_overtime_option)&&(input3_enable))
   {
      if((bpc2_timer>=bpc2_time)&&(!bpc2_overtime)) // si ya pasaron mas de 15 minutos con la puerta abierta
      {
         //fprintf(syrus,"Puerta Copiloto abierta X minutos\r\n");
         bpc2_timer = 0;
         bpc2_overtime_timer = 0;
         bpc2_overtime = true;
      }

      if((bpc2_overtime)&&(bpc2_overtime_timer>=bpc2_overtime_time))
      {
         bpc2_overtime_timer = 0;
         fprintf(syrus,"%s", bpc2_send_msg);
         if(exe_bpc2_flag)
         {
            delay_ms(1500);
            lock_by_door();
         }
      }
   }
}

void bpp()
{
   restart_wdt(); 
   if((bpp_flag)&&(test_copilot_door_flag))
   {
      //fprintf(syrus,"No ha habido movimiento en puertas\r\n");
      fprintf(syrus,"%s", bpp_send_msg);
      bpp_flag = false;
      if(exe_bpp_flag)
      {
         delay_ms(1500);
         lock_by_door();
      }
   }
}

void map_eeprom()
{
//!   int16 max_address = 256;
//!   int16 count_address;
//!   int16 map_data;
//!   int32 CRC;
   
   restart_wdt(); 
//!   fprintf(syrus,"%Lu\r\n", read_int16_ext_eeprom(password_1_address));
//!   fprintf(syrus,"%Lu\r\n", read_int16_ext_eeprom(password_2_address));
//!   fprintf(syrus,"%Lu\r\n", read_int16_ext_eeprom(password_3_address));
//!   fprintf(syrus,"%Lu\r\n", read_int16_ext_eeprom(password_4_address));

}

void acc_monitor()
{
   restart_wdt(); 
   if(acc_detected)
   {
      if(acc_connected_timer>=acc_connected_time)
      {
         acc_connected_timer = 0;
         acc_lost_counter++;
         fprintf(accesory,"*%c%c#", 0x02,0x0A);
         //fprintf(syrus,"Int++\r\n");
      }
           
      if((acc_lost_max_mgs_counter>=acc_lost_max_mgs_count)&&(acc_lost_long_msg_timer>=acc_lost_long_msg_time))
      {
         acc_lost_long_msg_timer = 0;
         write_int16_ext_eeprom(acc_lost_long_msg_timer_address, acc_lost_long_msg_timer);
         fprintf(syrus,"%s", acc_0_disconnected);
      }
      
      if((acc_lost_max_mgs_counter<acc_lost_max_mgs_count)&&(acc_lost_counter>=acc_lost_max_counter))
      {
         acc_lost_counter = 0;
         acc_lost_long_msg_timer = 0;
         acc_lost_max_mgs_counter++;
         write_int16_ext_eeprom(acc_lost_max_mgs_counter_address, acc_lost_max_mgs_counter);
         fprintf(syrus,"%s", acc_0_disconnected);
         //fprintf(accesory,"*rst#");
      }
   }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////        PROGRAMA PRINCIPAL           /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////


void main()
{
   setup_timer_0(T0_INTERNAL|RTCC_DIV_128);      //1.0 s overflow
   setup_timer_1(T1_INTERNAL|T1_DIV_BY_8);      //32.7 ms overflow
   setup_timer_2(T2_DIV_BY_16,100,16);      // 1 ms overflow
   //fprintf(syrus,"START\r\n");
   delay_ms(2010);
   restart_wdt();
   map_eeprom();
   init_pinlock();
   //fprintf(syrus,"\r\n%Lu,%Lu,%Lu,%Lu,%Lu\r\n",bpc1_time,bpc1_overtime_time,bpc2_time,bpc2_overtime_time,bpp_time);
   display_number = 2019;
   turn_off_7seg();
   
   while(true)
   {
      //fprintf(syrus,"Entrada: %u,%05Lu\r", input(in_1), debounce_copilot_door);
      
      // Monitorea contantemente si alguno de los botones ha sido presionado
      test_keypad();
         
      // Monitorea si hay comandos en el buffer del puerto serial para ser analizados
      scan_cmd();
      
      // Monitorea si hay comandos en el buffer del puerto serial del accesorio del pinlock
      scan_cmd2();
      
      // Si hay un dato nuevo para grabar en la eeprom realiza la accion correspondiente
      //update_eeprom();
      
      // Monitorea el estado de la entrada 0
      test_input();
      
      // Monitorea si se intenta usar una contraseÃ±a
      test_ok();
      
      // Si se ingresaron erroneamente las contraseÃ±as se bloquea el teclado
      display_block_timer();
      
      // Monitorea si ya es tiempo de reinciar el programa
      system_reset();
      
      // Monitorea si el contador para el reset de intentos por contraseÃ±as erroneas ya se cumplio
      system_reset_tries();
      
      // Monitorea si no hay actividad por mas de 15 segundos para apagar la pantalla
      system_display_sleep();
      
      // Funcion que monitorea el envio del  voltaje de bloqueo por motor apagado
      v3_by_ign_off(); 
      
      // Funcion que deshabilita el sistema de seguridad por completo
      disable_all_security();
      
      // Monitorea el estado de la puerta del piloto, si se habre se activa el paro al acelerador
      test_input_1();
      
      // Verifica si la entrada 2 fue activada - Retro del tamper
      test_input_2();
      
      // Verifica si la entrada 3 fue activada - Copiloto
      test_input_3();
      
      // Verifica si la entrada 4 fue activada - ignicion
      test_input_4();
      
      // funcion para verificar si la puerta del piloto quedo abierta por mas de 15 minutos
      bpc1();
      
      // funcion para verificar si la puerta del copiloto quedo abierta por mas de 15 minutos
      bpc2();
      
      // funcion para verificar si la puerta del piloto no ha tenido cambios aun y cuando el motor se ha estado encendiendo
      bpp();
      
      //send_cmd_00(); // Envia si se  cumplen las condiciones el mensaje para activar la expansion del motor
      
      // Monitorea si el accesorio sigue conectado
      acc_monitor();
   }
}


