#ifndef _Directives_h
 #define _Directives_h
 #include <Arduino.h> //needed

                                  // LED_BUILTIN;?
                                  // 2 is default on //GPIO 02 SignalLed definition onboard led on most esp8266 and esp32
                                  //led is 22 for lolin lite? clashes with audio! 
                                 //5 on d32 board (screws up I2C display!) = 
                                 // LED_BUILTIN = 2??; 
 #define SignalLed 2 // Revised definition in V5 this is now the PIN /GPIO number not D[0]. Is used (0nly) in SignOfLifeFlash 
// #define SignalLed 16 // for red led on esp8266  ???
 #define SignalON LOW  //defined so I can change the "phase of the SignalLED" easily.
 #define SignalOFF HIGH

 #define MinWiFi -85 // min (negative) wi fi signal for various options
 #define _Scan_for_Mosquitto  // New define to switch on scanning for mosquitto. Useful if your router does not have the IP address of the mosquitto server reserved!.

 // -----------------------------------------------------------------------------------------------------------
// NB -- Loco use is incompatble with OLED - so the OLED define is later, and switched off if PWM is set. ----
//------------------------------------------------------------------------------------------------------------
//#define _LOCO_SERVO_Driven_Port 1  // D1 if using as mobile (LOCO) decoder node.. node becomes a loco with servo on port D "1"  for motor control
//#define _LocoPWMDirPort  3         // D3 add this second Port if using PWM loco motor control Using "3" assumes a L293 inputs driven from port D(_LOCO_SERVO_Driven_Port) and D(_LocoPWMDirPort)
                                     //DO Notuse other ports unless you change the settings in detachservos
 

//IF PWM, which sort of PWM driver??

//#define _NodeMCUMotorShield      //NodeMCU Motor Shield has inverters to the main H drives and so pin D1 is the PWM  and D3 is the PWM direction
                                   //HOWEVER MY boards all exhibit strange behaviour after eeprom programming, when motor will only move one way.



//#define _DefaultPrintOut 1 //Set this so you can capture your own "defaults" and put them in the RocSubs:SetDefaultSVs() function 


 //#define _Use_Wifi_Manager //uncomment this to use a "standard" fixed SSID and Password
 
 
 //----------------DEBUG Defines 
 ////Debug settings
 // uncomment these to add extra debug messages. - useful after modifyng the code and something unexpected happens.. 
 // a minimum number of Mqtt debug message will always be set to allow monitoring of node status 
 //(typically the time synch is very useful to tell a node is prenet and working.) 
//#define _NoSend_Time_Synch_Debug
//#define _LoopTiming
//#define _SERIAL_DEBUG 
//#define _Input_DEBUG
//#define _SERIAL_Audio_DEBUG 
//#define _SERIAL_SUBS_DEBUG 
//#define _ROCDISP_SUBS_DEBUG
//#define _ROCDISP_EEPROM_DEBUG
//#define _showRocMSGS  // time synch?
//#define _SERIAL_MQTT_DEBUG
//#define _SERVO_DEBUG //(enables debug messages in set motor speed rc and pwm
//#define _PWM_DEBUG  // to assist pwm debug 
//#define _SpeedTableDEBUG 
//#define _ESP32_PWM_DEBUG  // to assist pwm debug 
//#define _ESP32_HallTest
// #define FTP_DEBUG in \libraries\esp8266FTPserver\ESP8266FtpServer.h to see ftp verbose on serial

  //ESP32 Stuff

  #ifdef ESP32
    //used in subroutines.h see  https://desire.giesecke.tk/index.php/2018/07/06/reserved-gpios/ 
    // note for GPIO pin 12 Make sure it is not pulled high by a peripheral device during boot or the module might not be able to start!
    //21,22 for audio (Audio uses I2C defaults SPI 21,22) 
    // inputs only with external pullup? 34 35 36 37 38 39 note GPIO34 ... GPIO39 do not have pu/pd circuits.
    // NO USE SPIO flash 6 7 8 9 10 (11?)  
    // ?? not working as expected             18(VSPI CLK)
    //                                                          25,26 default to dac mode give square wave, 11 khz 50% DAC!
    // following work fine  12 13 14 15 16 17    19   (21!) 23       27     32 33 
    // not exposed on my board / untested           20        24       28 29     36 37 38 39

    // also note from ESP32 servo library // quote "All pin numbers are allowed (for PWM), but only pins 2,4,12-19,21-23,25-27,32-33 are recommended.

  static const  uint8_t D0   = 2;  // V5 'D0' is not used now, used to be special pin identifier for signal led. 
  static const  uint8_t D1   = 19;//
  static const  uint8_t D2   = 12; // boot will fail if pulled high
  static const  uint8_t D3   = 13; //
  static const  uint8_t D4   = 14;  
  static const  uint8_t D5   = 15;
  static const  uint8_t D6   = 16;
  static const  uint8_t D7   = 17;
  static const  uint8_t D8   = 21; 
  static const  uint8_t D9   = 23; 
  static const  uint8_t D10   = 25;  
  static const  uint8_t  D11  = 26; 
  #define DAC25is  10
  #define DAC26is  11 //to keep port setting in rocsubs happy should not affect anything else ports D10 and D11
  static const  uint8_t  D12 = 27; 
  static const  uint8_t  D13  = 32; 
  static const  uint8_t  D14  = 33; 
  // these are input only
  static const  uint8_t  D15  = 34; 
  static const  uint8_t  D16  = 35; 
   // on oled board, 4 is oled scl 5 is oled sda for oled 

  #endif




#endif
