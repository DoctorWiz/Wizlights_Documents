;; filename: Renard Plus All in one firmware.asm
;; Orignal filename:	ren16_g2-2009731.asm
;; Copyright (c) 2006,2007,2009 Phil Short
;;
;;  This program is provided free for you to use in any way that you wish,
;;  subject to the laws and regulations where you are using it.  Due diligence
;;  is strongly suggested before using this code.
;;
;;  The Author makes no warranty of any kind, express or implied, with regard
;;  to this program or the documentation contained in this document.  The
;;  Author shall not be liable in any event for incidental or consequential
;;  damages in connection with, or arising out of, the furnishing, performance
;;  or use of these programs.
;;
;;  --------------------------------------------------------------------------
;;
;;  Note: there are some parameters that may be modified by the user.  These
;;    are located just below the list of processors that the firmware is
;;    intended to work with.
;;
;;  4 Jan 2009 PJS Taken as basis for 16-channel version coded up for the
;;     PIC16F722 microcntroller.
;;  11 Jan 2009 PJS Fixed bug so that it now recovers from frame error, added
;;     code for using 16-bit baudrate generator of PIC18 family, added code to
;;     to make the ZC LED to follow the ZC hardware input when that bit is not
;;     toggling (i.e. LED is on when input is open, LED is off when input is
;;     shorted to ground, flashes otherwise with a 5 second (approx) period.
;;  12 Jan 2009 PR Modified scheme for adjusting settings for different CPUs
;;  12 Jan 2009 PJS Fixed settings for PIC18F2221
;;  13 Jan 2009 PJS Removed portions of revision history that related to the
;;     old Renard firmware (PIC16F688-specific).  Added lots of macros to make
;;     it a little easier to re-map the outputs.  These macros will probably
;;     be removed from this file and placed in a separate include file so that
;;     the user doesn't have to re-enter them when a firmware update is issued.
;;  15 Jan 2009 PJS Moved LED from bit 7 of porta to bit 6 of port b, and
;;     tri-stated the former bit.
;;  17 Jan 2009 PJS Fixed problem causing channel 5 to ignore input settings
;;      (and to appear to cyclically dim), and fixed configuration error for
;;	16F722.
;;  28 April 2009 PJS Modified for pinout used on home-etch/SMD  board.  The
;;	VCAP  pin was moved, the LED logic was modified to use only three pins
;;      for four LEDs ('charlie-plexed'), and the triac output pins were moved
;;	around.  Lots  of little modifications to try and make things faster
;;      to compensate for extra charlie-plexing logic in the ISR.
;;  16 May 2009 PJS Split off the local customization file
;;  20 May 2009 PJS Made some fixes for PIC18F family (suggested by pr).
;;  27 May 2010 PJS Fixes for PIC18F family -
;;     - moved code for clearing PIE1, PIE2, and IPEN (previous location of
;;       that code prevented timer2 interrupts from occurring).
;;     - changed a few instances of '0x80' to BANK1 (previous code would access
;;       the wrong addresses).  NOTE that changes to ren16_g2_local.inc is also
;;       required, for the same reason).
;;  28 may 2010 PR Minor changes to the processor config sections. Added 2620 as
;;       per Mac's (Phoenix) recommendations.
;;  10 Aug 2010 Mac (Phoenix) added PSP Mode for PortE cofig errors.
;;  01 Oct 2010 Mac (Phoenix) changed Clockrate for PIC18F45K22 Device for 64Mhz 
;;      and  Heartbeat location moved, prior to that change back in August 2010
;;      PSP Mode added for PortE cofig errors.;;
;;  06 March 2011 Mac (Phoenix) changed Clockrate to 32Mhz due to poor dimming issues, needed to
;;       remove three status LEDs in order to accommodate 32 channels, there is now only
;;       one status LED that blinks at three different levels of intensities at about every 
;;       three seconds.
;;  22 April 2011 PJS added Start Address code for Ren-W wirelees capabilities
;;  25 April 2011 Mac (Phoenix) added Defines to avoid two sets of code 
;;	   for Start Address and non Start Address firmware
;;  05 August 2012 A. Williams (LabRat) added DMX code
;;  10 Oct 2013 A. Williams (LabRat) corrected DMX lag issue
;;  19 Dec 2013 Mac (Phoenix) combined .inc file with .asm files
;;  16 April 2015 Charles Kerr added DMX Lights out function when data signal is lost 
;;     between Controller and sequenced data stream.
;;  22 April 2015 Mac (Phoenix) fixed intermittant loss of dimming control.
;;  10 August 2016 Mac (Phoenix) Merged Renard, Renard Start Addressing, DMX and
;;     added PIC18F4xk22 PIC onto a single .asm file.
;;
;;  ******* dedicated port assignments ********
;;
;;  PIN 1  (RE0) - zero-crossing (input only)
;;  PIN 17 (RC6/TX) - uart_out (output)
;;  PIN 18 (RC7/RX) - uart_in (input)

;;  Outputs are good to drive 15 mA opto-isolators, except this will probably
;;  overload the VSS/VDD pins on the PIC.  So the current through each output
;;  should be limited to 6 mA (except the LED pins).
;;
;;  This code fits in less than 2K of program space, and takes advantage of
;;  that fact at various places to save a few bytes of code, and so it may
;;  not necessarily work as intended if the code size is increased.
;; 
;;  This code does not make use of automatic baud-rate detection, nor does
;;  it use timer1, the low-power, wake-up, analog comparator or A/D
;;  capabilities of the chip.
;;
;;  The interrupt used by this program is timer2, which is intended to create
;;  a periodic (32 us) clock.  Since the timer is reset inside of the ISR, the
;;  foreground routine should disable interrupts for as short a time as
;;  possible (or not at all, if this can be accomplished).

 LIST R=DEC
 
;;
;; {pr}
;; added Define statements for each processor that has been tested
;; The current list is:
;; 16f722   <- best cost
;; 18f2525  <- Most memory for future expansion
;; 18f2420  
;;
;; Tested 2012
;; 18f2221  <- Cheapest 18f family. assuming has 16bit uart
;;             Currently has config bit errors as it was a copy of the 2525
;;             settings will check settings once chips arrive

;;  --------------------------------------------------------------------------
;;
;;  You might want to change this line if you need different versions of the
;;    include file for any reason.
	
;;  This file contains the definitions that the user might reasonably want to
;;  change.  This includes the I/O pin mappings, the baud rate, and some
;;  clock-related items. These files were created by Phil Short, Peter Rogers
;;  and modified several times to accommodate the Renard Plus Controllers 
;;  designed by Mac Macmillan (Phoenix)
;;
;;  2) Processor selection (located as a menu item in the MPLAB IDE).
;;
;;  Hardware mappings
;;  PIN 1  (RE0) - zero-crossing (input only)
;;  PIN 2  (RA0) - triac driver 1 (output)
;;  PIN 3  (RA1) - triac driver 2 (output)
;;  PIN 4  (RA2) - triac driver 3 (output)
;;  PIN 5  (RA3) - triac driver 4 (output)
;;  PIN 6  (RA4) - triac driver 5 (output)
;;  PIN 7  (RA5) - VCAP
;;  PIN 10 (RA6) - spare - reserved for possible oscillator use
;;  PIN 9  (RA7) - spare - reserved for possible oscillator use
;;  PIN 11 (RC0) - triac driver 6 (output)
;;  PIN 12 (RC1) - triac driver 7 (output)
;;  PIN 13 (RC2) - triac driver 8 (output)
;;  PIN 14 (RC3) - LED driver (anode for RxD, cathode for FrameErr)
;;  PIN 15 (RC4) - LED driver (anode for HB, cathode for ZC)
;;  PIN 16 (RC5) - LED driver (anode for ZC, RxD, cathode for HB and FrameErr)
;;  PIN 17 (RC6/TX) - uart_out (output)
;;  PIN 18 (RC7/RX) - uart_in (input)



;;  NOTE:
;;
;;  This firmware is configured for PWM use with up to 460800 baud communications.
;;  Here are the DEFINE statements for changing this.
;;
;;  **********User tunable items*********

;;************** DMX MODE ***************
;;#DEFINE DMX  ;; ******uncomment for DMX******
#ifdef DMX
  #DEFINE BAUDRATE 250000

;; Uncomment to hard code the DMX address. Otherwise it will be read
;; from the EEROM locations
;; #DEFINE HARD_CODE_DMX_ADDR  ;;only needed for devices without EEPROM

  #DEFINE DMX_START_ADDRESS D'001'  ;;change for DMX starting channel in Vixen 1-512
; -- EE Address Mappings --
  #DEFINE EE_DMX_HIGH		(0x00)	
  #DEFINE EE_DMX_LOW		(0x01)
	
#else

;; Define Baud Rates. uncomment one of the following Baud Rates
 #DEFINE BAUDRATE 57600
;; #DEFINE BAUDRATE 115200
;; #DEFINE BAUDRATE 230400

;;************* Renard/Renard Start Address Mode ******************
;; Start address for channel start configuration on controller
;; also required for Ren-W wireless operation,, max baud rate is 57600

;;  #DEFINE START_ADDRESS  ;;******comment this line out for Standard Renard Protocol

;;  #DEFINE START_ADDR 0 ;; 0 starts channels 1-8, see Start Address Chart for
;; Renard Plus or Simple Renards (not the same as SS Renard boards)

#endif

;; CTR_LOCKOUT turns all outputs off early if it is > 0.

 #DEFINE CTR_LOCKOUT 0

;;**************** Internal OSC settings *********************
;;********************* Do Not Change *************************

 #DEFINE CLOCKRATE 32000000
;; #DEFINE CLOCKRATE 64000000  ;;Future plans for PIC18F25K22
 
;; All PIC18F parts currently require '1<<PLLEN' here.

 #DEFINE OSC_TUNE_LOCAL 1<<PLLEN ;; {pr} fixed typo

 #define ZC_TWEAK 15

#define DMX_BLINK_RATE 0x2C


;; I/O bit definitions

 #define ZC_BIT   3
 #define ZC_REG	  PORTE
 #define ZC       ZC_REG,(ZC_BIT)
 #define ZC_MASK  (1<<(ZC_BIT))

;; The rationale for these bit assignments is given in a short section at the
;; end of the main .asm file.
;;
;; valid bits 0,1,3,5
;; extra led's to phil board are 4,7 (7 = switched led)
;;
	
 #define HEARTBEAT_LED_BIT  0
 #define HEARTBEAT_LED_REG  LED_map
 #define HEARTBEAT_LED	HEARTBEAT_LED_REG, HEARTBEAT_LED_BIT

 #define ZC_LED_BIT 5
 #define ZC_LED_REG LED_map
 #define ZC_LED ZC_LED_REG,ZC_LED_BIT
	
 #define FRAME_ERR_LED_BIT  1
 #define FRAME_ERR_LED_REG  LED_map
 #define FRAME_ERR_LED  FRAME_ERR_LED_REG, FRAME_ERR_LED_BIT
	
 #define RXD_LED_BIT 3
 #define RXD_LED_REG LED_map
 #define RXD_LED RXD_LED_REG, RXD_LED_BIT

;; TEST_STROBE is used during the debug phase to create a signal that can be
;;   examined with an oscilloscope.
;; The following pin is currently tristated...so TEST_STROBE is disabled.
;; #define TEST_STROBE PORTA,4

;;  --------------------------------------------------------------------------
;;  Macros for assigning dimmer input channels to IO pins.
;;
;;  The serial communicationss and zero-crossing pins are pre-assigned, and are
;;    not intended to be easily changed.
;;
;;  This is also where output polarity and PWM vs non-PWM status is assigned.
;;  If the SINK_PORTx_CHy bit is set to '1' the signal will be active low, if
;;    it is set '0' the output signal will be active high.  Similarly, if the
;;    PORTx_CHy bit is set to '1' the output will be a PWM signal, if set
;;    to '0' the output will be just a pulse (although this latter option only
;;    makes sense when driving a TRIAC or SCR).

;;  Note: the pins used for LED output must have the SINK bit in the 'OFF'
;;    state and the PWM pin in the 'ON' state.

#IFNDEF YES
  #DEFINE YES 1
#ENDIF
#IFNDEF NO
  #DEFINE NO 0
#ENDIF
	
;;  PortA, bit 0 has channel 1 mapped to it
  #DEFINE PIN2_CH 1
  #DEFINE PIN2_SINK YES
  #DEFINE PIN2_PWM YES
	
;;  PortA, bit 1 has channel 2 mapped to it
  #DEFINE PIN3_CH 2
  #DEFINE PIN3_SINK YES
  #DEFINE PIN3_PWM YES
	
;;  PortA, bit 2 has channel 3 mapped to it
  #DEFINE PIN4_CH 3
  #DEFINE PIN4_SINK YES
  #DEFINE PIN4_PWM YES
	
;;  PortA, bit 3 has channel 4 mapped to it
  #DEFINE PIN5_CH 4
  #DEFINE PIN5_SINK YES
  #DEFINE PIN5_PWM YES

;;  PortA, bit 4 has channel 5 mapped to it
  #DEFINE PIN6_CH 5
  #DEFINE PIN6_SINK YES
  #DEFINE PIN6_PWM YES

;;  PortA, bit 5 is used for VCAP
  #DEFINE PIN7_SINK NO
  #DEFINE PIN7_PWM YES

;;  PortA, bit 6 has nothing mapped to it - reserved for possible oscillator
  #DEFINE PIN10_SINK NO
  #DEFINE PIN10_PWM YES

;;  PortA, bit 7 has nothing mapped to it - reserved for possible oscillator
  #DEFINE PIN9_SINK NO
  #DEFINE PIN9_PWM YES

;; ---------------------------- PORT B mapping -------------------------------

;;  PortB, bit 0 has channel 13 mapped to it
;;  #DEFINE PIN21_CH 13
  #DEFINE PIN21_SINK YES
  #DEFINE PIN21_PWM YES

;;  PortB, bit 1 has channel 14 mapped to it
;;  #DEFINE PIN22_CH 14
  #DEFINE PIN22_SINK YES
  #DEFINE PIN22_PWM YES

;;  PortB, bit 2 has channel 15 mapped to it
;;  #DEFINE PIN23_CH 15
  #DEFINE PIN23_SINK YES
  #DEFINE PIN23_PWM YES

;;  PortB, bit 3 has channel 16 mapped to it
;;  #DEFINE PIN24_CH 16
  #DEFINE PIN24_SINK YES
  #DEFINE PIN24_PWM YES

;;  PortB, bit 4 has channel 9 mapped to it
;;  #DEFINE PIN25_CH 9
  #DEFINE PIN25_SINK YES
  #DEFINE PIN25_PWM YES

;;  PortB, bit 5 has channel 10 mapped to it
;;  #DEFINE PIN26_CH 10
  #DEFINE PIN26_SINK YES
  #DEFINE PIN26_PWM YES

;;  PortB, bit 6 has channel 11 mapped to it
;;  #DEFINE PIN27_CH 11
  #DEFINE PIN27_SINK YES
  #DEFINE PIN27_PWM YES

;;  PortB, bit 7 has channel 12 mapped to it
;;  #DEFINE PIN28_CH 12
  #DEFINE PIN28_SINK YES
  #DEFINE PIN28_PWM YES

;; ---------------------------- PORT C mapping -------------------------------

;;  PortC, bit 0 has channel 6 mapped to it
  #DEFINE PIN11_CH 6
  #DEFINE PIN11_SINK YES
  #DEFINE PIN11_PWM YES

;;  PortC, bit 1 has channel 7 mapped to it
  #DEFINE PIN12_CH 7
  #DEFINE PIN12_SINK YES
  #DEFINE PIN12_PWM YES

;;  PortC, bit 2 has channel 8 mapped to it
  #DEFINE PIN13_CH 8
  #DEFINE PIN13_SINK YES
  #DEFINE PIN13_PWM YES

;;  PortC, bit 3 has LED driver mapped to it
  #DEFINE PIN14_SINK NO
  #DEFINE PIN14_PWM YES
  #DEFINE LED0_TRIS TRISC^BANK1,3
  #DEFINE LED0_anode_sel portc_image,3
			
;;  PortC, bit 4 has LED driver mapped to it
  #DEFINE PIN15_SINK NO
  #DEFINE PIN15_PWM YES
  #DEFINE LED1_TRIS TRISC^BANK1,4
  #DEFINE LED1_anode_sel portc_image,4

;;  PortC, bit 5 has LED driver mapped to it
  #DEFINE PIN16_SINK NO
  #DEFINE PIN16_PWM YES
  #DEFINE LED2_TRIS TRISC^BANK1,5
  #DEFINE LED2_anode_sel portc_image,5

;;  PortC, bits 6 and 7 are used for serial (UART) I/O
  #DEFINE PIN17_SINK NO
  #DEFINE PIN18_SINK NO

  	

;; ----------------------------------------------------------------------------
;; The following definitions probably won't need to be changed by the user.

  #IFDEF PIN2_CH
    #DEFINE PIN2_DIMMER (1<<0)
  #ELSE
    #DEFINE PIN2_DIMMER 0
  #ENDIF
  #IFDEF PIN3_CH
    #DEFINE PIN3_DIMMER (1<<1)
  #ELSE
    #DEFINE PIN3_DIMMER 0
  #ENDIF
  #IFDEF PIN4_CH
    #DEFINE PIN4_DIMMER (1<<2)
  #ELSE
    #DEFINE PIN4_DIMMER 0
  #ENDIF
  #IFDEF PIN5_CH
    #DEFINE PIN5_DIMMER (1<<3)
  #ELSE
    #DEFINE PIN5_DIMMER 0
  #ENDIF
  #IFDEF PIN6_CH
    #DEFINE PIN6_DIMMER (1<<4)
  #ELSE
    #DEFINE PIN6_DIMMER 0
  #ENDIF
  #IFDEF PIN7_CH
    #DEFINE PIN7_DIMMER (1<<5)
  #ELSE
    #DEFINE PIN7_DIMMER 0
  #ENDIF
  #IFDEF PIN10_CH
    #DEFINE PIN10_DIMMER (1<<6)
  #ELSE
    #DEFINE PIN10_DIMMER 0
  #ENDIF
  #IFDEF PIN9_CH
    #DEFINE PIN9_DIMMER (1<<7)
  #ELSE
    #DEFINE PIN9_DIMMER 0
  #ENDIF
	
  #IFDEF PIN21_CH
    #DEFINE PIN21_DIMMER (1<<0)
  #ELSE
    #DEFINE PIN21_DIMMER 0
  #ENDIF
  #IFDEF PIN22_CH
    #DEFINE PIN22_DIMMER (1<<1)
  #ELSE
    #DEFINE PIN22_DIMMER 0
  #ENDIF
  #IFDEF PIN23_CH
    #DEFINE PIN23_DIMMER (1<<2)
  #ELSE
    #DEFINE PIN23_DIMMER 0
  #ENDIF
  #IFDEF PIN24_CH
    #DEFINE PIN24_DIMMER (1<<3)
  #ELSE
    #DEFINE PIN24_DIMMER 0
  #ENDIF
  #IFDEF PIN25_CH
    #DEFINE PIN25_DIMMER (1<<4)
  #ELSE
    #DEFINE PIN25_DIMMER 0
  #ENDIF
  #IFDEF PIN26_CH
    #DEFINE PIN26_DIMMER (1<<5)
  #ELSE
    #DEFINE PIN26_DIMMER 0
  #ENDIF
  #IFDEF PIN27_CH
    #DEFINE PIN27_DIMMER (1<<6)
  #ELSE
    #DEFINE PIN27_DIMMER 0
  #ENDIF
  #IFDEF PIN28_CH
    #DEFINE PIN28_DIMMER (1<<7)
  #ELSE
    #DEFINE PIN28_DIMMER 0
  #ENDIF	

  #IFDEF PIN11_CH
    #DEFINE PIN11_DIMMER (1<<0)
  #ELSE
    #DEFINE PIN11_DIMMER 0
  #ENDIF
  #IFDEF PIN12_CH
    #DEFINE PIN12_DIMMER (1<<1)
  #ELSE
    #DEFINE PIN12_DIMMER 0
  #ENDIF
  #IFDEF PIN13_CH
    #DEFINE PIN13_DIMMER (1<<2)
  #ELSE
    #DEFINE PIN13_DIMMER 0
  #ENDIF
  #IFDEF PIN14_CH
    #DEFINE PIN14_DIMMER (1<<3)
  #ELSE
    #DEFINE PIN14_DIMMER 0
  #ENDIF
  #IFDEF PIN15_CH
    #DEFINE PIN15_DIMMER (1<<4)
  #ELSE
    #DEFINE PIN15_DIMMER 0
  #ENDIF
  #IFDEF PIN16_CH
    #DEFINE PIN16_DIMMER (1<<5)
  #ELSE
    #DEFINE PIN16_DIMMER 0
  #ENDIF
  #IFDEF PIN17_CH
    #DEFINE PIN17_DIMMER (1<<6)
  #ELSE
    #DEFINE PIN17_DIMMER 0
  #ENDIF
  #IFDEF PIN18_CH
    #DEFINE PIN18_DIMMER (1<<7)
  #ELSE
    #DEFINE PIN18_DIMMER 0
  #ENDIF

;; ----- Port A

  #DEFINE DIMMER_BITMAP_PORTA (PIN2_DIMMER | PIN3_DIMMER | PIN4_DIMMER | PIN5_DIMMER | PIN6_DIMMER | PIN7_DIMMER | PIN10_DIMMER | PIN9_DIMMER)

  #DEFINE SINK_MAP_PORTA (((((((PIN9_SINK * 2 + PIN10_SINK) * 2 + PIN7_SINK) * 2 + PIN6_SINK) * 2 + PIN5_SINK) * 2 + PIN4_SINK) * 2 + PIN3_SINK) * 2 + PIN2_SINK)

  #DEFINE PWM_MAP_PORTA ((((((((PIN9_PWM*2+PIN10_PWM)*2+PIN7_PWM)*2+PIN6_PWM)*2+PIN5_PWM)*2+PIN4_PWM)*2+PIN3_PWM)*2+PIN2_PWM))

;; ----- Port B
	
  #DEFINE DIMMER_BITMAP_PORTB (PIN21_DIMMER | PIN22_DIMMER | PIN23_DIMMER | PIN24_DIMMER | PIN25_DIMMER | PIN26_DIMMER | PIN27_DIMMER | PIN28_DIMMER)

  #DEFINE SINK_MAP_PORTB (((((((PIN28_SINK * 2 + PIN27_SINK) * 2 + PIN26_SINK) * 2 + PIN25_SINK) * 2 + PIN24_SINK) * 2 + PIN23_SINK) * 2 + PIN22_SINK) * 2 + PIN21_SINK)

  #DEFINE PWM_MAP_PORTB ((((((((PIN28_PWM*2+PIN27_PWM)*2+PIN26_PWM)*2+PIN25_PWM)*2+PIN24_PWM)*2+PIN23_PWM)*2+PIN22_PWM)*2+PIN21_PWM))

;; ----- Port C
	
  #DEFINE DIMMER_BITMAP_PORTC (PIN11_DIMMER | PIN12_DIMMER | PIN13_DIMMER | PIN14_DIMMER | PIN15_DIMMER | PIN16_DIMMER | PIN17_DIMMER | PIN18_DIMMER)

  #DEFINE SINK_MAP_PORTC (((((((PIN18_SINK * 2 + PIN17_SINK) * 2 + PIN16_SINK) * 2 + PIN15_SINK) * 2 + PIN14_SINK) * 2 + PIN13_SINK) * 2 + PIN12_SINK) * 2 + PIN11_SINK)

  #DEFINE PWM_MAP_PORTC (((((((PIN16_PWM)*2+PIN15_PWM)*2+PIN14_PWM)*2+PIN13_PWM)*2+PIN12_PWM)*2+PIN11_PWM))
	
;; Processor Configuration Directives - specific to each processor.  The actual
;;   processor is selected in the MPLAB IDE Configuration menu item.  The
;;   symbol 'OSC_CONTROL_LOCAL' is defined in ren16_local.inc.
	

 #IFDEF __18F25K22

 INCLUDE "p18f25K22.inc"
 #DEFINE PIC18_FAMILY

 #define OSC_CONTROL_LOCAL 0x60
;; #define OSC_CONTROL_LOCAL 0x70

	CONFIG PRICLKEN=ON, FCMEN=OFF,IESO=OFF,PLLCFG=ON, FOSC=INTIO67
;; wdt off, brownout is on and set for second highest voltage (hardware only)
	CONFIG PWRTEN=ON, BOREN=NOSLP, BORV=285

	CONFIG WDTEN=OFF,WDTPS=1
;; mclre disabled, timer1 cfg fow low power, portb pins are digital, ccp2 mux
	CONFIG MCLRE=INTMCLR,PBADEN=OFF,CCP2MX=PORTC1  ;;,CCP3MX = PORTB5,T3CMX = PORTB5

;; debug, extended_instruction, low-voltage programming and reset_stack_err clr
	CONFIG DEBUG=OFF,XINST=OFF,LVP=OFF,STVREN=ON,HFOFST = OFF

;; code protect some blocks (program memory and eeprom)
	CONFIG CP0=OFF,CP1=OFF
	CONFIG CPB=OFF,CPD=OFF

;; write protect some blocks (program memory and eeprom)
	CONFIG WRT0=OFF,WRT1=OFF
	CONFIG WRTB=OFF,WRTC=OFF,WRTD=OFF

;; following two words protect the various program code blocks from table reads
	CONFIG EBTR0=OFF,EBTR1=OFF
	CONFIG EBTRB=OFF

 #ENDIF


 #IFDEF __18F2525

 INCLUDE "p18f2525.inc"
 #DEFINE PIC18_FAMILY

;; #define OSC_CONTROL_LOCAL 0x60
 #define OSC_CONTROL_LOCAL 0x70

 #define OSC_CONFIG_LOCAL OSC=INTIO67


	CONFIG OSC_CONFIG_LOCAL, FCMEN=OFF,IESO=OFF
;; wdt off, brownout is on and set for second highest voltage (hardware only)
	CONFIG PWRT=ON, BOREN=NOSLP, BORV=1

	CONFIG WDT=OFF,WDTPS=1
;; mclre disabled, timer1 cfg fow low power, portb pins are digital, ccp2 mux
	CONFIG MCLRE=OFF,PBADEN=OFF,CCP2MX=PORTC

;; debug, extended_instruction, low-voltage programming and reset_stack_err clr
	CONFIG DEBUG=OFF,XINST=OFF,LVP=OFF,STVREN=ON

;; code protect some blocks (program memory and eeprom)
	CONFIG CP0=OFF,CP1=OFF
	CONFIG CPB=OFF,CPD=OFF

;; write protect some blocks (program memory and eeprom)
	CONFIG WRT0=OFF,WRT1=OFF
	CONFIG WRTB=OFF,WRTC=OFF,WRTD=OFF

;; following two words protect the various program code blocks from table reads
	CONFIG EBTR0=OFF,EBTR1=OFF
	CONFIG EBTRB=OFF

 #ENDIF

 #IFDEF __18F2620

 INCLUDE "p18f2620.inc"
 #DEFINE PIC18_FAMILY

;; #define OSC_CONTROL_LOCAL 0x60
 #define OSC_CONTROL_LOCAL 0x70

 #define OSC_CONFIG_LOCAL OSC=INTIO67


	CONFIG OSC_CONFIG_LOCAL, FCMEN=OFF,IESO=OFF
;; wdt off, brownout is on and set for second highest voltage (hardware only)
	CONFIG PWRT=ON, BOREN=NOSLP, BORV=1

	CONFIG WDT=OFF,WDTPS=1
;; mclre disabled, timer1 cfg fow low power, portb pins are digital, ccp2 mux
	CONFIG MCLRE=OFF,PBADEN=OFF,CCP2MX=PORTC

;; debug, extended_instruction, low-voltage programming and reset_stack_err clr
	CONFIG DEBUG=OFF,XINST=OFF,LVP=OFF,STVREN=ON

;; code protect some blocks (program memory and eeprom)
	CONFIG CP0=OFF,CP1=OFF
	CONFIG CPB=OFF,CPD=OFF

;; write protect some blocks (program memory and eeprom)
	CONFIG WRT0=OFF,WRT1=OFF
	CONFIG WRTB=OFF,WRTC=OFF,WRTD=OFF

;; following two words protect the various program code blocks from table reads
	CONFIG EBTR0=OFF,EBTR1=OFF
	CONFIG EBTRB=OFF

 #ENDIF


	
   #DEFINE BANK1 0
   #DEFINE BANK2_TWIDDLE 0
   #DEFINE BANK3_TWIDDLE 0
   #DEFINE BANK4_TWIDDLE 0

;; registers 0x6 (6) through 0x39 (57)
  CBLOCK 0x4

  zc_noedge_counter

  rx_char

  zc_delayline

  ctr_ckt1, ctr_ckt2, ctr_ckt3, ctr_ckt4
  ctr_ckt5, ctr_ckt6, ctr_ckt7, ctr_ckt8



;; event_flag is used for communicating between the ISR and the foreground code
;;   in the inactive state all of the bits are high.  A bit is cleared in the
;;   ISR to indicate an event and set in the foreground when that event is
;;   processed.
;;   bit 0 is cleared every 256'th interrupt, used for status LED timeouts and
;;      detecting missing zero-crossing transitions.
;;   bit 1 is cleared on the leading edge of the zero-cross signal.

  event_flag

;; state_flag is used to hold state info used in the ISR.
;;   bit 0 is the value of the ZC signal on the last timer interrupt
;;   bit 1 is set when ZC goes high (i.e. transition), and is cleared when ZC
;;     goes low.  It goes low (or remains low) if there are no low-high
;;     transitions on the ZC signal.

  state_flag

  porta_image, portb_image, portc_image
  ctr_timeslot

  zc_tmr_save

;; Variables related to the 'charlie-plexed' status LEDs:
	
  LED_map
  LED_bit_ctr
	
  LED_anode_rotator
  LED_cathode_rotator
  LED_data_rotator

  _w, _status, LED_TRIS_mask
  isr_counter
  heartbeat_countdown
  frame_err_countdown
  rxd_countdown
  zc_countdown
  Timeout
  Timecounter


  dmxhighbyte:0		;; DMX
  our_addr
  dmxlowbyte:0		;; DMX  
  rx_discard_count		; PJS 4/22/11
  skiphigh:0		;; DMX
  rx_copycount			; PJS 4/22/11
  skiplow:0			;; DMX
  rx_flag_extra  ;; bit 0 set if prior character was an ESC.

  dmxPayloadCount
  smDmxL
  smDmxH	;; 0x3D - only two more variables left

  dmxStatusCtr

  ENDC

;; registers 0x40 (64) through 0x5F (95)

  CBLOCK 0x40

  slot_id_ckt1, slot_id_ckt2, slot_id_ckt3, slot_id_ckt4
  slot_id_ckt5, slot_id_ckt6, slot_id_ckt7, slot_id_ckt8	
	


  ENDC


;; Register(s) that are accessed in both the first and second banks.
;; registers 0x70 (112) through 0x78  (120)




;; assigned numbers:

 #define DEFAULT_LINERATE 50
 #define MAX_LINERATE 70

;; derived numbers:

;; initializer for baudrate generator (based on formula from datasheet)
;;  ********************Do Not Change********************


;;   #DEFINE BAUD_INIT (CLOCKRATE/BAUDRATE/8 - 1)  ;;64mhz
   #DEFINE BAUD_INIT (CLOCKRATE/BAUDRATE/4 - 1)  ;;32mhz

;; **************Initializer for cell timer ZC Setup********************

 #define CELL_RATE ((2*DEFAULT_LINERATE) * 256)

 #define CELL_TMR_INIT (CLOCKRATE/8/CELL_RATE)  ;;32 mhz
;; #define CELL_TMR_INIT (CLOCKRATE/16/CELL_RATE)  ;;64 mhz
;; Number used for range-checking prospective values for the periodic timer
;;    interval.
 #define MIN_ZC_TIMER ((CLOCKRATE/4)/MAX_LINERATE/256/2)  ;;32mhz
;; #define MIN_ZC_TIMER ((CLOCKRATE/8)/MAX_LINERATE/256/2)  ;;64mhz
	 	


#ifdef DMX
; ----------------- dmxread -------	

chgDmxState macro newState		 
		movlw	LOW(newState)
		movwf	smDmxL
		movlw	HIGH(newState)
		movwf	smDmxH
   endm

gotoDmxState macro
		movff	smDmxH, PCLATH
		movf	smDmxL,W
		movwf	PCL
	endm

#endif
;; ---------------------------------------------------------------------------


 org 0
	nop
	goto initialize


    ORG 8

	
;; interrupt routine
;; Just the timer interrupt for now...
;; ISR timing.
;;   isr_regular path:	
;;     18 - start of ISR through isr_regular
;;     55 - do_regular through isr_status_LEDs
;;     26 - longest path from isr_status_LEDs to/including the return
;;     99 - total
;;
;;   isr_load_counters path:
;;     22 - start of ISR through isr_load_counters (including ZC lead edge)
;;     36 - isr_load_counters through isr_status_LEDs
;;     41 - longest path from isr_status_LEDs to/including the return
;;     99 - total
;;
;; Requirement for the maximum length of this ISR
;;  These timer interrupts occur once overy 30 uS (120 instructions at 16 MHz).
;;  At 57600 baud there is a character available every 190 uS or so, or once
;;    every 6-1/3 timer interrupts.  It takes about 40 instructions to process
;;    one of these characters, or about 7 instructions per timer interrupt.
;;  So this ISR must be kept under 113 (120 - 7) instructions.
;;
;;  This analysis doesn't account for event handling (which probably doesn't
;;    matter, since uart handling have precedence over event-handling).
;;
Int


	bcf     PIR1,TMR2IF	; reset the interrupt
			
	incf	isr_counter,f	; update count of interrupts ... used only for
	btfsc	STATUS,Z	; LED blinking and charlie-plex state variable
	bcf	event_flag,0	; create event once every 256'th interrupt

	btfss	ZC
	clrf	state_flag

	btfsc	ZC		; ZC set this time? (if not, not leading edge)
	btfsc	state_flag,0	; ZC set last time? (if yes, not leading edge)
	goto	isr_not_zc_lead_edge

;; ZC leading edge

	bcf	event_flag,1	; ZC leading edge flag for foreground code
	bsf	state_flag,1	; ZC leading edge for ISR code.
	bsf	state_flag,0
	
	movf	TMR0L,w
	movwf	zc_tmr_save

isr_not_zc_lead_edge:

	incfsz	ctr_timeslot,f
	btfsc	state_flag,1
	goto	isr_load_counters
;;
;; Handle the normal case (selectively turn on dimmer outputs).  This code is
;;   bypassed if ctr_timeslot = 0 OR if state_flag.1 is set.
;;
isr_regular:

	movlw	255 & ~(DIMMER_BITMAP_PORTA)
    #IFDEF PIN2_CH
	incfsz	ctr_ckt#v(PIN2_CH),f
	iorlw	1	; turn off this SSR
    #ENDIF
    #IFDEF PIN3_CH
	incfsz	ctr_ckt#v(PIN3_CH),f
	iorlw	2	; turn off this SSR
    #ENDIF
    #IFDEF PIN4_CH
	incfsz	ctr_ckt#v(PIN4_CH),f
	iorlw	4	; turn off this SSR
    #ENDIF
    #IFDEF PIN5_CH
	incfsz	ctr_ckt#v(PIN5_CH),f
	iorlw	8	; turn off this SSR
    #ENDIF
    #IFDEF PIN6_CH
	incfsz	ctr_ckt#v(PIN6_CH),f
	iorlw	16	; turn off this SSR
    #ENDIF
    #IFDEF PIN7_CH
	incfsz	ctr_ckt#v(PIN7_CH),f
	iorlw	32	; turn off this SSR
    #ENDIF
    #IFDEF PIN10_CH
	incfsz	ctr_ckt#v(PIN10_CH),f
	iorlw	64	; turn off this SSR
    #ENDIF
    #IFDEF PIN9_CH
	incfsz	ctr_ckt#v(PIN9_CH),f
	iorlw	128	; turn off this SSR
    #ENDIF
	andwf	porta_image,f
	
	movlw	255 & ~(DIMMER_BITMAP_PORTB)
    #IFDEF PIN21_CH
	incfsz	ctr_ckt#v(PIN21_CH),f
	iorlw	1	; turn off this SSR
    #ENDIF
    #IFDEF PIN22_CH
	incfsz	ctr_ckt#v(PIN22_CH),f
	iorlw	2	; turn off this SSR
    #ENDIF
    #IFDEF PIN23_CH
	incfsz	ctr_ckt#v(PIN23_CH),f
	iorlw	4	; turn off this SSR
    #ENDIF
    #IFDEF PIN24_CH
	incfsz	ctr_ckt#v(PIN24_CH),f
	iorlw	8	; turn off this SSR
    #ENDIF
    #IFDEF PIN25_CH
	incfsz	ctr_ckt#v(PIN25_CH),f
	iorlw	16	; turn off this SSR
    #ENDIF
    #IFDEF PIN26_CH
	incfsz	ctr_ckt#v(PIN26_CH),f
	iorlw	32	; turn off this SSR
    #ENDIF
    #IFDEF PIN27_CH
	incfsz	ctr_ckt#v(PIN27_CH),f
	iorlw	64	; turn off this SSR
    #ENDIF
    #IFDEF PIN28_CH
	incfsz	ctr_ckt#v(PIN28_CH),f
	iorlw	128	; turn off this SSR
    #ENDIF
	andwf	portb_image,f
	
	movlw	255 & ~(DIMMER_BITMAP_PORTC)
    #IFDEF PIN11_CH
	incfsz	ctr_ckt#v(PIN11_CH),f
	iorlw	1	; turn off this SSR
    #ENDIF
    #IFDEF PIN12_CH
	incfsz	ctr_ckt#v(PIN12_CH),f
	iorlw	2	; turn off this SSR
    #ENDIF
    #IFDEF PIN13_CH
	incfsz	ctr_ckt#v(PIN13_CH),f
	iorlw	4	; turn off this SSR
    #ENDIF
    #IFDEF PIN14_CH
	incfsz	ctr_ckt#v(PIN14_CH),f
	iorlw	8	; turn off this SSR
    #ENDIF
    #IFDEF PIN15_CH
	incfsz	ctr_ckt#v(PIN15_CH),f
	iorlw	16	; turn off this SSR
    #ENDIF
    #IFDEF PIN16_CH
	incfsz	ctr_ckt#v(PIN16_CH),f
	iorlw	32	; turn off this SSR
    #ENDIF
    #IFDEF PIN17_CH
	incfsz	ctr_ckt#v(PIN17_CH),f
	iorlw	64	; turn off this SSR
    #ENDIF
    #IFDEF PIN18_CH
	incfsz	ctr_ckt#v(PIN18_CH),f
	iorlw	128	; turn off this SSR
    #ENDIF
	andwf	portc_image,f

	movf	porta_image,w
	xorlw	(DIMMER_BITMAP_PORTA & ~SINK_MAP_PORTA)
	movwf	PORTA
	movlw	LOW (~PWM_MAP_PORTA)
	iorwf	porta_image,f	
		
	movf	portb_image,w
	xorlw	(DIMMER_BITMAP_PORTB & ~SINK_MAP_PORTB)
	movwf	PORTB
	movlw	LOW (~PWM_MAP_PORTB)
	iorwf	portb_image,f

	movf	portc_image,w
	xorlw	(DIMMER_BITMAP_PORTC & ~SINK_MAP_PORTC)
	movwf	PORTC
	movlw	~PWM_MAP_PORTC
	iorwf	portc_image,f
	goto	isr_status_LEDs

;;
;; Initialize the ctr_ckt registers for upcoming half-cycle.  Copy the slot_id
;;   registers to the ctr registers, set output port values according to
;;   whether the dimmer values are 255 or not.
;;
isr_load_counters:
	movlw	CTR_LOCKOUT
	movwf	ctr_timeslot

	btfsc	state_flag,0
	clrf	TMR0L

    #IFDEF PIN2_CH
	movf	slot_id_ckt#v(PIN2_CH),w
	movwf	ctr_ckt#v(PIN2_CH)
    #ENDIF

    #IFDEF PIN3_CH
	movf	slot_id_ckt#v(PIN3_CH),w
	movwf	ctr_ckt#v(PIN3_CH)
    #ENDIF
		
    #IFDEF PIN4_CH
	movf	slot_id_ckt#v(PIN4_CH),w
	movwf	ctr_ckt#v(PIN4_CH)
    #ENDIF
		
    #IFDEF PIN5_CH
	movf	slot_id_ckt#v(PIN5_CH),w
	movwf	ctr_ckt#v(PIN5_CH)
    #ENDIF
		
    #IFDEF PIN6_CH
	movf	slot_id_ckt#v(PIN6_CH),w
	movwf	ctr_ckt#v(PIN6_CH)
    #ENDIF
		
    #IFDEF PIN7_CH
	movf	slot_id_ckt#v(PIN7_CH),w
	movwf	ctr_ckt#v(PIN7_CH)
    #ENDIF

    #IFDEF PIN10_CH
	movf	slot_id_ckt#v(PIN10_CH),w
	movwf	ctr_ckt#v(PIN10_CH)
    #ENDIF
	
    #IFDEF PIN9_CH
	movf	slot_id_ckt#v(PIN9_CH),w
	movwf	ctr_ckt#v(PIN9_CH)
    #ENDIF

    #IFDEF PIN21_CH
	movf	slot_id_ckt#v(PIN21_CH),w
	movwf	ctr_ckt#v(PIN21_CH)
    #ENDIF

    #IFDEF PIN22_CH
	movf	slot_id_ckt#v(PIN22_CH),w
	movwf	ctr_ckt#v(PIN22_CH)
    #ENDIF
		
    #IFDEF PIN23_CH
	movf	slot_id_ckt#v(PIN23_CH),w
	movwf	ctr_ckt#v(PIN23_CH)
    #ENDIF
		
    #IFDEF PIN24_CH
	movf	slot_id_ckt#v(PIN24_CH),w
	movwf	ctr_ckt#v(PIN24_CH)
    #ENDIF
		
    #IFDEF PIN25_CH
	movf	slot_id_ckt#v(PIN25_CH),w
	movwf	ctr_ckt#v(PIN25_CH)
    #ENDIF
		
    #IFDEF PIN26_CH
	movf	slot_id_ckt#v(PIN26_CH),w
	movwf	ctr_ckt#v(PIN26_CH)
    #ENDIF

    #IFDEF PIN27_CH
	movf	slot_id_ckt#v(PIN27_CH),w
	movwf	ctr_ckt#v(PIN27_CH)
    #ENDIF
	
    #IFDEF PIN28_CH
	movf	slot_id_ckt#v(PIN28_CH),w
	movwf	ctr_ckt#v(PIN28_CH)
    #ENDIF

    #IFDEF PIN11_CH
	movf	slot_id_ckt#v(PIN11_CH),w
	movwf	ctr_ckt#v(PIN11_CH)
    #ENDIF

    #IFDEF PIN12_CH
	movf	slot_id_ckt#v(PIN12_CH),w
	movwf	ctr_ckt#v(PIN12_CH)
    #ENDIF
		
    #IFDEF PIN13_CH
	movf	slot_id_ckt#v(PIN13_CH),w
	movwf	ctr_ckt#v(PIN13_CH)
    #ENDIF
		
    #IFDEF PIN14_CH
	movf	slot_id_ckt#v(PIN14_CH),w
	movwf	ctr_ckt#v(PIN14_CH)
    #ENDIF
		
    #IFDEF PIN15_CH
	movf	slot_id_ckt#v(PIN15_CH),w
	movwf	ctr_ckt#v(PIN15_CH)
    #ENDIF
		
    #IFDEF PIN16_CH
	movf	slot_id_ckt#v(PIN16_CH),w
	movwf	ctr_ckt#v(PIN16_CH)
    #ENDIF

    #IFDEF PIN17_CH
	movf	slot_id_ckt#v(PIN17_CH),w
	movwf	ctr_ckt#v(PIN17_CH)
    #ENDIF
	
    #IFDEF PIN18_CH
	movf	slot_id_ckt#v(PIN18_CH),w
	movwf	ctr_ckt#v(PIN18_CH)
    #ENDIF

	movlw	DIMMER_BITMAP_PORTA
	iorwf	porta_image,f
	movf	porta_image,w
	xorlw	(DIMMER_BITMAP_PORTA & ~SINK_MAP_PORTA)
	movwf	PORTA
		
	movlw	DIMMER_BITMAP_PORTB
	iorwf	portb_image,f
	movf	portb_image,w
	xorlw	(DIMMER_BITMAP_PORTB & ~SINK_MAP_PORTB)
	movwf	PORTB

	movlw	DIMMER_BITMAP_PORTC
	iorwf	portc_image,f
	movf	portc_image,w
	xorlw	(DIMMER_BITMAP_PORTC & ~SINK_MAP_PORTC)
	movwf	PORTC

isr_status_LEDs:	

;;
;; Basically divide by 16...
;;
	btfss	isr_counter,3	; skip LED work if count >=3 (modulo 16)
	btfsc	isr_counter,2
	goto	isr_exit

;; Go through this path once every 16 times around...translates to an LED
;; refresh rate of 200 or 240 Hz.

isr_extra_work:	

	btfsc	isr_counter,1
	goto	isr_extra_work2
	
	btfsc	isr_counter,0
	goto	isr_extra_work1

;;
;; Turn all of the LEDs OFF before working on the PORTS to prevent ghosting
;;

	bsf	LED0_TRIS
	bsf	LED1_TRIS
	bsf	LED2_TRIS

	
;;
;; Set one bit in the portx_image register that will apply positive voltage to
;; one of the LED anodes, and make sure that the other bits are low (for the
;; cathodes).
;;

	bcf	LED0_anode_sel
	btfsc	LED_anode_rotator,7
	bsf	LED0_anode_sel

	bcf	LED1_anode_sel
	btfsc	LED_anode_rotator,6
	bsf	LED1_anode_sel


	goto	isr_exit	; 20 clocks from , including this instruction
	
isr_extra_work1:
;;
;; now turn on selected LED bits in the portx_image registers
;;
	bcf	LED2_anode_sel
	btfsc	LED_anode_rotator,5
	bsf	LED2_anode_sel

	movf	LED_anode_rotator,w
	iorwf	LED_cathode_rotator,w

	btfss	LED_data_rotator,7 ; clear the TRIS mask if the LED is OFF
	movlw	0x00

	movwf	LED_TRIS_mask
		
	goto	isr_exit	; 18 clocks, including this instruction
	
isr_extra_work2:
	btfsc	isr_counter,0
	goto	isr_extra_work3

;;
;; Now write to the TRIS registers (setting two bits, one for an anode and
;;   one for a cathode).
;;

	
	btfsc	LED_TRIS_mask,7
	bcf	LED0_TRIS

	btfsc	LED_TRIS_mask,6
	bcf	LED1_TRIS

	btfsc	LED_TRIS_mask,5
	bcf	LED2_TRIS


	
;; start updating registers for next time around
	
	rlcf	LED_anode_rotator,w
	rlcf	LED_anode_rotator,f

	goto	isr_exit	; 17 instructions, including this one.

isr_extra_work3:	
;;
;; Now update registers for next time.
;;
	rlcf	LED_cathode_rotator,w
	rlcf	LED_cathode_rotator,f

	rlcf	LED_data_rotator,f
	
	decfsz	LED_bit_ctr,f
	goto	isr_exit

	bsf	LED_bit_ctr,3
	
	movf	LED_map,w
	movwf	LED_data_rotator; 13 instructions, including this one.

isr_exit:


	retfie	FAST

	
;;
;; End of Interrupt Routine, start of Foreground code.
;;

initialize:
;;
;; initialize some registers
;;
	clrf	slot_id_ckt1	; set all 'lamps' to lowest intensity
	clrf	slot_id_ckt2
	clrf	slot_id_ckt3
	clrf	slot_id_ckt4
	clrf	slot_id_ckt5
	clrf	slot_id_ckt6
	clrf	slot_id_ckt7
	clrf	slot_id_ckt8



	
	clrf	FSR0L		; start out discarding data
	clrf	zc_noedge_counter

	clrf	FSR0H

	clrf	ctr_timeslot
	clrf	isr_counter
	clrf	porta_image
	clrf	portb_image
	clrf	portc_image

	movlw	0xFF
	movwf	event_flag	; no events pending

#IFDEF DMX
;-------------------------Timeout addition
    movlw   0x83
    movwf   Timecounter
    movlw   0xFF
    movwf   Timeout
;-----------------------------

#ENDIF

		
;; initialize the ports - all outputs 'OFF".
	
	BANKSEL	PORTA
	movlw	DIMMER_BITMAP_PORTA & SINK_MAP_PORTA
	movwf	PORTA
	movlw	DIMMER_BITMAP_PORTB & SINK_MAP_PORTB
	movwf	PORTB
	movlw	DIMMER_BITMAP_PORTC & SINK_MAP_PORTC
	movwf	PORTC


;; Set up the main oscillator, slightly different between the PIC18F and PIC16F
;;   families.
;;  {PR}	


 
	movlw	OSC_CONTROL_LOCAL
	movwf	OSCCON
	movlw	OSC_TUNE_LOCAL
	movwf	OSCTUNE


;; Turn off the analog stuff so that the I/O pins can be used for digital
;;   purposes.  The exact method of doing this will vary from part to part.
;; {PR}


  #IFDEF __18F25K22
;;	BANKSEL ANSELA   ; all A/D pins are digital
	clrf	ANSELA
	clrf	ANSELB
    clrf	ANSELC

    CLRF LATA ; Alternate method to clear output data latches
    CLRF LATB
    CLRF LATC
   
    CLRF ANSELA ; Configure analog pins or digital only
	CLRF ANSELB
    CLRF ANSELC



  #ENDIF

  #IFNDEF __18F25K22
	movlw	(1<<PCFG3) | (1<<PCFG2) | (1<<PCFG1) | (1<<PCFG0)
	movwf	ADCON1		; all A/D pins are digital
  #ENDIF




;; Start out with just the triac control bits programmed for output.  The TxD
;;   pin will become an output when the UART is configured, independent of the
;;   TRIS registers.  All other pins start out as inputs.  The status LED pins
;;   are re-configured from the ISR in accordance with the charlie-plexing
;;   scheme adopted here.
	
	BANKSEL	TRISA
	movlw	255 & ~(DIMMER_BITMAP_PORTA)
	movwf	TRISA^BANK1	; triac control bits are output
	movlw	255 & ~(DIMMER_BITMAP_PORTB)
	movwf	TRISB^BANK1	; triac control bits are output
	movlw	255 & ~(DIMMER_BITMAP_PORTC)
	movwf	TRISC^BANK1	; triac control bits are output





	bcf	RCON,IPEN
	clrf	PIE1
	clrf	PIE2

;; Read our address from the EEPROM at the end of this file
#ifdef DMX
  #ifdef HARD_CODE_DMX_ADDR
		movlw high DMX_START_ADDRESS
		movwf dmxhighbyte
		movlw low DMX_START_ADDRESS
		movwf dmxlowbyte
  #else
		movlw	LOW(EE_DMX_HIGH)
		movwf	EEADR
		bcf		EECON1, EEPGD 	; point to adata memory
		bcf		EECON1, CFGS	; Access EEPROM
		bsf		EECON1, RD		; EE read
		movff	EEDATA, dmxhighbyte	

		movlw	LOW(EE_DMX_LOW)
		movwf	EEADR
		bcf		EECON1, EEPGD 	; point to adata memory
		bcf		EECON1, CFGS	; Access EEPROM
		bsf		EECON1, RD		; EE read
		movff	EEDATA, dmxlowbyte
  #endif
	chgDmxState smDmxIdle
#else
  #IFDEF START_ADDRESS
	movlw   0x00
	movwf	EEADR
;;	movwf	EEADRH  ;;may need to be commented out depending on PIC device
	movwf	EECON1

	bsf		EECON1,RD
	movf	EEDATA,w

	movwf	our_addr

	movlw	0x01  ; convert 1-based value from EEPROM into
	subwf	our_addr,f  ; 0-based value used in rx routine.
  #ENDIF
#endif	
;; configure the UART - same for PIC16F and PIC18F parts.

	movlw	BAUD_INIT;
	BANKSEL SPBRG
	movwf	SPBRG ^ BANK1

    clrf	SPBRGH
	movlw	1<<BRG16   
	movwf	BAUDCTL


	BANKSEL TXSTA
	
	movlw	(1<<TXEN) | (1<<BRGH); enable 8-bit transmitter in async mode
				; (x16 baud divisor)
	movwf	TXSTA ^ BANK1

	BANKSEL RCSTA	
	movlw	(1<<SPEN) | (1<<CREN); enable serial port with continuous 8-bit rx
	movwf	RCSTA
	
;; Set up timer2 for generating periodic interrupts.

	movlw	CELL_TMR_INIT
	BANKSEL PR2
	movwf	PR2 ^ BANK1
	
	movlw	(1 << TMR2ON)

	BANKSEL T2CON
	movwf	T2CON

	BANKSEL PIE1
	bsf	PIE1 ^ BANK1,TMR2IE

	BANKSEL	PIR1
	bcf	PIR1,TMR2IF
	
;; Set up timer0 for timing how long the zero-crossing signal is OFF.


	movlw	1<<TMR0ON | 1<<T08BIT | 1<<T0PS0 | 1<<T0PS1 | 1<<T0PS2
	movwf	T0CON


	BANKSEL	PORTA

	clrf	INTCON

	clrf	state_flag
	btfsc	ZC
	bsf	state_flag,0	; bit 0 indicates zc is on at the very start
	btfsc	state_flag,0
	bsf	ZC_LED
	
	movlw	8
	movwf	LED_bit_ctr
	
	movlw	0x88
	movwf	LED_anode_rotator

	movlw	0x52
	movwf	LED_cathode_rotator

	clrf	LED_data_rotator; all status LEDs initially off.
	clrf	LED_map		;
	
	movlw	(1 <<GIE) | (1<<PEIE)
	movwf	INTCON		; enable interrupts

	clrf	zc_delayline

;; Longest serial RX pathlengths through this code (no events processed)
;; 21 instructions - discard state
;; 27 instructions - pure discard state
;; 19 instructions - Sync char (0x7E)
;; 41 instructions - command byte (just after the Sync char).
;; 41 instructions - data byte (not ESC)
;;
;; normal Rx char (data copied to memory)
mainloop:
	incfsz	event_flag,w	; check event flags from ISR
	call	event_processor
	
#ifdef DMX

dmxEngine
 BANKSEL RCSTA
  btfsc RCSTA,OERR  ; BANK 3 - test for overrun error (rare)
  goto dmxError

 BANKSEL PIR1
  btfss PIR1,RCIF  ; Received a character yet?
  goto mainloop  ; Nope - jump back
  gotoDmxState


dmxError
		bcf		RCSTA,CREN	; Overrun error
		movf	RCREG,w		; Flush the FIFO

		movf	RCREG,w		; Flush the FIFO

		bsf		RCSTA,CREN


		; Fall through
smDmxIdle
        movf    dmxhighbyte,w           ; Skipcounter is used to detmine how many
        movwf   skiphigh                ; received data bytes are skipped before the RGB
        movf    dmxlowbyte,w            ; data is collected.  Load skipcounter with
        movwf   skiplow                 ; the DMX address from above...
        movf    skiplow,f               ; ... then decrement it by one
        btfsc   STATUS,Z                ; so we know how many channels to ignore before the
        decf    skiphigh,f              ; useful data arrives.  We'll see more of the
        decf    skiplow,f               ; skipcounter a bit farther down the page.
		chgDmxState smWait4break


		; Fall through.. an optimization cheat

smWait4break

	BANKSEL RCSTA
        btfss   RCSTA,FERR               ; Here we're waiting to see if a break occurs

		goto	purgeSerial





		; Code to implement 1HZ blinking when DMX is detected

		decfsz	dmxStatusCtr,f

		goto	endBlink



		movlw	DMX_BLINK_RATE			; Blink every 44 frames = 1Hz

		movwf	dmxStatusCtr

		movf	RXD_LED_REG,W

		xorlw	1<<RXD_LED_BIT

		movwf	RXD_LED_REG

endBlink

		chgDmxState smWait4Start
purgeSerial

	BANKSEL RCREG

		movf	RCREG,w					; Purge the incoming Buffer

		goto	mainloop


smWait4Start
	BANKSEL RCSTA
        btfsc   RCSTA,FERR            	; a framing error.  If all is well AND the
        goto	purgeSerial		        ; new byte is zero (which means the start code
        movf    RCREG,w                 ; is also zero, it's okay to begin gethering channel
                                        ; data

	; Define target of payload



		btfss	STATUS,Z				; Check for a ZERO byte

		goto	smDmxIdle				; Was not a ZERO - start looking again
;--------------------------------Timeout addition ---------------------
		movlw	0xFF                    
		movwf	Timeout
;----------------------------------------------------------------------
	BANKSEL FSR0L
		movlw	slot_id_ckt1			; next state is normal body
		movwf	FSR0L					; point to start of slot_id area

	; Define PAYLOAD size
		movlw	0x08 					; 8 bytes of payload
		movwf	dmxPayloadCount
		chgDmxState	smScanDmxData

        movf    skiphigh,W           	; Here check to see if the highbyte is
        btfss   STATUS,Z                ; zero. If it is,check to see if the
        goto	mainloop	            ; lowbyte is 1.  If 1, grab the next 32 bytes
        movf    skiplow,w            	; which come through.  If <> 1, go to the routine
        btfss	STATUS,Z                ; which receives and discards bytes until the
        goto	mainloop                ; DMX address has been reached.
        chgDmxState smReadDmxPayload
		goto	mainloop

smScanDmxData

	BANKSEL RCSTA
        btfss   RCSTA,FERR            	; Test for a framing error.
        goto	continueScan	        ;



		chgDmxState smWait4Start

		goto	purgeSerial



continueScan

	BANKSEL RCREG
        movf    RCREG,w                 ; Then, capture & move to 'w'...

        movf    skiplow,f               ; ...decrement the skip counter...
        btfsc   STATUS,Z                ; (all sixteen bits of it)
        decf    skiphigh,f
        decf    skiplow,f
                                        ; ...and see if we've reached the start address.
        movf    skiplow,W               ; If the skip counter now equals zero, we know
        btfss   STATUS,Z                ; that we need to gather the next five bytes
        goto	mainloop	            ; and save them as RGBAW data.  If the counter is
        movf    skiphigh,W              ; still nonzero, loop back and do it again.
        btfss   STATUS,Z
        goto	mainloop
		chgDmxState smReadDmxPayload
		goto	mainloop

smReadDmxPayload

	BANKSEL RCSTA
        btfss   RCSTA,FERR            	; Test for a framing error.
        goto	continuePayload	        ;


		chgDmxState smWait4Start

		goto	purgeSerial



continuePayload

	BANKSEL RCREG
        movf    RCREG,W

	BANKSEL INDF0
		movwf	INDF0					; Copy to the target
		incf	FSR0L,f         		; Increment the target
		decf	dmxPayloadCount,F
		btfss	STATUS,Z
		goto	mainloop

smDmxDone

		chgDmxState smDmxIdle

		goto	mainloop
#else

	btfss	PIR1,RCIF	; skip if UART has a rx character to process
	goto	mainloop	; no character available

	movlw	6		; Rx error mask (framing error, overrun error)
	andwf	RCSTA,w		; get (masked) rx status
	btfss	STATUS,Z
	goto	rx_error	; goto error routine

	movf    RCREG,w
	movwf	rx_char		; and save a working copy
	
	addlw	0x82		; test for sync character
	btfss	STATUS,Z	; skip if sync character
	goto	rx_not_sync	; jump to code for processing non-sync

;; 
;; process sync character
;;
	clrf	FSR0L		; next byte is command byte, regardless of
	bsf		FSR0L,7		; prior state
rx_echo:	
	movf	rx_char,w	; echo most recent character to transmitter
	movwf	TXREG
	goto	mainloop

rx_error:
	movf	RCREG,w
	
	movlw	0x80


	movwf	frame_err_countdown
    bsf FRAME_ERR_LED


	bcf	RCSTA,CREN
	bsf	RCSTA,CREN
	clrf	FSR0L		; start discarding
	goto	mainloop

;; 
;; process non-sync character
;;

;; rx_state == FSR:
;; 00000000 -> discarding data (startup, just after rx error)
;; 1xxxxxxx -> expecting command/address byte (just after SYNC)
;; 010nnnnn -> process short-form data character
;; 011xxxxx -> echoing data (count was > 0 or after end of local data)
	
rx_not_sync:
	addlw	1		; test for PAD character - discard if yes
	btfsc	STATUS,Z
	goto	mainloop

	movf	FSR0L,f	
	btfsc	STATUS,Z	; skip if not discard state
	goto	mainloop
	btfsc	FSR0L,3		; ****skip if non-echo state,,was 5 RP32, RP16 4 TR8 3***
	goto	rx_echo		; pure echo
;;
;; handle non-echo, non-SYNC cases
;;
	btfss	FSR0L,7		; skip if first character after SYNC
	goto	decode_packet_body
decode_cmd_byte:		; first character after the SYNC
	bcf	FSR0L,7		; no longer on 'first char after SYNC'
	bsf	FSR0L,5		; just in case packet is not for us.
	btfss	rx_char,7
	goto	rx_echo		; long form packet, just echo it.

  #IFDEF START_ADDRESS

	movf	our_addr,w
	subwf	rx_char,f	; incoming address - 'our_addr'

	btfsc	rx_char,7	; bit 7 set after the subtract?  If so, data is
	goto	rx_echo		; all for downstream controllers.

	movlw	0x81
	addwf	rx_char,w
	movwf	rx_discard_count; PJS 4/22/11
	
	movlw	0x80
	movwf	rx_char
	movlw	slot_id_ckt1	; next state is normal body
	movwf	FSR0L		; point to start of slot_id area
	bcf	rx_flag_extra,7	; most recent char wasn't ESC
	
;;	incf	rx_char,f	; commented out be PJS 4/22/11
	clrf	rx_copycount	; PJS 4/22/11
	
	goto	rx_echo		; get next character (and echo the '0x80').

  #ELSE

	decf	rx_char,f	; 
	btfsc	rx_char,7	; value 0x7F after dec?  If so, it's for us
	goto	rx_echo

	movlw	slot_id_ckt1	; next state is normal body
	movwf	FSR0L		; point to start of slot_id area
	bcf	rx_flag_extra,7	; most recent char wasn't ESC
	
	incf	rx_char,f

	goto	rx_echo		; get next character (and echo the '0x80').

  #ENDIF

;;
;; done initializing packet, now handle the data
;;
;; to get here, char was not SYNC, FSR.7 was clear, FSR.5 was clear, FSR NZ
;;
decode_packet_body:

	btfss	rx_flag_extra,7	; skip if previous char was ESC
	goto	previous_not_ESC
	movlw	0x4E
	addwf	rx_char,f	; decode the ESC charactor
	goto	after_ESC_decoding
previous_not_ESC:	
	bsf	rx_flag_extra,7	; just in case current char is ESC
	movlw	0x7F
	xorwf	rx_char,w	; check for ESC
	btfsc	STATUS,Z	; skip if not ESC
	goto	mainloop	; go and discard the ESC
after_ESC_decoding:
	bcf	rx_flag_extra,7	; normal data state

  #IFDEF START_ADDRESS

	movf	rx_discard_count,f ; PJS 4/22/11  
	btfss	STATUS,Z	; PJS 4/22/11
	goto	rx_upfront_discard ; PJS 4/22/11
	
	movlw	0x80		; reset countdown flag to max value.

	movwf	rxd_countdown
    bsf RXD_LED


	movf	rx_char,w	; copy data to memory..
	movwf	INDF0
	incf	FSR0L,f

	goto	mainloop	; no echo !

; PJS 4/22/11 :		
;; logic for discarding/ignoring bytes until the place indicated by the
;; start address is reached.
	
rx_upfront_discard:		; PJS 4/22/11
	incf	rx_copycount,f	; PJS 4/22/11
	btfsc	rx_copycount,3	; PJS 4/22/11
	incf	rx_discard_count,f	; PJS 4/22/11
	bcf	rx_copycount,3	; PJS 4/22/11

	goto	mainloop	; PJS 4/22/11

  #ELSE

	movlw	0x80		; reset countdown flag to max value.
	movwf	rxd_countdown
    bsf RXD_LED


	movf	rx_char,w	; copy data to memory..
	movwf	INDF0
	incf	FSR0L,f

	goto	mainloop	; no echo ! 

  #ENDIF	
;;
#endif ;; RENARD protocol	
;; do all work associated with zero-crossing (ac line frequency detection)
;;
;; event_flag bit assignments
;; bit 0 - clear if interrupt counter incremented through 0
;; bit 1 - clear if rising edge on ZC was detected.

event_processor:

	btfsc	event_flag,0	; bit 0 is clear if 256 interrupts have occured
	goto	end_timer_events

#IFDEF DMX
;-----------------------Timeout addition ----------------------------
#DEFINE Lights_out

#ifdef Lights_out

    decf    Timecounter,F
    btfsc   STATUS,Z
    goto    NormalCode
    movlw   0x83
    movwf   Timecounter

    decf    Timeout,F
    btfsc   STATUS,Z
    goto    clearData
    goto    NormalCode
clearData:
    movlw   0xFF
    movwf   Timeout
;       We need to clear out all the data values
	BANKSEL slot_id_ckt1
	clrf	slot_id_ckt1	; set all 'lamps' to lowest intensity
	clrf	slot_id_ckt2
	clrf	slot_id_ckt3
	clrf	slot_id_ckt4
	clrf	slot_id_ckt5
	clrf	slot_id_ckt6
	clrf	slot_id_ckt7
	clrf	slot_id_ckt8



#endif

;---------------------------------------------------------------
NormalCode:

#ENDIF

;; 256 timer interrupts have occurred

	bsf	event_flag,0

; count down to determine when to toggle the heartbeat LED


    movlw   1<<HEARTBEAT_LED_BIT ; update heartbeat
	decf	heartbeat_countdown,f
	btfsc	STATUS,Z
	xorwf	HEARTBEAT_LED_REG,f


	
; count down to determine when to turn off the RxD LED.



	decf	rxd_countdown,f
	btfsc	STATUS,Z
    xorwf   HEARTBEAT_LED_REG,f
	
; count down to determine when to turn off the Framing Error LED
	
	decf	rxd_countdown,f
	btfsc	STATUS,Z
    bcf RXD_LED

; count down to determine when to turn off the Framing Error LED
	
	decf	frame_err_countdown,f
	btfsc	STATUS,Z
	bcf	FRAME_ERR_LED




; count down to detect missing ZC edges

	decfsz	zc_noedge_counter,f
	goto	end_timer_events

	bcf	ZC_LED		; approximately 16 missing ZC pulses, so set ZC
	btfsc	ZC		; LED to reflect the current state of the ZC.
	bsf	ZC_LED

	bsf	zc_noedge_counter,4

end_timer_events:
	btfsc	event_flag,1	; ZC rising edge detected?
	return		;

;;
;; The following code is only executed on the rising edge of ZC.  It is never
;;   executed if ZC is permanently high or low (i.e. DC application).
;;
	bsf	event_flag,1
	
	movlw	1<<ZC_LED_BIT	; select zero-crossing bit.

	decf	zc_countdown,f	; flip the zc LED once every 256 times.

	btfsc	STATUS,Z
	xorwf	ZC_LED_REG,f	; toggle the zc LED if countdown expired

	movlw	16		; reset the counter looking for missing edges
	movwf	zc_noedge_counter
	
; update the periodic interrupt limit (to adjust for differing AC frequencies).

	movf	zc_tmr_save,w
	sublw	MIN_ZC_TIMER	; MIN_ZC_TIMER - zc_tmr_save 

	btfsc	STATUS,C	; CY set means no borrow, so zc_tmr_save is
	return			; too small ... so return without updating PR2

	decf	zc_tmr_save,w	; update PR2 (and clear TMR2 if past PR2)
	
	bcf	INTCON,GIE

	BANKSEL	PR2
	movwf	PR2 ^ BANK1	; save new PR2 value
	BANKSEL TMR2
	subwf	TMR2,w		; TMR2 - PR2 
	btfsc	STATUS,C	; CY set means no borrow -> TMR2 >= PR2
	clrf	TMR2		; reset TMR2 because it's over the limit
	
	bsf	INTCON,GIE
zc_filter_restart_timer:
	return

  #IFDEF START_ADDRESS

	ORG 0xF00000
	de 2+START_ADDR

  #ENDIF

  #IFDEF DMX_START_ADDRESS

	ORG 0xF00000
	de HIGH(DMX_START_ADDRESS),LOW(DMX_START_ADDRESS)
	de 0xDE,0xAD,0xC0,0xDE

  #ENDIF

  
  

	END


