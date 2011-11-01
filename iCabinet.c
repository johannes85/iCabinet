/**
 * Open iCabinet ATUSB
 * v0.4
 *
 * by DomesticHacks
 * http://domestichacks.info/
 * http://www.youtube.com/DomesticHacks
 *
 * License:
 * Creative Commons: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)
 * http://creativecommons.org/licenses/by-nc-sa/3.0/
 */

#include <avr/io.h>
//#include <avr/pgmspace.h>
#include <string.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "usb_keyboard.h"

//Enables the usage of atari 7800 controllers. See documentation! 
#define ATARI_2_BUTTONS true

//Debounce time. Increase if one joystick movement triggers more then one action.
// 40 is a good value for the new Competition Pro
// 60 should work for older gamepads
#define DEBOUNCE_TIME 40

/*
 Configs:
 
 1: 1 Button Game 
    Tested: Super Breakout(A), Crystal Castles(A), Milipede(A)
 2: 2 Buttons Game
    Tested: Asteroids(A), Centipede(A), Asteroids Deluxe(A), Space Duel(A)
 3: 2 Buttons Game
    Tested: Liberator(A), Mayor Havoc(A)
 4: Asteroids
    Tested: Asteroids(A)
 5: Battlezone
    Tested: Battlezone(A)
 6: Debug mapping: Even the joystick is mapped to buttons, so every button can be tested
 
 Note:
 The configs are for a German iOS keyboard layout.
 Swap Z and Y for a english one.
*/

static uint8_t onStateConfigs[6][8] = {
  {KEY_W,KEY_X,KEY_A,KEY_D,KEY_L,KEY_L,KEY_L,KEY_L},
  {KEY_W,KEY_X,KEY_A,KEY_D,KEY_L,KEY_O,KEY_L,KEY_O},
  {KEY_W,KEY_X,KEY_A,KEY_D,KEY_L,KEY_J,KEY_L,KEY_J},
  {KEY_W,KEY_X,KEY_A,KEY_D,KEY_J,KEY_K,KEY_L,KEY_L},
  {KEY_W,KEY_X,KEY_A,KEY_D,KEY_J,KEY_J,KEY_J,KEY_J},
  {KEY_Z,KEY_U,KEY_I,KEY_O,KEY_H,KEY_J,KEY_K,KEY_L}
};
static uint8_t offStateConfigs[6][8] = {
  {KEY_E,KEY_Y,KEY_Q,KEY_C,KEY_V,KEY_V,KEY_V,KEY_V},
  {KEY_E,KEY_Y,KEY_Q,KEY_C,KEY_V,KEY_G,KEY_V,KEY_G},
  {KEY_E,KEY_Y,KEY_Q,KEY_C,KEY_V,KEY_N,KEY_V,KEY_N},
  {KEY_E,KEY_Y,KEY_Q,KEY_C,KEY_N,KEY_P,KEY_V,KEY_V},
  {KEY_E,KEY_Y,KEY_Q,KEY_C,KEY_N,KEY_N,KEY_N,KEY_N},
  {KEY_T,KEY_F,KEY_M,KEY_G,KEY_R,KEY_N,KEY_P,KEY_V}
};

#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD &= ~(1<<6))
#define LED_OFF		(PORTD |= (1<<6))
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz       0x00
#define CPU_8MHz        0x01


uint8_t onStates[8];
uint8_t offStates[8];
uint8_t btnUpState = 1;
uint8_t btnDownState = (1<<1);
uint8_t btnLeftState = (1<<2);
uint8_t btnRightState = (1<<3);
uint8_t btnAState = (1<<4);
uint8_t btnBState = (1<<5);
uint8_t btnCState = (1<<6);
uint8_t btnDState = (1<<7);
uint8_t btnModeState = (1<<7);
uint8_t btnInvertModeState = 255; // Set to not occuring value to force action on power up
uint8_t currentConfig = 0;
uint8_t maxConfig = 5;
uint8_t needsDebounce = 0;
uint8_t buttonState;

/**
 * Activates a configuration by id
 */
void setConfig(uint8_t config){
  // Writing config id into eprom to keep it after power down
  eeprom_write_byte(0,config);
  
  memcpy(onStates,onStateConfigs[config],8);
  memcpy(offStates,offStateConfigs[config],8);
  
  // Giving feedback to the user which config is set
  for(uint8_t a=0;a<config+1;a++){
	PORTC |= (1<<4);
	_delay_ms(100);
    PORTC &= ~(1<<4);
    _delay_ms(150);
  }
}

int main(void)
{
	CPU_PRESCALE(CPU_16MHz);
	
	LED_CONFIG;	
	LED_OFF;
	
	/*
	 * PB = up(pin0) down left right a b c d(pin7)
	 */
	DDRB = 0x00;	// Set port direction
	PORTB = 0xFF; 	// Enable internal pullups
	/*
	 * PC4 = LED
	 * PC6 = Inverted (ATARI 7800) mode button
	 * PC7 = Mode button
	 */
	DDRC = (1<<4);	// Set port direction
	PORTC = (1<<7);	// Enable internal pullups
	
	LED_ON;
	usb_init();
	while (!usb_configured()) // Wait until usb is configured
	_delay_ms(1000);
	LED_OFF;
	
	currentConfig = eeprom_read_byte(0);
	if(currentConfig >= maxConfig){
		currentConfig = 0;
	}
	if(currentConfig < 0){
		currentConfig = 0;
	}
	setConfig(currentConfig);
	
	while (1) {		
		#if defined(ATARI_2_BUTTONS)
		// Switch to inverted button mode (for Atari 7800 controllers)
		buttonState = (PINC & (1<<6));
		if(buttonState != btnInvertModeState){
			if(!buttonState){ // Button pressed
				// Atari 7800 mode
				PORTB &= ~((1<<4) | (1<<5));	// Disable internal pullups
				//Set default state to not pressed
				btnAState = 0;
				btnBState = 0;
			}else{
				// Atari one button mode (ATARI 2600, C64 etc.)
				PORTB |= (1<<4) | (1<<5);	// Enable internal pullups
				//Set default state to not pressed
				btnAState = (1<<4);
				btnBState = (1<<5);
			}
			btnInvertModeState = buttonState;
			needsDebounce = 1; 
		}
		#else
			btnInvertModeState = (1<<6);
		#endif
	
		// Mode Button
		buttonState = (PINC & (1<<7));
		if(buttonState != btnModeState){
			if(!buttonState){ // Button pressed
				currentConfig++; // Set config id to next one
				if(currentConfig > maxConfig){ // Content id overflow
					currentConfig = 0;
				}
				setConfig(currentConfig); // Activate config by id
			}
			btnModeState = buttonState; // Store current button state for edge detection
			needsDebounce = 1; 
		}
		
		// Up Button
		buttonState = (PINB & 1);
		if (buttonState != btnUpState) {
			if(buttonState == 0){ 
				usb_keyboard_press(onStates[0],0);			
			}else{
				usb_keyboard_press(offStates[0],0);	
			}
			btnUpState = buttonState;
			needsDebounce = 1;
		}
		
		// Down Button
		buttonState = (PINB & (1<<1));
		if (buttonState != btnDownState) {
			if(buttonState == 0){ 
				usb_keyboard_press(onStates[1],0);		
			}else{
				usb_keyboard_press(offStates[1],0);			 
			}
			btnDownState = buttonState;
			needsDebounce = 1;
		}
		
		// Left Button
		buttonState = (PINB & (1<<2));
		if (buttonState != btnLeftState) {
			if(buttonState == 0){ 
				usb_keyboard_press(onStates[2],0);				
			}else{
				usb_keyboard_press(offStates[2],0);	
			}
			btnLeftState = buttonState;
			needsDebounce = 1;
		}
		
		// Right Button
		buttonState = (PINB & (1<<3));
		if (buttonState != btnRightState) {
			if(buttonState == 0){ 
				usb_keyboard_press(onStates[3],0);		
			}else{
				usb_keyboard_press(offStates[3],0);	
			}
			btnRightState = buttonState;
			needsDebounce = 1;
		}
		
		// A Button
		buttonState = (PINB & (1<<4));
		if (buttonState != btnAState) {
			if(!btnInvertModeState){
				if(buttonState == 0){ 
					usb_keyboard_press(offStates[4],0);		
				}else{
					usb_keyboard_press(onStates[4],0);	
				}
			}else{
				if(buttonState == 0){ 
					usb_keyboard_press(onStates[4],0);		
				}else{
					usb_keyboard_press(offStates[4],0);	
				}
			}
			btnAState = buttonState;
			needsDebounce = 1;
		}
		
		// B Button
		buttonState = (PINB & (1<<5));
		if (buttonState != btnBState) {
			if(!btnInvertModeState){
				if(buttonState == 0){ 
					usb_keyboard_press(offStates[5],0);		
				}else{
					usb_keyboard_press(onStates[5],0);	
				}
			}else{
				if(buttonState == 0){ 
					usb_keyboard_press(onStates[5],0);		
				}else{
					usb_keyboard_press(offStates[5],0);	
				}
			}
			btnBState = buttonState;
			needsDebounce = 1;
		}
		
		// C Button
		buttonState = (PINB & (1<<6));
		if (buttonState != btnCState) {
			if(buttonState == 0){ 
				usb_keyboard_press(onStates[6],0);		
			}else{
				usb_keyboard_press(offStates[6],0);	
			}
			btnCState = buttonState;
			needsDebounce = 1;
		}
		
		// D Button
		buttonState = (PINB & (1<<7));
		if (buttonState != btnDState) {
			if(buttonState == 0){ 
				usb_keyboard_press(onStates[7],0);		
			}else{
				usb_keyboard_press(offStates[7],0);	
			}
			btnDState = buttonState;
			needsDebounce = 1;
		}
		
		// Ugly debounce, do someting about it ;-)
		if(needsDebounce == 1){
			needsDebounce = 0;
			_delay_ms(DEBOUNCE_TIME);
		}
	}
}



