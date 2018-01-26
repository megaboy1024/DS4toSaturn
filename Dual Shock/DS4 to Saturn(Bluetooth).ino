#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
//#include <spi4teensy3.h>
#include <SPI.h>
#endif 


USB Usb;
//USBHub Hub1(&Usb); // For dongle with an included hub
BTD Btd(&Usb); // Bluetooth instance
PS4BT PS4(&Btd, PAIR); //this creates instance of the PS4BT class
//Pair your Dual Shock controller the usual way by holding the PS button plus the share button

#define dead_zone 10  //Dead area of analog stick
#define analog_digital_dead_zone 15
uint8_t mode_Selector = 0;  //0 is digital mode 1 is analog

unsigned char DATA1_1, DATA1_2;
unsigned char DATA2_1, DATA2_2;
unsigned char DATA3_1, DATA3_2;
unsigned char DATA4_1, DATA4_2;
unsigned char DATA5_1, DATA5_2;
unsigned char DATA6_1, DATA6_2;
unsigned char DATAEND_1, DATAEND_2;

bool ACK_LINE = 0;
bool battery_status = false;
bool power_on = false;
long battery_timer_start;


void setup() {

 DATA1_1 = B00001111;                 // DATA1_1
 DATA1_2 = B00001111;                 // DATA1_2
 DATA2_1 = B00001111;                 // DATA2_1
 DATA2_2 = B00001111;                 // DATA2_2
 DATA3_1 = B00000111;                 // DATA3_1
 DATA3_2 = B00001111;                 // DATA3_2 
 DATA4_1 = B00000111;                 // DATA4_1
 DATA4_2 = B00001111;                 // DATA4_2 
 DATA5_1 = B00001111;                 // DATA5_1
 DATA5_2 = B00001111;                 // DATA5_2
 DATA6_1 = B00001111;                 // DATA6_2
 DATA6_2 = B00001111;                 // DATA6_2
 DATAEND_1 = B00000000;
 DATAEND_2 = B00000001;

 

 DDRC |= B00011111;

  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
 Serial.print(F("\r\nPS4 USB Library Started")); 
 
 
 if(managePowerState()){
	//pull the ack line high on startup
	changeACKState();
 }
 //we need to wait for the saturn to startup, 1.5 seconds
 delayMicroseconds(500);
 battery_timer_start = millis();
 }

 void loop() {

  Usb.Task();
 power_on=managePowerState(); //handles powering on and off the saturn
 
 //only if power is on do we want to run this routine
 if(power_on){
	if (PS4.connected()) {
	
		checkBatteryStatus();
		//Changes the controller from Digital to Analog
		if (PS4.getButtonClick(OPTIONS)){
			if(mode_Selector==0){
				mode_Selector=1; //change state to now operating in analog mode
				Serial.print(F("\r\nController switched to Analog"));
			}
			else if(mode_Selector==1){
				mode_Selector=2; //change state to now operating in digital analog mode
				Serial.print(F("\r\nController switched to Digital with stick enabled"));
			}
			else{
				mode_Selector=0; //change state to now operating in digital mode
				Serial.print(F("\r\nController switched to Digital"));
			}
		}
		if(mode_Selector==0){
			emulateDigitalController(); 
			setLedColor(); //led would change to blue
		}
		else if(mode_Selector==1) {
			emulateAnalogController();
			setLedColor(); //led would change to green
		}
		else{
			emulateDigitalController();//With functional stickS
			setLedColor(); //led would change to magenta
		}
	}
	else{
		//sendDataStateHigh(); //if no PS controller is attached we send the data lines to high emulating no controller for the Saturn
		//emulateUSB
	}
 }
 else{
	sendDataStateLow();//we sent all the data out pin low as to not damage the saturn when its not powered on
 }

 }

//this entire function takes 378 us once controller s0 goes low
void emulateDigitalController(){
	
////////Handshake////////
	//we now start communication with the saturn
	// Wait for the Saturn to select controller
	if(readS0()==0){
		delayMicroseconds(40);
		//waiting for the saturn
		while(readS1()==1 && readS0()==0){
		}
	
		//0000 = Digital  // 0001 = Analog
		//pulled low to 0000 to id itself as a digital controller
		PORTC =  B00000000;
		
		//this is set to low to let the saturn know the controller is ready to id itself
		changeACKState(); 
		
		//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
		
		// Wait for Saturn to request Data Size
		
		while(readS1()==0 && readS0()==0){
		}
		//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
		PORTC = B00000010;
		
		//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
		changeACKState(); 
		///////Handshake ends///////
		
		///// START FIRST BYTE
		
		// Wait for Saturn to request first half of Data1
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATA1_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data1
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATA1_2;
		changeACKState();
		///// END FIRST BYTE
		
		//// START SECOND BYTE
		
		//Wait for Saturn to request first half of Data2
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATA2_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data2
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATA2_2;
		changeACKState();
		//// END SECOND BYTE
		
		//// START THIRD BYTE, END OF DATA
		
		// Wait for Saturn to request first half End
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATAEND_1;
		changeACKState();
		// Wait for Saturn to request second half of End
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATAEND_2;
		changeACKState();
		
		/// END THIRD BYTE
		// Wait for the Saturn to deselect controller	
		//This wait is 104us and was removed so that we begin to poll the Dual Shock controller input earlier. 
		//while(readS0()==0){}
		
		DATA1_1 =  B00001111;
		DATA1_2 =  B00001111;
		DATA2_1 =  B00001111;
		DATA2_2 =  B00001111;
		
		//polling the Dual Shock takes  about 76us
		
		uint8_t analogReading;
		if(mode_Selector==2){
			analogReading = PS4.getAnalogHat(LeftHatX);
			if(analogReading >=(127+analog_digital_dead_zone) || analogReading <= (127-analog_digital_dead_zone)) {
				//Right
				if(analogReading>=127+analog_digital_dead_zone){
					DATA1_1 &= B00000111;
				}
				//Left
				if(analogReading<= 127-analog_digital_dead_zone){
					DATA1_1 &= B00001011;
				}
			}
			
			analogReading = PS4.getAnalogHat(LeftHatY);
			if(analogReading >=(127+analog_digital_dead_zone) || analogReading <= (127-analog_digital_dead_zone)) {
				//Up
				if(analogReading<= 127-analog_digital_dead_zone){
					DATA1_1 &= B00001110;
				}
				//Down
				if(analogReading>=127+analog_digital_dead_zone){
					DATA1_1 &= B00001101;
				}
			}
		}
		
			
		//If the Up button pressed
		if(PS4.getButtonPress(UP)){DATA1_1 &= B00001110;}
		//If the Down button pressed
		if(PS4.getButtonPress(DOWN)){DATA1_1 &= B00001101;}
		//If the Left button pressed
		if(PS4.getButtonPress(LEFT)){DATA1_1 &= B00001011;}
		//If the Right button pressed
		if(PS4.getButtonPress(RIGHT)){DATA1_1 &=B00000111;}
		
		//If the B button pressed
		if(PS4.getButtonPress(CIRCLE)){DATA1_2 &= B00001110;}
		//If the C button pressed
		if(PS4.getButtonPress(R3)){DATA1_2 &= B00001101;}
		//If the A button pressed
		if(PS4.getButtonPress(CROSS)){DATA1_2 &= B00001011;}
		//If the Start button pressed
		if (PS4.getButtonPress(PS)){DATA1_2 &= B00000111;}
		
		//If the Z button pressed
		if(PS4.getButtonPress(L3)){DATA2_1 &= B00001110;}
		//If the Y button pressed
		if(PS4.getButtonPress(TRIANGLE)){DATA2_1 &= B00001101;}
		//If the X button pressed
		if(PS4.getButtonPress(SQUARE)){DATA2_1 &= B00001011;}
		//If the Right trigger button pressed
		if(PS4.getButtonPress(R1)){DATA2_1 &= B00000111;}
		
		//If the Left trigger button pressed
		if(PS4.getButtonPress(L1)){DATA2_2 &= B00000111;}
	
		delayMicroseconds(28);//added since we need to be at 104us wait average
	}
	
}
//this entire function takes 650 us once controller s0 goes low
void emulateAnalogController(){

	////////Handshake////////
	//we now start communication with the saturn
	// Wait for the Saturn to select controller
	if(readS0()==0){
		delayMicroseconds(40);
		
		//waiting for the saturn
		while(readS1()==1 && readS0()==0){ 
		}
		
		//0000 = Digital  // 0001 = Analog
		//pulled low to 0000 to id itself as a digital controller
		PORTC = B00000001;
		
		//this is set to low to let the saturn know the controller is ready to id itself
		changeACKState(); 
		
		//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
		
		// Wait for Saturn to request Data Size
		while(readS1()==0 && readS0()==0){
		}
		//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
		PORTC = B0000110;
		
		//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
		changeACKState(); 
		///////Handshake ends///////
		
		///// START FIRST BYTE
		
		// Wait for Saturn to request first half of Data1
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATA1_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data1
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATA1_2;
		changeACKState();
		///// END FIRST BYTE
		
		
		///// START SECOND BYTE
		//Wait for Saturn to request first half of Data2
		while(readS1()==1 && readS0()==0){
		}
		PORTC = DATA2_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data2
		while(readS1()==0 && readS0()==0){
		}
		PORTC = DATA2_2;
		changeACKState();
		
		///// END SECOND BYTE
		
		///// START THIRD BYTE,
		
		// Wait for Saturn to request first half of Data3
		while(readS0()==0 && readS1()==1){
		}
		PORTC = DATA3_1;
		changeACKState();
		// Wait for Saturn to request second half of Data3
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATA3_2;
		changeACKState();
		//Usb.Task(); //new	
		//// END THIRD BYTE
		
		///// START FOURTH BYTE,
		
		// Wait for Saturn to request first half of Data4
		while(readS0()==0 && readS1()==1){
		}
		PORTC = DATA4_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data4
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATA4_2;
		changeACKState();
		///// END FOURTH BYTE
		
		///// START FIFTH BYTE,
		
		// Wait for Saturn to request first half of Data5
		while(readS0()==0 && readS1()==1){
		} 
		PORTC = DATA5_1;
		changeACKState();
		
		// Wait for Saturn to request second half of Data5
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATA5_2;
		changeACKState();
		
		///// END FIFTH BYTE

		///// START SIXTH BYTE,
		
		// Wait for Saturn to request first half of Data6
		while(readS0()==0 && readS1()==1){
		}
		PORTC = DATA6_1;
		changeACKState();
		// Wait for Saturn to request second half of Data6
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATA6_2;
		changeACKState();
		///// END SIXTH BYTE
		
		///// START SEVENTH BYTE, END OF DATA
		// Wait for Saturn to request first half End
		while(readS0()==0 && readS1()==1){
		}
		PORTC = DATAEND_1;
		changeACKState();
		
		// Wait for Saturn to request second half of End
		while(readS0()==0 && readS1()==0){
		}
		PORTC = DATAEND_2;
		changeACKState();
		
		///// END SEVENTH BYTE
		
		// Wait for the Saturn to deselect controller
		/*while(readS0()==0){
		}*/
		
		///// END COMMS WITH SATURN
		
		DATA1_1 =  B00001111;
		DATA1_2 =  B00001111;
		DATA2_1 =  B00001111;
		DATA2_2 =  B00001111;
		
		bool button_clicked = false; 
		uint8_t analogReading;
		
		//If the Up button pressed
		if(PS4.getButtonPress(UP)){DATA1_1 &= B00001110;}
		//If the Down button pressed
		if(PS4.getButtonPress(DOWN)){DATA1_1 &= B00001101;}
		//If the Left button pressed
		if(PS4.getButtonPress(LEFT)){DATA1_1 &= B00001011;}
		//If the Right button pressed
		if(PS4.getButtonPress(RIGHT)){DATA1_1 &=B00000111;}
		
		//If the B button pressed
		if(PS4.getButtonPress(CIRCLE)){DATA1_2 &= B00001110;}
		//If the C button pressed
		if(PS4.getButtonPress(R3)){DATA1_2 &= B00001101;}
		//If the A button pressed
		if(PS4.getButtonPress(CROSS)){DATA1_2 &= B00001011;}
		//If the Start button pressed
		if (PS4.getButtonPress(PS)){DATA1_2 &= B00000111;}
		
		//If the Z button pressed
		if(PS4.getButtonPress(L3)){DATA2_1 &= B00001110;}
		//If the Y button pressed
		if(PS4.getButtonPress(TRIANGLE)){DATA2_1 &= B00001101;}
		//If the X button pressed
		if(PS4.getButtonPress(SQUARE)){DATA2_1 &= B00001011;}
		//If the Right trigger button pressed
		if(PS4.getButtonPress(R1)){DATA2_1 &= B00000111;}
		
		//If the Left trigger button pressed
		if(PS4.getButtonPress(L1)){DATA2_2 &= B00000111;}
		
		///Data3_1
		analogReading = PS4.getAnalogHat(LeftHatX);
		if(analogReading >=(127+dead_zone) || analogReading <= (127-dead_zone)) {
			getBinary(analogReading,DATA3_1,DATA3_2);
			button_clicked=true;
		}//x axis default in neutral
		else{
			DATA3_1 = B00000111; 
		}
		///Data3_2
		if(button_clicked){
			//writes the second half of the binary output
			button_clicked=false;
		}//run only if there was no input from the dual shock controller
		else{
			//default for the next nibble when x axis is in neutral position
			DATA3_2 = B00001111;
		}
		///Data4_1
		analogReading = PS4.getAnalogHat(LeftHatY);
		if(analogReading >= (127+dead_zone) || analogReading <= (127-dead_zone)) {
			getBinary(analogReading,DATA4_1,DATA4_2);
			button_clicked=true;
		}//x axis default in neutral
		else{
			DATA4_1 = B00000111;
		}
		///Data4_2
		if(button_clicked){
			button_clicked=false;
		}//run only if there was no input from the dual shock controller
		else{
			//default for the next nibble when y axis is in neutral position
			DATA4_2 = B00001111;
		}
		//Data5_1
		analogReading = PS4.getAnalogButton(R2);
		if(analogReading >0) {
			analogReading = (255-analogReading);
			getBinary(analogReading,DATA5_1,DATA5_2);
			button_clicked=true;
		}//x axis default in neutral
		else{
			DATA5_1 = B00001111;
		}
		///Data5_2
		if(button_clicked){
			button_clicked=false;
		}//run only if there was no input from the dual shock controller
		else{
			//default for the nibble when Right trigger is in neutral position
			DATA5_2 = B00001111;
		}
			
		//Data6_1
		analogReading = PS4.getAnalogButton(L2);
		if(analogReading >0) {
			analogReading = (255-analogReading);
			getBinary(analogReading,DATA6_1,DATA6_2);
			button_clicked=true;
		}//x axis default in neutral
		else{
			DATA6_1 = B00001111;
		}
		///Data6_2
		if(button_clicked){
			button_clicked=false;
		}//run only if there was no input from the dual shock controller
		else{
			//default for the nibble when Right trigger is in neutral position
			DATA6_2 = B00001111;
		}
	}	
}


//use to determine the status of the ACK return line and send back the opposite.eg. If its low now then return a high;
 void changeACKState(){
  if(ACK_LINE==0){
	PORTC|=B00010000;
	ACK_LINE =1;
  }
  else{
	PORTC &=~ B00010000;
	ACK_LINE =0;
  }
 }


 void sendDataStateLow(){
  PORTC &= ~B00001111;
 }
 
 void sendDataStateHigh(){
  PORTC |= B00001111;
 }

 void getBinary(uint8_t num, unsigned char &first_nibble, unsigned char &second_nibble){
	unsigned char temp;
	temp= char(num);
	second_nibble = (second_nibble & 0b11110000) | (temp & 0b00001111);
	temp = temp>>4;
	first_nibble = (first_nibble & 0b11110000) | (temp & 0b00001111);
 }
 
bool readS0(){
 if(PIND & (1<<PD3)){
	return 1;
 }
 else{
	return 0;
 }
}

bool readS1(){
 if(PIND & (1<<PD4)){
	return 1;
 }
 else{
	return 0;
 }
}

//checks if the saturn is powered on by reading the 5v line that would have been used on the standard controller for power
bool readPowerState(){
 if(PIND & (1<<PD5)){
	return 1;
 }
 else{
	return 0;
 }
}

 bool managePowerState(){

	if(readPowerState()==1){
		return true;
	}
	else{
		return false;
	}
 }

//checks the battery status and sets led red for 1/2 a minute if battery is less than 14% of its full charge
void checkBatteryStatus(){
    if(battery_status==false){
        if(PS4.getBatteryLevel() < 2){
            long battery_timer_elapsed = millis()-battery_timer_start;
            if(battery_timer_elapsed >=30000){
                battery_status=true;
            }
        }
        else{
            battery_status=true;
        }
    }
}

void setLedColor(){

	if(!power_on){
		PS4.disconnect();
	}
     else if(battery_status == false){
        PS4.setLed(Red); //battery weak
    }
    else if(mode_Selector==0){
        PS4.setLed(Blue); //digital mode
    }
	else if(mode_Selector==1){
		PS4.setLed(Green); //analog mode
	}
    else if(mode_Selector==2){
        PS4.setLed(255,0,255);// digital analog mode
    }

}