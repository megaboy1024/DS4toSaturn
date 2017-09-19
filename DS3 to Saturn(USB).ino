
#include <PS3USB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
/* You can create the instance of the class in two ways */
PS3USB PS3(&Usb); // This will just create the instance
//PS3USB PS3(&Usb,0x00,0x15,0x83,0x3D,0x0A,0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch


#define DeadZone 10  //Dead area of analog stick


uint8_t  mode_Selector = 0;  //0 is digital mode 1 is analog
float sensitivity = 1.0; //set the sensitivity of the analog stick. Default is 1. Increment in .1's

char DATA1_1, DATA1_2;
char DATA2_1, DATA2_2;
char DATA3_1, DATA3_2;
char DATA4_1, DATA4_2;
char DATA5_1, DATA5_2;
char DATA6_1, DATA6_2;
char DATAEND_1, DATAEND_2;

bool ACK_LINE = 0;
bool power_on = false;


void setup() {


 DATA1_1 = B00001111;                 // DATA1_1
 DATA1_2 = B00001111;                 // DATA1_2
 DATA2_1 = B00001111;                 // DATA2_1
 DATA2_2 = B00001111;                 // DATA2_2
 DATA3_1 = B00001111;                 // DATA3_1
 DATA3_2 = B00001111;                 // DATA3_2 
 DATA4_1 = B00001111;                 // DATA4_1
 DATA4_2 = B00001111;                 // DATA4_2 
 DATA5_1 = B00001111;                 // DATA5_1
 DATA5_2 = B00001111;                 // DATA5_2
 DATA6_1 = B00001111;                 // DATA6_2
 DATA6_2 = B00001111;                 // DATA6_2
 DATAEND_1 = B00000000;
 DATAEND_2 = B00000001;

 DDRC = B00011111;

  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
 Serial.print(F("\r\nPS3 USB Library Started")); 
 
 
 if(managePowerState()){
 //pull the ack line high on startup
 changeACKState();
 }
 //we need to wait for the saturn to startup, 1.5 seconds
 delayMicroseconds(500);
 }

 void loop() {

 Usb.Task();
 
 power_on=managePowerState(); //handles powering on and off the saturn
 
 //only if power is on do we want to run this routine
 if(power_on){
	if (PS3.PS3Connected) {
	
		//Changes the controller from Digital to Analog
		if (PS3.getButtonClick(SELECT)){
			if(mode_Selector==0){
				mode_Selector=1; //change state to now operating in analog mode
				Serial.print(F("\r\nController switched to Analog"));
			}
			else{
				mode_Selector=0; //change state to now operating in digital mode
				Serial.print(F("\r\nController switched to Digital"));
			}
		}
		if(mode_Selector==0){
			emulateDigitalController();
			setLed(); //set led one on
		}
		else{
			emulateAnalogController();
			setLed(); //set led two on
		}
	}
	else{
		sendDataStateHigh(); //if no PS controller is attached we send the data lines to high emulating no controller for the Saturn
	}
 }
 else{
	sendDataStateLow();//we sent all the data out pin low as to not damage the saturn when its not powered on
 }
 
 }


 void emulateDigitalController(){

	////////Handshake////////
	//we now start communication with the saturn
	
	// Wait for the Saturn to select controller
	while(readS0()==1){
	} 
	
	delayMicroseconds(40);
	//waiting for the saturn
	while(readS1()==1 && readS0()==0){
	}
	
	//0000 = Digital  // 0001 = Analog
	//pulled low to 0000 to id itself as a digital controller
	PORTC &= ~ B00001111;
	
	//this is set to low to let the saturn know the controller is ready to id itself
	//digitalWrite(pin_ACK_Y4, LOW);
	changeACKState(); 
	
	//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
	
	// Wait for Saturn to request Data Size
	while(readS1()==0 && readS0()==0){
	//Serial.print(F("\r\nwaiting for Saturn 2")); 
	}
	//sendDataStateLow();
	//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
	PORTC |= B00000010;
	
	//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
	//digitalWrite(pin_ACK_Y4, HIGH);
	changeACKState(); 
	///////Handshake ends///////
	
	///// START FIRST BYTE
	
	// Wait for Saturn to request first half of Data1
	while(readS1()==1 && readS0()==0){
	}
	sendDataStateHigh();
	PORTC &= ~ DATA1_1;
	changeACKState();
	
	// Wait for Saturn to request second half of Data1
	while(readS1()==0 && readS0()==0){
	}
	sendDataStateHigh();
	PORTC &= ~ DATA1_2;
	changeACKState();
	///// END FIRST BYTE

 Usb.Task();
	
	//// START SECOND BYTE
	
	//Wait for Saturn to request first half of Data2
	while(readS1()==1 && readS0()==0){
	}
	sendDataStateHigh();
	PORTC &= ~ DATA2_1;
	changeACKState();
	
	// Wait for Saturn to request second half of Data2
	while(readS1()==0 && readS0()==0){
	}
	sendDataStateHigh();
	PORTC &= ~ DATA2_2;
	changeACKState();
	
	//// END SECOND BYTE
	
	//// START THIRD BYTE, END OF DATA
	
	// Wait for Saturn to request first half End
	while(readS1()==1 && readS0()==0){
	}
	sendDataStateLow();
	changeACKState();
	
	// Wait for Saturn to request second half of End
	while(readS1()==0 && readS0()==0){
	}
	//Not needed since we are already in a low state for all output
	//sendDataStateLow();
	PORTC |= DATAEND_2;
	changeACKState();
	
	/// END THIRD BYTE
Usb.Task();
	while(readS0()==0){ 
	}
	
	//// END COMMS WITH SATURN
	
	DATA1_1 =  B00000000;
	DATA1_2 =  B00000000;
	DATA2_1 =  B00000000;
	DATA2_2 =  B00000000;
	
	//If the Up button pressed
	if(PS3.getButtonPress(UP)){
		DATA1_1 |= B00000001;
	}//If the Down button pressed
	if (PS3.getButtonPress(DOWN)){
		DATA1_1 |= B00000010;
	}//If the Left button pressed
	if (PS3.getButtonPress(LEFT)){
		DATA1_1 |= B00000100;
	}//If the RIght button pressed
	if (PS3.getButtonPress(RIGHT)){
		DATA1_1 |=B00001000;
	}
	
	//If the B button pressed
	if(PS3.getButtonPress(CIRCLE)){
		DATA1_2 |= B00000001;
	}//If the C button pressed
	if (PS3.getButtonPress(R3)){
		DATA1_2 |= B00000010;
	}
	//If the A button pressed
	if (PS3.getButtonPress(CROSS)){
		DATA1_2 |= B00000100;
	}//If the Start button pressed
	if (PS3.getButtonPress(START)){
		DATA1_2 |= B00001000;
	}
	
	//If the Z button pressed
	if(PS3.getButtonPress(L3)){
		DATA2_1 |= B00000001;
	}
	//If the Y button pressed
	if (PS3.getButtonPress(TRIANGLE)){
		DATA2_1 |= B00000010;
	}//If the X button pressed
	if (PS3.getButtonPress(SQUARE)){
		DATA2_1 |= B00000100;
	}//If the Right trigger button pressed
	if (PS3.getButtonPress(R1)){
		DATA2_1 |= B00001000;
	}
	
	//If the Left trigger button pressed
	if (PS3.getButtonPress(L1)){
		DATA2_2 |= B00001000;
	}

}
void emulateAnalogController(){

 char first_nibble[4];
 char second_nibble[4];
 bool button_clicked = false; 
 int analogReading;

	////////Handshake////////
	//we now start communication with the saturn
	
	// Wait for the Saturn to select controller
	while(readS0()==1){ 
	}
	delayMicroseconds(40);
	//waiting for the saturn
	while(readS1()==1 && readS0()==0){ 
	}
	
	//0000 = Digital  // 0001 = Analog
	//pulled low to 0000 to id itself as a digital controller
	PORTC |= B00000001;
	
	//this is set to low to let the saturn know the controller is ready to id itself
	changeACKState(); 
	
	//at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
	
	// Wait for Saturn to request Data Size
	while(readS1()==0 && readS0()==0){
	}
	sendDataStateLow();
	//we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
	PORTC |= B0000110;
	
	//the ack line will now go high telling the saturn it is ready for the data to be read from the controller
	changeACKState(); 
	///////Handshake ends///////
	
	///// START FIRST BYTE
	
	// Wait for Saturn to request first half of Data1
	while(readS1()==1 && readS0()==0){
	}
	sendDataStateHigh();
	PORTC &= ~ DATA1_1;
	changeACKState();
	
	// Wait for Saturn to request second half of Data1
	while(readS1()==0 && readS0()==0){
	}
	sendDataStateHigh();
	PORTC &= ~ DATA1_2;
	changeACKState();
	///// END FIRST BYTE
Usb.Task();

	///// START SECOND BYTE
	//Wait for Saturn to request first half of Data2
	while(readS1()==1 && readS0()==0){
	}
	sendDataStateHigh();
	PORTC &= ~ DATA2_1;
	changeACKState();
	
	// Wait for Saturn to request second half of Data2
	while(readS1()==0 && readS0()==0){
	}
	sendDataStateHigh();
	PORTC &= ~ DATA2_2;
	changeACKState();
	
	///// END SECOND BYTE

	///// START THIRD BYTE,
	
	// Wait for Saturn to request first half of Data3
	while(readS0()==0 && readS1()==1){
	}
	
	sendDataStateHigh();
	PORTC &= ~ DATA3_1;
	changeACKState();
	// Wait for Saturn to request second half of Data3
	while(readS0()==0 && readS1()==0){
	}
	
	sendDataStateHigh();
	PORTC &= ~ DATA3_2;
	changeACKState();
	
	//// END THIRD BYTE

	///// START FOURTH BYTE,
	
	// Wait for Saturn to request first half of Data4
	while(readS0()==0 && readS1()==1){
	}
	
	sendDataStateHigh();
	PORTC &= ~ DATA4_1;
	changeACKState();
	// Wait for Saturn to request second half of Data4
	while(readS0()==0 && readS1()==0){
	}
	
	sendDataStateHigh();
	PORTC &= ~ DATA4_2;
	changeACKState();
	///// END FOURTH BYTE
	
	
	///// START FIFTH BYTE,
	
	// Wait for Saturn to request first half of Data5
	while(readS0()==0 && readS1()==1){
	} 
	
	sendDataStateHigh();
	PORTC &= ~ DATA5_1;
	changeACKState();
	
	// Wait for Saturn to request second half of Data5
	while(readS0()==0 && readS1()==0){
	}
	
	sendDataStateHigh();
	PORTC &= ~ DATA5_2;
	changeACKState();
	///// END FIFTH BYTE
 Usb.Task();
	///// START SIXTH BYTE,
	
	// Wait for Saturn to request first half of Data6
	while(readS0()==0 && readS1()==1){
	}
	
	sendDataStateHigh();
	PORTC &= ~ DATA6_1;
	changeACKState();
	// Wait for Saturn to request second half of Data6
	while(readS0()==0 && readS1()==0){
	}
	sendDataStateHigh();
	PORTC &= ~ DATA6_2;
	changeACKState();
	///// END SIXTH BYTE
	
	///// START SEVENTH BYTE, END OF DATA
	// Wait for Saturn to request first half End
	while(readS0()==0 && readS1()==1){
	}
	sendDataStateLow();
	changeACKState();
	
	// Wait for Saturn to request second half of End
	while(readS0()==0 && readS1()==0){
	}
	//sendDataStateLow();
	PORTC |=  DATAEND_2;
	changeACKState();
	
	///// END SEVENTH BYTE
	
	// Wait for the Saturn to deselect controller
	while(readS0()==0){    
	}
	///// END COMMS WITH SATURN
	
	DATA1_1 =  B00000000;
	DATA1_2 =  B00000000;
	DATA2_1 =  B00000000;
	DATA2_2 =  B00000000;
	
	
	//If the Up button pressed
	if(PS3.getButtonPress(UP)){
		DATA1_1 |= B00000001;
	}//If the Down button pressed
	if (PS3.getButtonPress(DOWN)){
		DATA1_1 |= B00000010;
	}//If the Left button pressed
	if (PS3.getButtonPress(LEFT)){
		DATA1_1 |= B00000100;
	}//If the RIght button pressed
	if (PS3.getButtonPress(RIGHT)){
		DATA1_1 |=B00001000;
	}
	
	//If the B button pressed
	if(PS3.getButtonPress(CIRCLE)){
		DATA1_2 |= B00000001;
	}//If the C button pressed
	if (PS3.getButtonPress(R3)){
		DATA1_2 |= B00000010;
	}
	//If the A button pressed
	if (PS3.getButtonPress(CROSS)){
		DATA1_2 |= B00000100;
	}//If the Start button pressed
	if (PS3.getButtonPress(START)){
		DATA1_2 |= B00001000;
	}
	
	//If the Z button pressed
	if(PS3.getButtonPress(L3)){
		DATA2_1 |= B00000001;
	}
	//If the Y button pressed
	if (PS3.getButtonPress(TRIANGLE)){
		DATA2_1 |= B00000010;
	}//If the X button pressed
	if (PS3.getButtonPress(SQUARE)){
		DATA2_1 |= B00000100;
	}//If the Right trigger button pressed
	if (PS3.getButtonPress(R1)){
		DATA2_1 |= B00001000;
	}
	
	//If the Left trigger button pressed
	if (PS3.getButtonPress(L1)){
		DATA2_2 |= B00001000;
	}
 Usb.Task();
	///Data3_1
	analogReading = PS3.getAnalogHat(LeftHatX);
	if(analogReading >=(127+DeadZone) || analogReading <= (127-DeadZone)) {
		analogReading = (255-analogReading);// since the analog stick gives a reverse output to what the saturn needs we need to subtract the reading from 255 
		getBinary(analogReading,first_nibble,second_nibble);
		DATA3_1 = changeToDataState(first_nibble); 
		button_clicked=true;
	}//x axis default in neutral
	else{
		DATA3_1 = B00001000; 
	}
	///Data3_2
	if(button_clicked){
		//writes the second half of the binary output
		DATA3_2 = changeToDataState(second_nibble);
		button_clicked=false;
	}//run only if there was no input from the dual shock controller
	else{
		//default for the next nibble when x axis is in neutral position
		DATA3_2 = B00000000;
	}
	
	//Data5_1
	analogReading = PS3.getAnalogButton(R2);
	if(analogReading >0) {
		getBinary(analogReading,first_nibble,second_nibble);
		DATA5_1 = changeToDataState(first_nibble);
		button_clicked=true;
	}//x axis default in neutral
	else{
		DATA5_1 = B00000000;
	}
	///Data5_2
	if(button_clicked){
		DATA5_2 = changeToDataState(second_nibble);
		button_clicked=false;
	}//run only if there was no input from the dual shock controller
	else{
		//default for the nibble when Right trigger is in neutral position
		DATA5_2 = B00000000;
	}

	///////////////////////////////
 Usb.Task();
	///Data4_1
	analogReading = PS3.getAnalogHat(LeftHatY);
	if(analogReading >= (127+DeadZone) || analogReading <= (127-DeadZone)) {
		analogReading = (255-analogReading); // since the analog stick give a reverse output to what the saturn needs we need to subtract the reading from 255
		getBinary(analogReading,first_nibble,second_nibble);
		DATA4_1 = changeToDataState(first_nibble);
		button_clicked=true;
	}//x axis default in neutral
	else{
		DATA4_1 = B00001000;
	}
	///Data4_2
	if(button_clicked){
		DATA4_2 = changeToDataState(second_nibble);
		button_clicked=false;
	}//run only if there was no input from the dual shock controller
	else{
		//default for the next nibble when y axis is in neutral position
		DATA4_2 = B00000000;
	}
	
	//Data6_1
	analogReading = PS3.getAnalogButton(L2);
	if(analogReading >0) {
		getBinary(analogReading,first_nibble,second_nibble);
		DATA6_1 = changeToDataState(first_nibble);   
		button_clicked=true;
	}//x axis default in neutral
	else{
		DATA6_1 = B00000000;
	}
	///Data6_2
	if(button_clicked){
		DATA6_2 = changeToDataState(second_nibble);
		button_clicked=false;
	}//run only if there was no input from the dual shock controller
	else{
		//default for the nibble when Right trigger is in neutral position
		DATA6_2 = B00000000;
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

 //Turn the char array nibbles into pinout data
 char changeToDataState(char input[]){
	char tempDATA =  B00000000;
	for (int i = 0; i < 4; i++){
		if (i == 3){
			if (input[i] == '0'){
				tempDATA |= B00000000;
			}
			else{
				tempDATA |= B00000001;
			}
		}
		else if (i == 2){
			if (input[i] == '0'){
				tempDATA |= B00000000;
			}
			else{
				tempDATA |= B00000010;
			}
		}
		else if (i == 1){
			if (input[i] == '0'){
				tempDATA |= B00000000;
			}
			else{
				tempDATA |= B00000100;
			}
		}
		else if (i == 0){
			if (input[i] == '0'){
				tempDATA |= B00000000;
			}
			else{
				tempDATA |= B00001000;
			}
		}
	}
	return tempDATA;
 }

 //here we convert the integer reading from the DS4 controller to a binary number and split it into two nibbles
 void getBinary(int num, char *first_nibble, char* second_nibble){
	
	for (int i = 7; i >= 0; --i, num >>= 1){
		if (i>3){
			second_nibble[i-4] = (num & 1) + '0';
		}
		else{
			first_nibble[i] = (num & 1) + '0';
		}
	}	
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





void setLed(){

    if(mode_Selector==0){
        PS3.setLedOff();
		PS3.setLedOn(LED1); //digital mode
    }
    else{
        PS3.setLedOff();
		PS3.setLedOn(LED2); //analog mode
    }

}