#include <PS4USB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
//#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
PS4USB PS4(&Usb);

uint8_t oldL2Value, oldR2Value;


#define pin_In_S0 3 //th Select0
#define pin_In_S1 4 //tr Select1
#define DeadZone 10  //Dead area of analog stick

char DATA1_1, DATA1_2;
char DATA2_1, DATA2_2;
char DATA3_1, DATA3_2;
char DATA4_1, DATA4_2;
char DATA5_1, DATA5_2;
char DATA6_1, DATA6_2;
char DATAEND_1, DATAEND_2;

bool ACK_LINE = 0;
int  mode_Selector = 0; //0 is digital mode 1 is analog


void setup() {
 pinMode(pin_In_S0, INPUT);
 pinMode(pin_In_S1, INPUT);


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
 //DDRD = DDRD | B00000000;
  
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
 Serial.print(F("\r\nPS4 USB Library Started"));  
  
 //we need to wait for the saturn to startup, 1.5 seconds
 //delay(1500);
 
 //pull the ack line high on startup
 changeACKState();  
 
 delayMicroseconds(500);

 }


 void loop() {
 Usb.Task();
 
 //Changes the controller from Digital to Analog
 if (PS4.connected()) {
	if (PS4.getButtonClick(OPTIONS)){
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
		PS4.setLed(Blue);
	}
	else{
		emulateAnalogController();
		PS4.setLed(Green);
	}
 
 }
 }


 void emulateDigitalController(){

////////Handshake////////
 //we now start communication with the saturn
 
 // Wait for the Saturn to select controller
 while(digitalRead(pin_In_S0)==1){
 }
 delayMicroseconds(40);
 
 //waiting for the saturn
 while(digitalRead(pin_In_S1)==1 && digitalRead(pin_In_S0)==0){
 }
 
 //0000 = Digital  // 0001 = Analog
 //pulled low to 0000 to id itself as a digital controller
 PORTC &= ~ B00001111;
 
 //this is set to low to let the saturn know the controller is ready to id itself
 //digitalWrite(pin_ACK_Y4, LOW);
 changeACKState(); 
 
 //at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
 
 // Wait for Saturn to request Data Size
 while(digitalRead(pin_In_S1)==0 && digitalRead(pin_In_S0)==0){  
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
  
    /* while (TR_REQ == 1 && TH_SEL == 0) {}  // Wait for Saturn to request first half of Data1
    PORTC = DATA1_1;        // DR, DL, DD, DU
    TL_ACK = 0;             // Here is the data */
  
  // Wait for Saturn to request first half of Data1
 while(digitalRead(pin_In_S1)==1 && digitalRead(pin_In_S0)==0){
 }
 sendDataStateHigh();
 PORTC &= ~ DATA1_1;
 changeACKState();
 
 
 // Wait for Saturn to request second half of Data1
 while(digitalRead(pin_In_S1)==0 && digitalRead(pin_In_S0)==0){ 
 }
 sendDataStateHigh();
 PORTC &= ~ DATA1_2;
 changeACKState();
///// END FIRST BYTE

///// START SECOND BYTE

 
 //Wait for Saturn to request first half of Data2
 while(digitalRead(pin_In_S1)==1 && digitalRead(pin_In_S0)==0){   
 }
 sendDataStateHigh();
 PORTC &= ~ DATA2_1;
 changeACKState();
 

  // Wait for Saturn to request second half of Data2
 while(digitalRead(pin_In_S1)==0 && digitalRead(pin_In_S0)==0){     
 }
 sendDataStateHigh();
 PORTC &= ~ DATA2_2;
 changeACKState();
 
  
///// END SECOND BYTE

///// START THIRD BYTE, END OF DATA
 
 // Wait for Saturn to request first half End
 while(digitalRead(pin_In_S1)==1 && digitalRead(pin_In_S0)==0){ 
 }
 sendDataStateLow();
 changeACKState();
 
 
 // Wait for Saturn to request second half of End
 while(digitalRead(pin_In_S1)==0 && digitalRead(pin_In_S0)==0){
 }
 //Not needed since we are already in a low state for all output
 sendDataStateLow();
 PORTC |= DATAEND_2;
 changeACKState();

//// END THIRD BYTE
  

 while(digitalRead(pin_In_S0)==0){ 
 }
///// END COMMS WITH SATURN
 
 DATA1_1 =  B00000000;
 DATA1_2 =  B00000000;
 DATA2_1 =  B00000000;
 DATA2_2 =  B00000000;

 
  //If the Up button pressed
 if(PS4.getButtonPress(UP)){
	DATA1_1 |= B00000001;
 }//If the Down button pressed
 if (PS4.getButtonPress(DOWN)){
	DATA1_1 |= B00000010;
 }//If the Left button pressed
 if (PS4.getButtonPress(LEFT)){
	DATA1_1 |= B00000100;
 }//If the RIght button pressed
 if (PS4.getButtonPress(RIGHT)){
	DATA1_1 |=B00001000;
 }
 
 //If the B button pressed
 if(PS4.getButtonPress(CIRCLE)){
	DATA1_2 |= B00000001;
 }//If the C button pressed
 if (PS4.getButtonPress(L3)){
	DATA1_2 |= B00000010;
 }
 //If the A button pressed
 if (PS4.getButtonPress(CROSS)){
	DATA1_2 |= B00000100;
 }//If the Start button pressed
 if (PS4.getButtonClick(PS)){
	DATA1_2 |= B00001000;
 }
 
  //If the Z button pressed
if(PS4.getButtonClick(R3)){
	DATA2_1 |= B00000001;
 }
 //If the Y button pressed
 if (PS4.getButtonPress(TRIANGLE)){
	DATA2_1 |= B00000010;
 }//If the X button pressed
 if (PS4.getButtonPress(SQUARE)){
	DATA2_1 |= B00000100;
 }//If the Right trigger button pressed
 if (PS4.getButtonPress(R1)){
	DATA2_1 |= B00001000;
 }
 
 //If the Left trigger button pressed
  if (PS4.getButtonPress(L1)){
	DATA2_2 |= B00001000;
 }
 //////////////////////////////////////////////////////
 
}

void emulateAnalogController(){

 char first_nibble[4],second_nibble[4];
 bool button_clicked = false; 
 int analogReading;

////////Handshake////////
 //we now start communication with the saturn
 
 // Wait for the Saturn to select controller
 while(digitalRead(pin_In_S0)==1){ 
 }
 delayMicroseconds(40);
 
 //waiting for the saturn
 while(digitalRead(pin_In_S1)==1 && digitalRead(pin_In_S0)==0){ 
 }
 
 //0000 = Digital  // 0001 = Analog
 //pulled low to 0000 to id itself as a digital controller
 PORTC |= B00000001;
 
 //this is set to low to let the saturn know the controller is ready to id itself
 changeACKState(); 
 
 //at this stage in the sequence s1 goes high which is the saturn again requesting data from the controller
 
 // Wait for Saturn to request Data Size
 while(digitalRead(pin_In_S1)==0 && digitalRead(pin_In_S0)==0){   
 }
 sendDataStateLow();
 //we now set 0010, which means 2 Bytes for the remaining Data. 6 for the analog controller(0110)
 PORTC |= B0000110;

 //the ack line will now go high telling the saturn it is ready for the data to be read from the controller
 changeACKState(); 
 ///////Handshake ends///////
 
 ///// START FIRST BYTE
  
  // Wait for Saturn to request first half of Data1
 while(digitalRead(pin_In_S1)==1 && digitalRead(pin_In_S0)==0){   
 }
 sendDataStateHigh();
 PORTC &= ~ DATA1_1;
 changeACKState();
 
 
 // Wait for Saturn to request second half of Data1
 while(digitalRead(pin_In_S1)==0 && digitalRead(pin_In_S0)==0){    
 }
 sendDataStateHigh();
 PORTC &= ~ DATA1_2;
 changeACKState();
///// END FIRST BYTE

///// START SECOND BYTE
 
 //Wait for Saturn to request first half of Data2
 while(digitalRead(pin_In_S1)==1 && digitalRead(pin_In_S0)==0){  
 }
 sendDataStateHigh();
 PORTC &= ~ DATA2_1;
 changeACKState();


  // Wait for Saturn to request second half of Data2
 while(digitalRead(pin_In_S1)==0 && digitalRead(pin_In_S0)==0){
 }
 sendDataStateHigh();
 PORTC &= ~ DATA2_2;
 changeACKState();

///// END SECOND BYTE


 ///// START THIRD BYTE,
  
// Wait for Saturn to request first half of Data3
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==1){    
  }
  
 sendDataStateHigh();
 PORTC &= ~ DATA3_1;
 changeACKState();
 // Wait for Saturn to request second half of Data3
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==0){    
 }
 
 sendDataStateHigh();
 PORTC &= ~ DATA3_2;
 changeACKState();

//// END THIRD BYTE


 ///// START FOURTH BYTE,
 
// Wait for Saturn to request first half of Data4
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==1){    
 }
  
 sendDataStateHigh();
 PORTC &= ~ DATA4_1;
 changeACKState();
 // Wait for Saturn to request second half of Data4
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==0){    
 }
  
 sendDataStateHigh();
 PORTC &= ~ DATA4_2;
 changeACKState();
 ///// END FOURTH BYTE


 ///// START FIFTH BYTE,
 
// Wait for Saturn to request first half of Data5
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==1){    
 } 

 sendDataStateHigh();
 PORTC &= ~ DATA5_1;
 changeACKState();

 // Wait for Saturn to request second half of Data5
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==0){    
 }
  
 sendDataStateHigh();
 PORTC &= ~ DATA5_2;
 changeACKState();
///// END FIFTH BYTE
 
 ///// START SIXTH BYTE,

// Wait for Saturn to request first half of Data6
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==1){    
 }
  
 sendDataStateHigh();
 PORTC &= ~ DATA6_1;
 changeACKState();
// Wait for Saturn to request second half of Data6
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==0){    
 }
 sendDataStateHigh();
 PORTC &= ~ DATA6_2;
 changeACKState();
///// END SIXTH BYTE

////////////////////////////////////////////////////////////////////////////////
  
   
 ///// START SEVENTH BYTE, END OF DATA
  
  // Wait for Saturn to request first half End
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==1){    
 }
 sendDataStateLow();
 
 changeACKState();
  
  // Wait for Saturn to request second half of End
 while(digitalRead(pin_In_S0)==0 && digitalRead(pin_In_S1)==0){    
 }
 sendDataStateLow();
 PORTC |=  DATAEND_2;
 changeACKState();

///// END SEVENTH BYTE
   
   // Wait for the Saturn to deselect controller
 while(digitalRead(pin_In_S0)==0){    
 }
  
///// END COMMS WITH SATURN
 
 DATA1_1 =  B00000000;
 DATA1_2 =  B00000000;
 DATA2_1 =  B00000000;
 DATA2_2 =  B00000000;

 
  //If the Up button pressed
 if(PS4.getButtonPress(UP)){
	DATA1_1 |= B00000001;
 }//If the Down button pressed
 if (PS4.getButtonPress(DOWN)){
	DATA1_1 |= B00000010;
 }//If the Left button pressed
 if (PS4.getButtonPress(LEFT)){
	DATA1_1 |= B00000100;
 }//If the RIght button pressed
 if (PS4.getButtonPress(RIGHT)){
	DATA1_1 |=B00001000;
 }
 
 //If the B button pressed
 if(PS4.getButtonPress(CIRCLE)){
	DATA1_2 |= B00000001;
 }//If the C button pressed
 if (PS4.getButtonPress(L3)){
	DATA1_2 |= B00000010;
 }
 //If the A button pressed
 if (PS4.getButtonPress(CROSS)){
	DATA1_2 |= B00000100;
 }//If the Start button pressed
 if (PS4.getButtonClick(PS)){
	DATA1_2 |= B00001000;
 }
 
  //If the Z button pressed
if(PS4.getButtonClick(R3)){
	DATA2_1 |= B00000001;
 }
 //If the Y button pressed
 if (PS4.getButtonPress(TRIANGLE)){
	DATA2_1 |= B00000010;
 }//If the X button pressed
 if (PS4.getButtonPress(SQUARE)){
	DATA2_1 |= B00000100;
 }//If the Right trigger button pressed
 if (PS4.getButtonPress(R1)){
	DATA2_1 |= B00001000;
 }
 
 //If the Left trigger button pressed
 if (PS4.getButtonPress(L1)){
	DATA2_2 |= B00001000;
 }
 
 ///Data3_1
 analogReading = PS4.getAnalogHat(LeftHatX);
 if(analogReading >=(128+DeadZone) || analogReading <= (128-DeadZone)) {
	 analogReading = (255-analogReading);// since the analog stick give a reverse output to what the saturn needs	 
	getBinary(analogReading,first_nibble,second_nibble);
	DATA3_1 = changeToDataState(first_nibble); 
	button_clicked=true;
 }//x axis default in netural
 else{
	DATA3_1 = B00001000; 
 }
  ///Data3_2
  if(button_clicked){
	//writes the scondf half of the binary output
	DATA3_2 = changeToDataState(second_nibble);
	button_clicked=false;
 }//run only if there was no input from the dual shock controller
 else{
	//default for the next nibble when x axis is in netural position
	DATA3_2 = B00000000;
 }
 ///Data4_1
 analogReading = PS4.getAnalogHat(LeftHatY);
if(analogReading >= (128+DeadZone) || analogReading <= (128-DeadZone)) {
	analogReading = (255-analogReading); // since the analog stick give a reverse output to what the saturn needs
	getBinary(analogReading,first_nibble,second_nibble);
	DATA4_1 = changeToDataState(first_nibble);
	//Serial.print(F("\r\nAnalog Y, "));
	//Serial.print(analogReading);  
	button_clicked=true;
 }//x axis default in netural
 else{
	DATA4_1 = B00001000;
 }
 ///Data4_2
 if(button_clicked){
	DATA4_2 = changeToDataState(second_nibble);
	button_clicked=false;
 }//run only if there was no input from the dual shock controller
 else{
	//default for the next nibble when y axis is in netural position
	DATA4_2 = B00000000;
 }
 
 
 //Data5_1
  if(PS4.getAnalogButton(R2) != oldR2Value) {
	getBinary(PS4.getAnalogButton(R2),first_nibble,second_nibble);
	DATA5_1 = changeToDataState(first_nibble);  
	button_clicked=true;
 }//x axis default in netural
 else{
	DATA5_1 = B00000000;
 }
  ///Data5_2
  if(button_clicked){
	DATA5_2 = changeToDataState(second_nibble);
	oldR2Value = PS4.getAnalogButton(R2);
	button_clicked=false;
 }//run only if there was no input from the dual shock controller
 else{
	//default for the nibble when Right trigger is in netural position
	DATA5_2 = B00000000;
 }
 
 //Data6_1
  if(PS4.getAnalogButton(L2) != oldR2Value) {
	getBinary(PS4.getAnalogButton(L2),first_nibble,second_nibble);
	DATA6_1 |= changeToDataState(first_nibble);   
	oldR2Value = PS4.getAnalogButton(L2);
	button_clicked=true;
 }//x axis default in netural
 else{
	DATA6_1 = B00000000;
 }
  ///Data6_2
  if(button_clicked){
	DATA6_2 = changeToDataState(second_nibble);
	button_clicked=false;
 }//run only if there was no input from the dual shock controller
 else{
	//default for the nibble when Right trigger is in netural position
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
 void getBinary(int num, char *first_nibble, char* second_nibble)
 {
	for (int i = 7; i >= 0; --i, num >>= 1){
		if (i>3){
			second_nibble[i-4] = (num & 1) + '0';
		}
		else{
			first_nibble[i] = (num & 1) + '0';
		}
	}	
 }
 
 
//Not used at this time
/* 
int readS0(){
  if(PIND & (1<<B00001000)){
   return 1;
  }
  else{
  return 0;
  }
}

int readS1(){

  if(PIND & (1<<B00001000)){
   return 1;
  }
  else{
  return 0;
  }
}
*/