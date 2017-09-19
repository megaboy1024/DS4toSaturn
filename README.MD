## Playstation Dual Shock 3/4 Controller to Sega Saturn adapter.


This is released under the GNU General Public License.



![Alt text](/DS4toSat.jpg?raw=true "DS4 to Saturn")
 
 
 
 ##### What is new
  
  - Added Dual Shock 3 support(USB only for now)
  - Removed badly implemented sensitivity 
  - Eliminated the horrible bluetooth lag
  - 5v line from the controller port now needs to be connected to pin 5 of the Arduino 
 
 


 
 
#### What it is
This lets you use a PS3 and PS4 controller on your Sega Saturn.


#### How it works

This uses an Arduino and a USB shield to emulate the Sega Saturn controller, enabling you to use a PS3/PS4 controller or with a little editing to the source code, just about any controller that is compatible with the USB shield library.
You can toggle between analog and digital using the Options button on the DS4 and Select on the DS3. When the controller light is green it is set to analog. Blue is digital. The light would be red for 30 seconds on start-up if the battery is below 14% of its full charge.

There is of course no rumble support because the Saturn never supported rumble, but rudimentary support can be added eg. Say you are playing Quake you can add the vibrate code to the button mapped to the gun 
Trigger to simulate recoil for that specific game. The button will always vibrate the controller though if pressed while not in game.

Being that the Saturn controller has six face buttons and the Dual Shock has only four, the C button is mapped to R3 and Z to L3 on Dual Shock controller(not ideal for fighting games which the DS is a poor controller for anyway).


#### What's Needed

 - Arduino(I used the Arduino UNO rev 3)
 - USB Shield
 - Compatible bluetooth dongle(or you can use a usb cable)
 - Playstation 4 Dual Shock controller
 - Sega Saturn controller cable to connect the controller port to the Arduino


#### Putting it together

 - Pins A0-A3 on the Arduino is used for D0-D3 on the Saturn. Pin 3 on the Arduino is used for the TH line, pin 4 for the TR line, pin 5 is the 5v line and the ACK line is pin A4. DO NOT plug this into the Saturn as yet.   
 - Connect your shield and plug in your bluetooth dongle. Compile and flash your Arduino with the source code from here(don't forget to include the USB shield library).
 - Pair the controller with the Arduino.
 - Power on the Saturn and now plug the controller cable with the Arduino into the Saturn port.
 
 ##### Warning
 Powering on the Arduino while the Saturn is off may damage your Saturn (you will be sending 5v down the controller port when the Saturn is not in a state to handle it).



 #### Credit
 
Some of the code used here was built upon the previous work and help of user RDC @ [NFGgames.com](http://nfggames.com/forum2/index.php?topic=5055.msg33242#msg33242) Major props to him!!
Some minor code used from the USB Host Shield Library.




