## Playstation Dual Shock 4 Controller to Sega Saturn adapter.

This is released under the GNU General Public License.

#### What it is
This lets you use a PS4 controller on your Sega Saturn.


#### How it works

This uses an Arduino and a USB shield to emulate the Sega Saturn controller, enabling you to use a PS4 controller or with a little editing to the source code, just about any controller that is compatible with the USB shield library.
You can toggle between analog and digital using the Options button. When the controller light is green it is set to analog. Blue is digital.
There is of course no rumble support because the Saturn never supported rumble, but rudimentary support can be added eg. Say you are playing Quake you can add the virbate code to the button mapped to the gun 
trigger to simulate recoil for that specific game. The button will always virbate the controller though if pressed while not in game.


#### Needed

 - Arduino(I used the Arduino UNO rev 3)
 - USB Shield
 - Compatible bluetooth dongle(or you can use a usb cable)
 - Playstation 4 Dual Shock controller
 - Sega Saturn controller cable to connect the controller port to the Arduino


#### Putting it Together

 - Pins A0-A3 on the Arduino is used for D0-D3 on the Saturn. Pin 3 on the Aruduino is used for the TH line, pin 4 for the TR line and the ACK line is pin A4. DO NOT plug this into the saturn as yet.   
 - Connect your shield and plug in your bluetooth dongle. Compile and flash your arudino with the source code from here(don't forget to include the USB shield libarary).
 - Pair the controller with the Arduino.
 - Power on the Saturn and now plug the controller cable with the Arduino into the Saturn port.
 
 #### Warming
 Powering on the Arduino while the Saturn is off may damage your Saturn(you will be sending 5v down the controller port when the Saturn is not in a state to handle it).


 #### Credit
 
Some of the code used here was built upon the previous work and help of user RDC @ [NFGgames.com](http://nfggames.com/forum2/index.php?topic=5055.msg33242#msg33242) Major props to him!!
Some minor code used from the USB Host Shield Library.