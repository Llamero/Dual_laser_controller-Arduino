//Import the SPI library
#include <SPI.h>;

// Define various ADC prescaler
//const unsigned char PS_2 = (1 << ADPS0); //Does not seem to work, ADC stops.
const unsigned char PS_4 = (1 << ADPS1);
//const unsigned char PS_8 = (1 << ADPS1) | (1 << ADPS0);
//const unsigned char PS_16 = (1 << ADPS2);
//const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
//const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //Default pre-scaler in Arduino
const uint8_t d2 = 2; //ms delay in display SPI between commands
const float gammaInt = 2; //Gamma exponent for intensity
const float gammaTime = 3; //Gamma exponent for time

uint8_t Bmode = 0; //Saves the current mode of the blue laser (modulo-4) 0 = Manual; 1 = Scanimage sync; 2 = Ext. Trigger; 3 = Scanimage trigger
uint8_t Rmode = 0; //Saves the current mode of the red laser 0 = Manual; 1 = Scanimage sync; 2 = Ext. Trigger; 3 = Scanimage trigger
boolean Ron = 0; //Whether red laser is on
boolean Bon = 0; //Whether blue laser is on
char state[21] = "     ms     % Unlock";
uint16_t intB = 4; //Saves blue laser intensity 0-4095
uint16_t intR = 4; //Saves red laser intensity 0-4095
uint16_t timeR = 100; //Time for red laser to stay on
uint16_t timeB = 1000; //Time for blue laser to stay on
uint8_t buttonIntB = 70; //Saves on intensity of blue button
uint8_t buttonIntR = 255; //Saves on intensity of red button
uint8_t buttonIn = 0; //Saves the button command 0 = red On, 1 = blue On, 2 = red program, 3 = blue program, 4 = display program
uint32_t nCycle = 0; //Idle cycle counter
uint32_t displayOff = 500000; //Number of idle cycles before display is turned off - approx. 1,000,000 cycles = 3 seconds
boolean dispOn = 1; //Whether display is on or off
boolean locked = 0; //Whether controls are locked or unlocked
uint8_t debounce = 200; //Time delay after button press/release to wait for bouncing to stop
boolean trigger = 0; //Recorded state of external trigger

void setup() {
  // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  ADCSRA |= PS_4;    //Sets sample rate to 308kHz - best temporal precision for 1-10kHz mirror
  DIDR0 = B11111111; //Turns off digital input on pins A0-A5 (PORTC) to decrease noise to ADC and current load
  
  Serial.begin(250000);
  Serial.setTimeout(100); //Wait up to 1 second for full byte string
  DDRD = B01101010; //Set pins 1, 3, 5, and 6 as output and the rest as input
  DDRB |= B00101011; //Set CS pins 8, 9 and SPI pins 11, 13 as output
  PORTD |= B10011000; //Set pin 3 high, and pins 4 and 7 as input pullup
  PORTB |= B00000011; //Set CS high to block SPI to components
  initializeDisplay(); //Boot the OLED display to the default text - must be initiazlized before SPI.begin

  //Initialize SPI communication
  SPI.begin();

  //Set both laser outputs to 0 intensity
  DACout(0,0);
  DACout(0,1);
}
 
void loop() {
  //Check to see if a button is pressed
  if(~PIND & B10010000){
    SPI.end();
    buttonPress();
    SPI.begin();
  }
  if(nCycle > displayOff && dispOn){ //If inactivity timer runs out, turn off display
    SPI.end();
    command(B00001000); //turn off display
    dispOn = 0;
    SPI.begin();
  }

  nCycle++;
  if(Bon | Ron) laserOut(); //Generate output if either laser is on
  
  
}

void laserOut(){
  //0 = Manual; 1 = Scanimage sync; 2 = Ext. Trigger; 3 = Scanimage trigger
  if(Bon){ //If blue laser on
    if(Bmode == 0) DACout(intB, 1);
    if(Bmode == 1) DACout(analogRead(0)*4,1);
  }
  else{
    
  }
}

void DACout(uint16_t intensity, uint16_t DAC){
     //Add 4 bit header to intensity value (0011)
    intensity = intensity + 12288 + 32768*DAC; //MSB - Bit 15 is 0,1 (32768) - DAC select, Bit 14 is 0 (16384) - unbuffered DAC, Bit 13 is 1 (8192) - 1x gain,  Bit 12 is 1 (4096) - Vout is active: 8192+4096 = 12288
  
    //Start communication by pulling CS pin (8) low
    PORTB &= B11111110;
    
    //Initialize communication settings
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0)); //Configure SPI to 20MHz, MSB first, and clock idle LOw/clock phase falling
  
    //Send the 16-bit intensity information to the DAC register
    SPI.transfer16(intensity);
  
    //Signal end of SPI communication
    SPI.endTransaction();
  
    //Raise CS pin to mark end of commnication
    PORTB |= B00000001;
  
    //Latch value
    PORTD &= B11110111;
    PORTD &= B11110111;
    PORTD |= B00001000; 
}

void buttonPress(){
   
  if(!dispOn){ //If diplay is off, only turn display on without changing settings
    command(B00001100); //Turn display on if off
    dispOn = 1;
    while(~PIND & B10010000); //Wait for button release
    delay(debounce); //Debounce
  }
  else{ //If display is already on, change settings
    uint8_t timer1 = 0;
    timer1++; //Start button hold timer
    while(~PIND & B10010000 && timer1){ //While button is held
      timer1++;
      delay(10);
      if(!(PIND & B00010000)) buttonIn = 0; //If red button
      if(!(PIND & B10000000)) buttonIn = 1; //If blue button
      if(!(PIND & B10010000) && !locked) timer1 = 0; //If both pressed and is unlocked, immediately lock
      if(!(timer1)){ //If timer has fully cycled, then set to button hold options
        if(!(PIND & B00010000)) buttonIn = 3;
        if(!(PIND & B10000000)) buttonIn = 4; 
        if(!(PIND & B10010000)) buttonIn = 5; 
      }
    }
    if(timer1 && !locked){ //If momentary press and unlocked , toggle lasers
      if(buttonIn) toggleLaser(0);
      else toggleLaser(2);
      timer1 = 0;
    }
    else if(!locked && buttonIn < 5) configure(); //Otherwise if siplay i
    else if(buttonIn == 5) lockDisp(); //If both buttons are held, lock/unlock display
    else; //Othewise do nothing other than momentarily illuminate the screen
  }
  nCycle = 0;
}

void lockDisp(){
  if((!Bmode && Bon) || (!Rmode && Ron)){  //Block locking the control in manual for safety reasons - do not want laser locked on
    if(Bon) command(192); //If blue on move cursor to R2, C15
    else command(212); //If red on move cursor to R4, C15
    printStr("No lock if manual on");

    while(~PIND & B10010000); //Wait for button release
    delay(debounce); //Debounce

    //Refresh full display to remove "N/A" text
    displayState(2+Ron);
    displayState(Bon);
  }
  else{
    locked = !locked; //Switch locked state
    if(locked){
      if(Bon) command(206); //If blue on move cursor to R2, C15
      else if(Ron) command(226); //If red on move cursor to R4, C15
      else command(218); //If both off, move to R4, C7
      printStr("Locked");
    }
    else{ //Otherwise, refresh full display to remove lock text
      displayState(2+Ron);
      displayState(Bon);
    }
    while(~PIND & B10010000); //Wait for button release
    delay(debounce); //Debounce
  }

}

void configure(){
  
  if(buttonIn < 5){ //If single button pressed, then configure individual laser
    buttonIn -= 3; //set button in to boolean value for simplicity - 0=red, 1=blue
    if(buttonIn) command(135); //Move cursor to R1 C12
    else command(155); //Move cursor to R3 C12
    printStr("CON");

    uint8_t a = 0; //initialize internal counter
    
    while(~PIND & B10010000){ //blink button while waiting for release
      delay(40);
      a++;
      if(a%32 == 0){
        if(buttonIn) analogWrite(6, buttonIntB);
        else analogWrite(5, buttonIntR);
      }
      else if (a%32 == 16){
        if(buttonIn) analogWrite(6, 0);
        else analogWrite(5, 0);
      }
    }

    uint8_t laser = 0; //Set display state configuration and show settings 
    uint16_t a1a = 0; //store current a1
    uint16_t a2a = 0; //store current a2
    uint16_t a1b = 0; //store past a1
    uint16_t a2b = 0; //store past a2
    uint16_t num = 0; //store gamma corrected value and current ana sum
    int b = 0; //counter
    if(buttonIn) laser = 5;
    else laser = 7;
    displayState(laser);
    
    while((buttonIn && (PIND & B10000000)) || (!buttonIn && (PIND & B00010000))){ //While waiting for exiting button press
      a++;
      
      //Read knobs and change time/int accordingly
      num = 0;
      for(b=0; b<1064; b++){ 
        analogRead(1);
        if(b>=1000){
          delayMicroseconds(156);
          num += analogRead(1);
        }
              
      }
      a1a = num>>6;
      num = 0;
      for(b=0; b<1064; b++){
        analogRead(2);
        if(b>=1000){
          delayMicroseconds(156);
          num += analogRead(2);
        }
        
      }
      a2a = num>>6;
      delay(20);
      if(abs(a1a-a1b)>2){
        num = (uint16_t) (pow(1020, 1-gammaInt)*pow(a1a,gammaInt)*4);
        if(num > 4095) num = 4095; //Cap value at max DAC number
        if(buttonIn) intB = num; 
        else intR = num;
        a1b = a1a;
        Serial.println(num);
      }
      if(abs(a2a-a2b)>2){
        num = (uint16_t) (pow(1024, 1-gammaTime)*pow(a2a,gammaTime)*9.766);
        if(buttonIn) timeB = num;
        else timeR = num;
        a2b = a2a;
        Serial.println(num);
      }      
      if(a%16 == 0){
        if(buttonIn) analogWrite(6, buttonIntB);
        else analogWrite(5, buttonIntR);
      }
      else if (a%16 == 8){
        if(buttonIn) analogWrite(6, 0);
        else analogWrite(5, 0);
      }

      //Read mode button press and cycle mode accordingly 
      if (buttonIn && !(PIND & B00010000)){
        Bmode++; //if blue laser config and red pressed cycle blue mode 1
        if(Bmode >= 4) Bmode = 0; 
        while(~PIND & B00010000); //wait for button release
      }
      if (!buttonIn && !(PIND & B10000000)){
        Rmode++; 
        if(Rmode >= 4) Rmode = 0; 
        while(~PIND & B10000000); //wait for button release
      }
      displayState(laser); //Update the display accordingly
    }
    while(~PIND & B10010000); //wait for button release
    delay(debounce); //Debounce
      
    if(buttonIn) laser = Bon; //set display  and buttonsto prior configuration
    else laser = 2 + Ron;
    displayState(laser);
    if(buttonIn) command(135); //Move cursor to R1 C12
    else command(155); //Move cursor to R3 C12
    if((buttonIn && Bon) || (!buttonIn && Ron)) printStr("On ");
    else printStr("Off");
    if(buttonIn && Bon) analogWrite(6, buttonIntB);
    else if(buttonIn && !Bon) analogWrite(6, 0);
    if(!buttonIn && Ron) analogWrite(5, buttonIntR);
    else if(!buttonIn && !Ron) analogWrite(5, 0);
  }
  else{ //Othwise go into the display configure mode
    
    
  }
}

void toggleLaser(uint8_t laser){
  //First set both lasers to default of 0 int
  SPI.begin();
  DACout(0,0);
  DACout(0,1);
  SPI.end();
  
  if(laser){ //If red laser
    Ron = !Ron;
    if(Ron) analogWrite(5, buttonIntR);
    else analogWrite(5, 0);
    command(155); //Move Cursor to R3, C10
    laser += Ron;
  }
  else{ //if blue laser
    Bon = !Bon;
    if(Bon) analogWrite(6, buttonIntB);
    else analogWrite(6, 0);
    command(135); //Move Cursor to R1, C10
    laser += Bon;
  }
  if(laser & B00000001) printStr("On ");
  else printStr("Off");
  displayState(laser);

  if(Bon & Ron){ //If both lasers are on, turn off the other laser
    if(laser & B00000010) toggleLaser(0);
    else toggleLaser(2);
  }
}


void displayState(uint8_t laser){

  //Update laser output
  uint16_t power = 0;
  uint16_t timed = 0;
  int a = 0;
  if(laser & B00000010){ //If red laser move to R4, C1
    command(212); 
    power = intR;
    timed = timeR; 
  }
  else{ //if blue laser move to R2, C1
    command(192);
    power = intB;
    timed = timeB;
  }
  power = (power*0.2443)+0.5; //Convert power from 12-bit to 0-1000
  if(laser & B00000001){ //If laser is on, show state
    if(timed > 0 && (((laser & B00000010) && (Rmode != 1)) || (!(laser & B00000010) && (Bmode != 1)))){ //If time > 0 and mode is not scan-sync, then show power
      for(a=4; a>=0; a--){ //Read out time numbers into state array
        if(timed){
          state[a] = (char) (timed % 10 + '0'); //Add 0 since result will be converted to ASCII value - 0 != '0'
          timed /= 10;
        }
        else state[a] = ' ';
      }
      state[5] = 'm';
    }
    else{ //Otherwise show sync
      state[1] = 'S';
      state[2] = 'y';
      state[3] = 'n';
      state[4] = 'c';
      state[5] = ' ';
    }
    if(((laser & B00000010) && (Rmode == 1)) || (!(laser & B00000010) && (Bmode == 1))){ //If in scan-sync - show sync - only scan-sync uses ADC to sync power
      state[8] = 'S';
      state[9] = 'y';
      state[10] = 'n';
      state[11] = 'c';
    }
    else if(power < 10){
      state[11] = (char) (power % 10 + '0'); //Add 0 since result will be converted to ASCII value - 0 != '0'
      state[10] = '.';
      state[9] = '0';
      state[8] = ' ';
    }
    else if(power < 100){
      state[11] = (char) (power % 10 + '0'); //Add 0 since result will be converted to ASCII value - 0 != '0'
      state[10] = '.';
      power /= 10;
      state[9] = (char) (power % 10 + '0'); //Add 0 since result will be converted to ASCII value - 0 != '0'
      state[8] = ' ';
    }
    else if(power < 995){
      power /= 10;
      state[11] = (char) (power % 10 + '0'); //Add 0 since result will be converted to ASCII value - 0 != '0'
      power /= 10;
      state[10] = (char) (power % 10 + '0'); //Add 0 since result will be converted to ASCII value - 0 != '0'
      state[9] = ' ';
      state[8] = ' ';
    }
    else{
      state[11] = '0';
      state[10] = '0';
      state[9] = '1';
      state[8] = ' ';
    }

    if(laser & B00000100) state[0] = B11110110; //If in configure, add arrow to start of line
    printStr(state);
  }
  else for(a=0; a<10; a++) printStr("- "); //if laser is off, display dashed lines 
  
  //Update mode
  if(laser & B00000010){ //If red laser move to R3, C12
    command(159);
    if(Rmode%4 == 0) printStr("Manual   ");
    else if(Rmode%4 == 1) printStr("Scan-Sync");
    else if(Rmode%4 == 2) printStr("Scan-Trig");
    else if(Rmode%4 == 3) printStr("Ext. Trig");
  }
  else{
    command(139);
    if(Bmode%4 == 0) printStr("Manual   ");
    else if(Bmode%4 == 1) printStr("Scan-Sync");
    else if(Bmode%4 == 2) printStr("Scan-Trig");
    else if(Bmode%4 == 3) printStr("Ext. Trig");
  }
}

void initializeDisplay(){
  int a = 0;

  //Clear display and reset settings
  //Start communication by pulling CS pin (9) low
  PORTB &= B11111101;
  
  //Initialize communication settings with header
  RsRW(0,0);
  bitbang(B00111011);  //Set function - choose western european table 2
  delay(d2);
  RsRW(0,0);
  bitbang(B00001000); //Turn off display
  delay(d2);
  RsRW(0,0);
  bitbang(B00000001); //Clear display    
  delay(d2);
  RsRW(0,0);
  bitbang(B00000110); //Entry mode set to increment  
  delay(d2);
  RsRW(0,0);
  bitbang(B00000010); //Home Command
  delay(d2);
  RsRW(0,0);
  bitbang(B00001100); //Display On - no cursor

  //end communication by pulling CS pin (9) high
  PORTB |= B0000010;  
  delay(d2);

  //Initialize text on screen
  //Start communication by pulling CS pin (9) low
  printStr("488 nm Off Manual   635 nm Off Manual   ");
  command(192);  //Move cursor to R2-CA
  for(a=0; a<20; a++) printStr("- ");

}

//Function to bit bang the RS and R/W leading bits to the OLED display
void RsRW(boolean RS, boolean RW){
  PORTB &= B11011111; //Set CLK low
  if(RS) PORTB |= B00001000; //Set MOSI pin to data state
  else   PORTB &= B11110111;
  PORTB |= B00100000; //Set CLK high to latch data

  PORTB &= B11011111; //Set CLK low
  if(RW) PORTB |= B00001000; //Set MOSI pin to data state
  else   PORTB &= B11110111;
  PORTB |= B00100000; //Set CLK high to latch data
}

void bitbang (byte ptr) {
    for(int b = 0; b<8; b++){
      PORTB &= B11011111; //Set CLK low
      if (ptr & 0x80) PORTB |= B00001000; //Set MOSI pin to data state
      else PORTB &= B11110111;
      PORTB |= B00100000; //Set CLK high to latch data
      ptr <<= 1; //Sift to next bit
    }
}

void printStr(char str[]){
  //Get the length of the char array
  uint8_t strLen = strlen(str);
  
  //Start communication by pulling CS pin (9) low
  PORTB &= B11111101;
      
  //Initialize communication settings with header
  RsRW(1,0);

  //Since Arduino and display both use ASCII, send ASCII char bytes one at a time
  for(int a = 0; a<strLen; a++) bitbang(str[a]);
  
  //end communication by pulling CS pin (9) high
  PORTB |= B0000010; 
} 

void command (uint8_t command){
  //Start communication by pulling CS pin (9) low
  PORTB &= B11111101;
  
  //Initialize communication settings with header
  RsRW(0,0);
  bitbang(command);  //send command

  //end communication by pulling CS pin (9) high
  PORTB |= B0000010;  
  delay(d2);
}





