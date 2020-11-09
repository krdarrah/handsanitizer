/*
   Hand Sanitizer Project - KD CIRCUITS
  https://www.kdcircuits.com

   This is the one from Amazon: https://www.amazon.com/gp/product/B08CXTCLJQ/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1
   The Sanitizer I'm using: https://www.bedbathandbeyond.com/store/product/sparoom-reg-8-oz-spray-bottle-antibacterial-hand-sanitizer/5511915?opbthead=true&ta=typeahead&keyword=hand-sanitizer

   Based on the ATMEGA328P - CHOOSE ARDUINO MINI from the tools menu
   https://www.tindie.com/products/kdcircuits/328-board-atmega328p-breakout/


   Motor Mosfet: FQD13N06LTM
   https://www.digikey.com/en/products/detail/on-semiconductor/fqd13n06ltm/1053548


*/


//PINS
const int ir_LEDpin = 2;
const int ir_RXpin = 6;
const int motorPin = 3;
const int irAnalogPin = 0;

//This changes the trip point, the bigger this is, the more sensitive
const float percentChange = 10.0;

//variables
int pastReadings[20];//running avg of steady state, so we have something to compare against
float runningAvg;//running avergae of 20 past readings
int preloadCount = 0;//this is used on powerup only, just we can wait for all 20 readings for a good avg
boolean handIsPresent = false;// needed so we don't keep spraying if someone leaves there hand there
int handCount = 0;//used so we know how long the hand is there, and can recalibrate in case it falsely thinks the hand is there

//functions
void gotoSleep(byte prescaler);// just put the sleep routine in a function, you can pass the prescaler in here

void setup() {
  pinMode(ir_RXpin, OUTPUT);
  pinMode(ir_LEDpin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  Serial.begin(115200);
  Serial.println("booting");
  delay(100);
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep
  //check video on sleep modes: https://www.youtube.com/watch?v=urLSDi7SD8M

}

void loop() {
  gotoSleep(6);// immediatly sleep for 1 sec

  //WAKEUP HERE!
  digitalWrite(ir_RXpin, HIGH);//first power up the photodiode
  delay(2);//let things settle out, but this time can be adjusted

  int darkReading = analogRead(irAnalogPin);//get a "dark" reading, to check against before we fire up the IR LED
  digitalWrite(ir_LEDpin, HIGH);//IR LED ON now
  delay(1);//this time is critical, but try to keep on for as little time as possible

  int IRreading = analogRead(irAnalogPin);//get a "hot" reading
  //ok, we're done here, so kill the photodiode and IR LED
  digitalWrite(ir_LEDpin, LOW);
  digitalWrite(ir_RXpin, LOW);
  //modify that reading now to be a difference from the dark reading, and get it away from 0.. 100 feels about right
  IRreading = (IRreading - darkReading) + 100;

  //increment the preload, but only if under 20, this is used for powerup only, to "preload" the rolling average
  if (preloadCount < 20)
    preloadCount++;

  // for debugging you can see the value in real time
  //  Serial.println(IRreading);
  //  delay(10);

  // THE CHECK
  // only if we're past the preload and lower than the treshold means a "hand" is detected
  if (IRreading < (runningAvg - ((percentChange / 100.00)*runningAvg)) && preloadCount >= 20) {
    Serial.println("#");
    delay(1);
    handCount++;
    if (handCount > 20) {//the hand has been here for too long, 20+seconds, let's reset the cal
      Serial.println("reseting calibration...");
      delay(10);
      handCount = 0;
      preloadCount = 0;
    }
    if (!handIsPresent) {// hand was previously not here, so let's spray that hand!
      handIsPresent = true;//set it true, so on the next wake we don't keep spraying if the hand is still here

      // some debug info
      Serial.print(IRreading);
      Serial.print("read, ");
      Serial.print(runningAvg);
      Serial.print("avg ");
      Serial.print(((IRreading - runningAvg) / runningAvg) * 100);
      Serial.println("% ");

      //fire up the motor!
      digitalWrite(motorPin, HIGH);
      delay(500);// how long to keep on for
      digitalWrite(motorPin, LOW);
    }


  } else {//no hand, so let's keep the average moving along

    //reset if the hand was previously there
    handIsPresent = false;
    handCount = 0;

    for (int i = 1; i < 20; i++) {//movin the average along, kickin the last out
      pastReadings[i] = pastReadings[i - 1];
    }
    pastReadings[0] = IRreading;//new one goes in [0]

    // now take the average
    runningAvg = 0;
    for (int i = 0; i < 20; i++) {
      runningAvg = runningAvg + pastReadings[i];
    }
    runningAvg = runningAvg / 20.0;
  }

}


void gotoSleep(byte prescaler) {

  //SETUP WATCHDOG TIMER
  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (prescaler);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1 << 6); //enable interrupt mode
  ADCSRA &= ~(1 << 7);//kill ADC
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
  __asm__  __volatile__("sleep");//in line assembler to go to sleep
  ADCSRA |= (1 << 7);//woke back up, turn ON ADC

  //Prescaler Chart copied out of datasheet for reference:
  // setup like this 32,4,2,1
  // so for a delay of .125ms, you would set prescaler to 3, or 8 sec would be 32+1 = 33

  //0 0 0 0 2K (2048) cycles 16 ms
  //0 0 0 1 4K (4096) cycles 32 ms
  //0 0 1 0 8K (8192) cycles 64 ms
  //0 0 1 1 16K (16384) cycles 0.125 s
  //0 1 0 0 32K (32768) cycles 0.25 s
  //0 1 0 1 64K (65536) cycles 0.5 s
  //0 1 1 0 128K (131072) cycles 1.0 s
  //0 1 1 1 256K (262144) cycles 2.0 s
  //1 0 0 0 512K (524288) cycles 4.0 s
  //1 0 0 1 1024K (1048576) cycles 8.0 s
}


ISR(WDT_vect) {
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
}// watchdog interrupt
