// NEW IN VERSION 1.5:
// Changed pin 8 from standard servo to normal digital output

// NEW IN VERSION 1.4:
// Changed Serial.print() for Serial.write() in ScratchBoardSensorReport function to make it compatible with latest Arduino IDE (1.0)

// NEW IN VERSION 1.3:
// Now it works on GNU/Linux. Also tested with MacOS and Windows 7.
// timer2 set to 20ms, fixing a glitch that made this period unstable in previous versions.
// readSerialport() function optimized.
// pulse() modified so that it receives pulse width as a parameter instead using a global variable.
// updateServoMotors changes its name as a global variable had the same name.
// Some minor fixes.

// Thanks to Jorge Gomez for all these new fixes!

// LB_S4AFirmware littleBits version, compatible with Leonardo-based w6-arduino bit module.


#define TIMER_PRELOAD 100

char outputs[10];
int states[10];

unsigned long lastDataReceivedTime;

volatile boolean updateOutputs;
volatile boolean newInterruption;

void setup()
{
 Serial.begin(38400);
 Serial.flush();
 configurePins();
 lastDataReceivedTime = millis();
}

void loop()
{
 if (updateOutputs)
 {
   sendSensorValues();
   updateOutputs = false;
 }
 else
 {
   readSerialPort();
 }
}

void configurePins()
{
 for (int index = 0; index < 10; index++)
 {
    states[index] = 0;
    pinMode(index+4, OUTPUT);
    digitalWrite(index+4, LOW); //reset pins
 }
 
 pinMode(1, OUTPUT);
 digitalWrite(1, LOW);
 
 pinMode(9, OUTPUT);
 digitalWrite(9, LOW);

 //D0 Input
 pinMode(0,INPUT);
 
 //Not on Board
 outputs[0] = 'd'; //pin 4 -- Hacked and mapped to d1
 
 //Could be mapped to d2 and d3
 outputs[2] = 'd'; //pin 6
 outputs[3] = 'd'; //pin 7
 outputs[4] = 'd'; //pin 8
 
 //Bit Snaps
 outputs[1] = 'a'; //pin 5
 outputs[5] = 'a'; //pin 9
 
 //Thru Holes
 outputs[6] = 'd'; //pin 10
 outputs[7] = 'd'; //pin 11
 outputs[9] = 'd'; //pin 13
 
 outputs[8] = 'd'; //pin 12
  
 newInterruption = false;
 updateOutputs = false;

 TCCR4A = 0;
 TCCR4B = 0<<CS12 | 1<<CS11 | 1<<CS10; //64 pre-scalar
 TIMSK4 = 1<<TOIE4; //timer Overflow Interrupt
 TCNT4 = TIMER_PRELOAD; //start timer

}

void sendSensorValues()
{
  int sensorValues[6], readings[5], sensorIndex;
    for (sensorIndex = 0; sensorIndex < 6; sensorIndex++) //for analog sensors, calculate the median of 5 sensor readings in order to avoid variability and power surges
    {
      for (int p = 0; p < 5; p++)
        readings[p] = analogRead(sensorIndex);
      InsertionSort(readings, 5); //sort readings
      sensorValues[sensorIndex] = readings[2]; //select median reading
    }

    //send analog sensor values
    for (sensorIndex = 0; sensorIndex < 6; sensorIndex++)
      ScratchBoardSensorReport(sensorIndex, sensorValues[sensorIndex]);

    //send digital sensor values
    //Hacked: mapped d0 instead of d2 and d2 instead of d3.
    ScratchBoardSensorReport(6, digitalRead(0)?1023:0);
    ScratchBoardSensorReport(7, digitalRead(2)?1023:0);
}

void InsertionSort(int* array, int n)
{
  for (int i = 1; i < n; i++)
    for (int j = i; (j > 0) && ( array[j] < array[j-1] ); j--)
      swap( array, j, j-1 );
}

void swap (int* array, int a, int b)
{
  int temp = array[a];
  array[a] = array[b];
  array[b] = temp;
}

void ScratchBoardSensorReport(int sensor, int value) //PicoBoard protocol, 2 bytes per sensor
{
  Serial.write( B10000000
                | ((sensor & B1111)<<3)
                | ((value>>7) & B111));
  Serial.write( value & B1111111);
}

void readSerialPort()
{
  int pin, inByte, sensorHighByte;

  if (Serial.available() > 1)
  {
    lastDataReceivedTime = millis();
    inByte = Serial.read();

    if (inByte >= 128) // Are we receiving the word's header?
    {
      sensorHighByte = inByte;
      pin = ((inByte >> 3) & 0x0F);
      while (!Serial.available()); // Wait for the end of the word with data
      inByte = Serial.read();
      if (inByte <= 127) // This prevents Linux ttyACM driver to fail
      {
        //Hack to deal with d1
        if (pin == 0) {
          pin = 4; //mapping d1 to "pin 4"
        }
        states[pin - 4] = ((sensorHighByte & 0x07) << 7) | (inByte & 0x7F);
          updateActuator(pin - 4);
      }
    }
  }
  else checkScratchDisconnection();
}

void reset() //with xbee module, we need to simulate the setup execution that occurs when a usb connection is opened or closed without this module
{
  for (int pos = 0; pos < 10; pos++)  //stop all actuators
  {
    states[pos] = 0;
    digitalWrite(pos + 2, LOW);
  }

  newInterruption = false;
  updateOutputs = false;
  TCNT4 = TIMER_PRELOAD;

  //protocol handshaking
  sendSensorValues();
  lastDataReceivedTime = millis();
}

void updateActuator(int pinNumber)
{
  if (pinNumber == 0) {
     digitalWrite(1, states[pinNumber]); //Hack to map d1
  } else if (outputs[pinNumber] == 'd')  {
    digitalWrite(pinNumber + 4, states[pinNumber]);
  } else if (outputs[pinNumber] == 'a') {
    analogWrite(pinNumber + 4, states[pinNumber]);
  }
}

void checkScratchDisconnection() //the reset is necessary when using an wireless arduino board (because we need to ensure that arduino isn't waiting the actuators state from Scratch) or when scratch isn't sending information (because is how serial port close is detected)
{
  if (millis() - lastDataReceivedTime > 1000) reset(); //reset state if actuators reception timeout = one second
}

ISR(TIMER4_OVF_vect) //timer1 overflow interrupt vector handler
{ //timer2 => 8 bits counter => 256 clock ticks
  //preeescaler = 1024 => this routine is called 61 (16.000.000/256/1024) times per second approximately => interruption period =  1 / 16.000.000/256/1024 = 16,384 ms
  //as we need a 20 ms interruption period but timer2 doesn't have a suitable preescaler for this, we program the timer with a 10 ms interruption period and we consider an interruption every 2 times this routine is called.
  //to have a 10 ms interruption period, timer2 counter must overflow after 156 clock ticks => interruption period = 1 / 16.000.000/156/1024 = 9,984 ms => counter initial value (TCNT) = 100
  if (newInterruption)
  {
    updateOutputs = true;
  }
  newInterruption = !newInterruption;
  TCNT4 = TIMER_PRELOAD;  //reset timer
}
