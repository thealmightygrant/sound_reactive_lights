#include <avr/wdt.h>

// Sample rate in samples per second.
// This will be rounded so the sample interval
// is a multiple of the ADC clock period.
const uint32_t SAMPLE_RATE = 40000;
// Desired sample interval in CPU cycles (will be adjusted to ADC/timer1 period)
const uint32_t SAMPLE_INTERVAL = F_CPU/SAMPLE_RATE;
// Minimum ADC clock cycles per sample interval
const uint16_t MIN_ADC_CYCLES = 15;
const uint8_t ANALOG_PIN = 0;
const int R_PWM = 10;
const int G_PWM = 9;
const int B_PWM = 8;
const int hledPin =  43;      // the number of the LED pin
const int sledPin =  41;      // the number of the LED pin
const int vledPin =  39;      // the number of the LED pin

const int manledPin = 49;
const int randledPin = 51;
const int micledPin = 45;
const int notnamedledPin = 47;

// Reference voltage
// Zero - use External Reference AREF
// (1 << REFS0) - Use Vcc
// (1 << REFS1) | (1 << REFS0) - use Internal 1.1V Reference
uint8_t const ADC_REF_AVCC = (1 << REFS0);

void adcInit(uint32_t ticks, uint8_t pin, uint8_t ref);
void adcStart();
uint16_t acquire();
uint16_t findMax(uint8_t arr[], int n);
uint16_t findMax(uint16_t arr[], int n);


//COLOR RELATED!!
//Hue: 0=0°, 255=360°
byte H;
//Saturation: 0=0, 255=1
byte S;
//Value: 0=0, 255=1
byte V;
//final vals for lights
byte r,g,b;
byte Vmax, Smax;

//button related
const int upButtonPin = 24;     // the number of the pushbutton pin
const int downButtonPin = 26;     // the number of the pushbutton pin
// variables will change:
int upButtonState = 0;          // variable for reading the pushbutton status
int downButtonState = 0;        // variable for reading the pushbutton status
bool upButtonCounted = 0;       // so that each button press only does one change
bool downButtonCounted = 0;     // so that each button press only does one change

const int settingButtonPin = 30;     // the number of the pushbutton pin
const int valueButtonPin = 28;       // the number of the pushbutton pin
// variables will change:
int settingButtonState = 0;          // variable for reading the pushbutton status
int valueButtonState = 0;        // variable for reading the pushbutton status
bool settingButtonCounted = 0;       // so that each button press only does one change
bool valueButtonCounted = 0;     // so that each button press only does one change

//Type of oscillation
enum oscType {
 MAN,  //No oscillation, use buttons
 RAND, //RANDOM, oooo
 MIC   //Microphone input
};

//Type of oscillation
enum settingType {
 HUESET,  //h
 SATSET,  //s
 VALSET,  //v
};

oscType valOsc;
oscType satOsc;
oscType hueOsc;
settingType currentSetting;

#define LIN_OUT 1 // use the log output function
#define FHT_N 256 // set to 256 point fht

#include <FHT.h> // include the library


unsigned long StartAcq = 0;
unsigned long EndAcq = 0;
unsigned long FftCount = 0;
unsigned long SampFreqAvg = 0;
unsigned long AvgFreq = 0;
unsigned long NumFreq = 0;
unsigned long AvgAvgFreq = 0;
unsigned long NumAvgs = 0;

int BufPtr = 0;


boolean TogState = false;
boolean ErrorCond = false;

long k_d, k_d_prec, micV, curMicV;
int curMicV_diff, reset_period;
byte lastMicV; 

void setup()
{

  // initialize the pushbutton pin as an input:
  pinMode(upButtonPin, INPUT);
  pinMode(downButtonPin, INPUT);  
  
  //Initialization of Random
  randomSeed(analogRead(0));
  
  wdt_reset();
  Serial.begin(115200); // use the serial port
  adcInit(SAMPLE_INTERVAL, ANALOG_PIN, ADC_REF_AVCC);

  pinMode(R_PWM, OUTPUT);
  pinMode(G_PWM, OUTPUT);
  pinMode(B_PWM, OUTPUT);

  pinMode(hledPin, OUTPUT);      
  pinMode(sledPin, OUTPUT); 
  pinMode(vledPin, OUTPUT); 
  
  wdt_enable(WDTO_8S);

  //Initialization of variables
  valOsc = MIC; 
  satOsc = MAN; 
  hueOsc = MIC;
  currentSetting = VALSET;

  H = 0;
  S = 0;
  V = 0;
  Smax = 255;
  Vmax = 150;
  lastMicV = constrain(Vmax - 20, 0, 255);
  micV = constrain(Vmax - 20, 0, 255);
  k_d_prec = 100000;
  k_d = Vmax * k_d_prec * 2;
  reset_period = 30;
}


void loop()
{
  //Get the type of setting being done (H, S, V)
  readSettingType();
  set_led_display();

  //Get value oscillation type
  readValueOsc();
  
  
  if (ErrorCond)
  {
    Serial.println("Error");
    return;
  }
  
  uint16_t freq = acquire();
  NumFreq++;
  AvgFreq = (AvgFreq * (NumFreq - 1) + freq) / NumFreq;

  NumAvgs++;
  AvgAvgFreq = (AvgAvgFreq * (NumAvgs - 1) + AvgFreq) / NumAvgs;
  //calculate sin of angle as number between 0 and 255
  curMicV = ((2550000 * AvgFreq) / (15 * AvgAvgFreq)) / 1000;    // 255 * (AvgFreq / (1.7 * AvgAvgFreq))
  if(curMicV > Vmax)
    curMicV = curMicV - Vmax;
  if(curMicV < 0)
    curMicV = curMicV + Vmax;
    
  curMicV_diff = ((curMicV - lastMicV) > 0) ? 1 : -1;             // k_d is what the frequeny should affect...

  micV = lastMicV + ((k_d * curMicV_diff)/255)/k_d_prec;
  if(micV > Vmax)
    micV = micV - Vmax;
  if(micV < 0)
    micV = micV + Vmax;
    
  lastMicV = micV;
  //Serial.print("Number of FHT's computed: ");
  //Serial.println(FftCount);
//  Serial.print("Addition: ");
//  Serial.println(((k_d * curMicV_diff)/255)/k_d_prec);
//    Serial.print("maxV: ");
//  Serial.println(Vmax);
//  Serial.print("MicV: ");
//  Serial.println(curMicV);
//  Serial.print("lastMicV: ");
//  Serial.println(lastMicV);
//  Serial.print("Diff: ");
//  Serial.println(curMicV_diff);
//  //Serial.print("new V: ");
//  //Serial.println(micV);
//  Serial.print("Average frequency: ");
//  Serial.println(AvgFreq);
//  Serial.print("Average Average frequency: ");
//  Serial.println(AvgAvgFreq);
  //Serial.print("Average Sampling frequency: ");
  //Serial.println(SampFreqAvg);
  
  if ((FftCount % reset_period) == 0)
  {
    //RESET AVG
    NumFreq = 0;
    NumAvgs = 0;
  }

  set_colors();
  hsv_to_rgb();

//
//  Serial.print("H: ");
//  Serial.print(H);
//  Serial.print(" S: ");
//  Serial.print(S);
//  Serial.print(" V: ");
//  Serial.print(V);
//  
//  Serial.print("R: ");
//  Serial.print(r);
//  Serial.print(" G: ");
//  Serial.print(g);
//  Serial.print(" B: ");
//  Serial.println(b);
//
//  Serial.print("valOsc: ");
//  Serial.println(valOsc);
//  Serial.print("satOsc: ");
//  Serial.println(satOsc);
//  Serial.print("hueOsc: ");
//  Serial.println(hueOsc);
//
//  Serial.print("currentSetting: ");
//  Serial.println(currentSetting);

  // put your main code here, to run repeatedly:
  analogWrite(R_PWM, r);
  analogWrite(G_PWM, g);
  analogWrite(B_PWM, b);
  
  wdt_reset();
}

uint16_t acquire()
{
  uint16_t fs, binNum, freq;
  
  adcStart();
  BufPtr = 0;
  
  StartAcq = micros();
  while(BufPtr < FHT_N)
  {
    millis();
  }
  EndAcq = micros();
  
  FftCount++;
  SampFreqAvg = (SampFreqAvg * (FftCount - 1) + (EndAcq - StartAcq)) / FftCount;
  
  fs = 1.0 / ((double)(EndAcq - StartAcq) / 1000000.0 / FHT_N);
  
  // process data
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht
  fht_mag_lin(); // take the output of the fht

  binNum = findMax(fht_lin_out, 40); // FHT_N/2);  NOTE: DON'T BOTH WITH MORE THAN 1KHz (CAN'T HEAR IT) 
  freq = binNum * (fs / FHT_N);
  
  return freq;
}

uint16_t findMax(uint8_t arr[], int n)
{
  uint16_t m = 0;
  uint8_t val = 0;
  for (int i = 0; i < n; i++)
  {
    if (arr[i] > val)
    {
      m = i;
      val = arr[i];
    }
  }
  return m;
}

uint16_t findMax(uint16_t arr[], int n)
{
  uint16_t m = 0;
  uint16_t val = 0;
  for (int i = 0; i < n; i++)
  {
    if (arr[i] > val)
    {
      m = i;
      val = arr[i];
    }
  }
  return m;
}

void adcInit(uint32_t ticks, uint8_t pin, uint8_t ref) {
  if (ref & ~((1 << REFS0) | (1 << REFS1))) {
    //error("Invalid ADC reference bits");
    ErrorCond = true;
    return;
  }
  // Set ADC reference andlow three bits of analog pin number
  ADMUX = ref | (pin & 7);
#if RECORD_EIGHT_BITS
  // Left adjust ADC result to allow easy 8 bit reading
  ADMUX |= (1 << ADLAR);
#endif  // RECORD_EIGHT_BITS
  
 // trigger on timer/counter 1 compare match B
  ADCSRB = (1 << ADTS2) | (1 << ADTS0);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // the MUX5 bit of ADCSRB selects whether we're reading from channels
  // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
  if (pin < 8) {
    ADCSRB &= ~(1 << MUX5);
    // disable Digital input buffer
    DIDR0 |= 1 << pin;
  } else {
    ADCSRB |= (1 << MUX5);
    // disable Digital input buffer
    DIDR2 |= 1 << (7 & pin);
  }
#else  // defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // not a Mega disable Digital input buffer
  if (pin < 6) DIDR0 |= 1 << pin;
#endif  // defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#if ADPS0 != 0 || ADPS1 != 1 || ADPS2 != 2
#error unexpected ADC prescaler bits
#endif

  uint8_t adps;  // prescaler bits for ADCSRA
  for (adps = 7; adps > 0; adps--) {
   if (ticks >= (MIN_ADC_CYCLES << adps)) break;
  }
  if (adps < 3)
  {
    Serial.println("Sample Rate Too High");
    ErrorCond = true;
    return;
  }
  
  Serial.print("ADC clock MHz: ");
  Serial.println((F_CPU >> adps)*1.0e-6, 3);

  // set ADC prescaler
  ADCSRA = adps;
  
  // round so interval is multiple of ADC clock
  ticks >>= adps;
  ticks <<= adps;
  
  // Setup timer1
  // no pwm
  TCCR1A = 0;
  
  uint8_t tshift;
  if (ticks < 0X10000) {
  // no prescale, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    tshift = 0;
  } else if (ticks < 0X10000*8) {
    // prescale 8, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    tshift = 3;
  } else if (ticks < 0X10000*64) {
    // prescale 64, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    tshift = 6;
  } else if (ticks < 0X10000*256) {
    // prescale 256, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
    tshift = 8;
  } else if (ticks < 0X10000*1024) {
    // prescale 1024, CTC mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS10);
    tshift = 10;
  } else {
    Serial.println("Sample Rate Too Slow");
    ErrorCond = true;
    return;
  }
  // divide by prescaler
  ticks >>= tshift;
  // set TOP for timer reset
  ICR1 = ticks - 1;
  // compare for ADC start
  OCR1B = 0;
  
  // multiply by prescaler
  ticks <<= tshift;
  Serial.print("Sample interval usec: ");
  Serial.println(ticks*1000000.0/F_CPU);
  Serial.print("Sample Rate: ");
  Serial.println((float)F_CPU/ticks);
}


void adcStart() {
  // Enable ADC, Auto trigger mode, Enable ADC Interrupt, Start A2D Conversions
  ADCSRA |= (1 << ADATE)  |(1 << ADEN) | (1 << ADIE) | (1 << ADSC) ;
  // enable timer1 interrupts
  TIMSK1 = (1 <<OCIE1B);
  TCNT1 = 0;
}

// ADC done interrupt
ISR(ADC_vect) {
  // read ADC
#if RECORD_EIGHT_BITS
  uint8_t d = ADCH;
#else  // RECORD_EIGHT_BITS
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  uint16_t d = (high << 8) | low;
#endif  // RECORD_EIGHT_BITS
  
  int k = d - 0x0200; // form into a signed int
  k <<= 6; // form into a 16b signed int
  
  // Only write to the buffer if it's not full.
  if (BufPtr < FHT_N)
  {
    fht_input[BufPtr] = k;
  }
  
  BufPtr++;
}

void set_colors(){
  if(currentSetting == HUESET){
    changeValByButtons(1, 0, 0);
  }
  else if(currentSetting == SATSET){
    changeValByButtons(0, 1, 0);
  }
  else if(currentSetting == VALSET){
    changeValByButtons(0, 0, 1);
  }
  set_hue();
  set_sat();
  set_val();
}

void set_led_display(){
  if(currentSetting == HUESET){
    digitalWrite(hledPin, HIGH);
    digitalWrite(sledPin, LOW);
    digitalWrite(vledPin, LOW);
  }
  else if(currentSetting == SATSET){
    digitalWrite(hledPin, LOW);
    digitalWrite(sledPin, HIGH);
    digitalWrite(vledPin, LOW);
  }
  else if(currentSetting == VALSET){
    digitalWrite(hledPin, LOW);
    digitalWrite(sledPin, LOW);
    digitalWrite(vledPin, HIGH);
  }
}

void set_val(){
  if(valOsc == MAN){
    V = Vmax;
    //Read buttons to set the maximum value
    if(currentSetting == VALSET){
      digitalWrite(manledPin, HIGH);
      digitalWrite(randledPin, LOW);
      digitalWrite(micledPin, LOW);
      digitalWrite(notnamedledPin, LOW);
    }
  }
  else if(valOsc == RAND){
    V = constrain(V + random(-3,4), 0, Vmax);
    if(currentSetting == VALSET){
      digitalWrite(manledPin, LOW);
      digitalWrite(randledPin, HIGH);
      digitalWrite(micledPin, LOW);
      digitalWrite(notnamedledPin, LOW);
    }
  } 
  else if(valOsc == MIC) {
    V = micV;
    if(currentSetting == VALSET){
      digitalWrite(manledPin, LOW);
      digitalWrite(randledPin, LOW);
      digitalWrite(micledPin, HIGH);
      digitalWrite(notnamedledPin, LOW);
    }
  }  
}

void set_sat(){
  if(satOsc == MAN){
    S = Smax;
    //Read buttons to set the maximum value
    if(currentSetting == SATSET){
      digitalWrite(manledPin, HIGH);
      digitalWrite(randledPin, LOW);
      digitalWrite(micledPin, LOW);
      digitalWrite(notnamedledPin, LOW);
    }
  }
  else if(satOsc == RAND){

    int nextS = S + random(-1,2);
    if(nextS < 0)
      S = 0;
    else if(nextS > Smax)
      S = Smax;
    else
      S = nextS;
    if(currentSetting == SATSET){
      digitalWrite(manledPin, LOW);
      digitalWrite(randledPin, HIGH);
      digitalWrite(micledPin, LOW);
      digitalWrite(notnamedledPin, LOW);
    }
  } 
  else if(satOsc == MIC) {
    S = micV;
    if(currentSetting == SATSET){
      digitalWrite(manledPin, LOW);
      digitalWrite(randledPin, LOW);
      digitalWrite(micledPin, HIGH);
      digitalWrite(notnamedledPin, LOW);
    }
  }  
}

void set_hue(){
  if(hueOsc == MAN){
    if(currentSetting == HUESET){
      digitalWrite(manledPin, HIGH);
      digitalWrite(randledPin, LOW);
      digitalWrite(micledPin, LOW);
      digitalWrite(notnamedledPin, LOW);
    }
  }
  else if(hueOsc == RAND){
    H = H + random(-1,2);
    if(currentSetting == HUESET){
      digitalWrite(manledPin, LOW);
      digitalWrite(randledPin, HIGH);
      digitalWrite(micledPin, LOW);
      digitalWrite(notnamedledPin, LOW);
    }
  } 
  else if(hueOsc == MIC) {
    H = micV;
    if(currentSetting == HUESET){
      digitalWrite(manledPin, LOW);
      digitalWrite(randledPin, LOW);
      digitalWrite(micledPin, HIGH);
      digitalWrite(notnamedledPin, LOW);
    }
  }  
}

void hsv_to_rgb(){
  unsigned int region, remainder, p, q, t;
  if (S == 0) { //grey
    r = V;
    g = V;
    b = V;
  } else {
    region = H / 43; //range: 0..5
    remainder = (H - (region * 43)) * 6; //range: 0..240
    p = (V * (255 - S)) >> 8;
    q = (V * (255 - ((S * remainder) >> 8))) >> 8;
    t = (V * (255 - ((S * (255 - remainder)) >> 8))) >> 8;
    switch (region) {
      case 0:
        r = V; g = t; b = p;
        break;
      case 1:
        r = q; g = V; b = p;
        break;
      case 2:
        r = p; g = V; b = t;
        break;
      case 3:
        r = p; g = q; b = V;
        break;
      case 4:
        r = t; g = p; b = V;
        break;
      default:
        r = V; g = p; b = q;
        break;
    }
  }
}

//Get the type of oscillation for the value
void readValueOsc(){
    // read the state of the pushbutton value:
  valueButtonState = digitalRead(valueButtonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if ((valueButtonState == HIGH) && (valueButtonCounted == 0)) {  
    if(currentSetting == VALSET)       
      valOsc =  (oscType)(((valOsc + 1) > MIC) ? 0 : (valOsc + 1));
    else if(currentSetting == SATSET)       
      satOsc =  (oscType)(((satOsc + 1) > MIC) ? 0 : (satOsc + 1));
    else if(currentSetting == HUESET)       
      hueOsc =  (oscType)(((hueOsc + 1) > MIC) ? 0 : (hueOsc + 1));
    valueButtonCounted = 1;
  } 
  else if((valueButtonState == LOW) && (valueButtonCounted == 1)){
    valueButtonCounted = 0;
  }
}

//Get the type of oscillation for the value
void readSettingType(){
    // read the state of the pushbutton value:
  settingButtonState = digitalRead(settingButtonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if ((settingButtonState == HIGH) && (settingButtonCounted == 0)) {     
    currentSetting = (settingType)(((currentSetting + 1) > VALSET) ? 0 : (currentSetting + 1));
    settingButtonCounted = 1;
  } 
  else if((settingButtonState == LOW) && (settingButtonCounted == 1)){
    settingButtonCounted = 0;
  }
}

void changeValByButtons(bool hue, bool sat, bool val){
  // read the state of the pushbutton value:
  upButtonState = digitalRead(upButtonPin);
  downButtonState = digitalRead(downButtonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if ((upButtonState == HIGH) && (upButtonCounted == 0)) { 
    if(val)    
      Vmax = constrain(Vmax + 5, 0, 255);
    else if(sat)
      Smax = constrain(Smax + 5, 0, 255);
    else if(hue)
      H = constrain(H + 5, 0, 255);
    upButtonCounted = 1;
  } 
  else if((upButtonState == LOW) && (upButtonCounted == 1)){
    upButtonCounted = 0;
  }
  
  if ((downButtonState == HIGH) && (downButtonCounted == 0)){     
    if(val)    
      Vmax = constrain(Vmax - 5, 0, 255);
    else if(sat)
      Smax = constrain(Smax - 5, 0, 255);
    else if(hue)
      H = constrain(H - 5, 0, 255);
    downButtonCounted = 1;
  }
  else if((downButtonState == LOW) && (downButtonCounted == 1)){
    downButtonCounted = 0;
  }
}

ISR(TIMER1_COMPB_vect) {}
