// most launchpads have a red LED
#define LED RED_LED

//see pins_energia.h for more LED definitions
//#define LED GREEN_LED

volatile int mili_seconds;
volatile int seconds;

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(LED, OUTPUT);
  seconds = 0;
  mili_seconds = 0;
  // setup timerA
  TA0CTL = TASSEL_2 + ID_1 + MC_1;  // タイマ0を設定(SMCLK/2分周/カウントアップ)
  TA0CCR0 = 8000;                 // 8000カウントごとに割り込み
  TA0CCTL0 |= CCIE;              // enable interrupt
  digitalWrite(LED, HIGH);
}

// the loop routine runs over and over again forever:
void loop() {
  _BIS_SR(LPM1_bits + GIE);
  if ( seconds > 1 ) {
    digitalWrite(LED, LOW);
  } else {
    digitalWrite(LED, HIGH);
  }
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A (void)
{
  mili_seconds++;
  if (mili_seconds >= 1000) {
    mili_seconds = 0;
    seconds++;
  }
  if( seconds > 50 ) {
     seconds = 0;
 }
}

bool blink_times( int times ) {
  for ( int i = 0; i < times; i++ ) {
    digitalWrite(RED_LED, HIGH);
    delay(500);
    digitalWrite(RED_LED, LOW);
    delay(500);
  }
}
