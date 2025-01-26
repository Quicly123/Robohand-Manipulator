#include <avr/interrupt.h>
#include <Arduino.h>
#include "VarSpeedServo.h"

#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to tick (assumes prescale of 8)
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds


#define TRIM_DURATION       2                               // compensation ticks to trim adjust for digitalWrite delays

static servo_t servos[MAX_SERVOS];                          // static array of servo structures
static volatile int8_t Channel[_Nbr_16timers ];             // counter for the servo being pulsed for each timer (or -1 if refresh interval)

uint8_t ServoCount = 0;                                     // the total number of attached servos

servoSequencePoint initSeq[] = {{0,100},{45,100}};

#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER)) // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)       // returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])            // macro to access servo class by timer and channel

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)
{
  if( Channel[timer] < 0 )
    *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer
  else{
    if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && SERVO(timer,Channel[timer]).Pin.isActive == true )
      digitalWrite( SERVO(timer,Channel[timer]).Pin.nbr,LOW); // pulse this channel low if activated
  }

  Channel[timer]++;    // increment to the next channel
  if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {

	if (SERVO(timer,Channel[timer]).speed) {
		if (SERVO(timer,Channel[timer]).target > SERVO(timer,Channel[timer]).ticks) {
			SERVO(timer,Channel[timer]).ticks += SERVO(timer,Channel[timer]).speed;
			if (SERVO(timer,Channel[timer]).target <= SERVO(timer,Channel[timer]).ticks) {
				SERVO(timer,Channel[timer]).ticks = SERVO(timer,Channel[timer]).target;
				SERVO(timer,Channel[timer]).speed = 0;
			}
		}
		else {
			SERVO(timer,Channel[timer]).ticks -= SERVO(timer,Channel[timer]).speed;
			if (SERVO(timer,Channel[timer]).target >= SERVO(timer,Channel[timer]).ticks) {
				SERVO(timer,Channel[timer]).ticks = SERVO(timer,Channel[timer]).target;
				SERVO(timer,Channel[timer]).speed = 0;
			}
		}
	}

    *OCRnA = *TCNTn + SERVO(timer,Channel[timer]).ticks;
    if(SERVO(timer,Channel[timer]).Pin.isActive == true)     // check if activated
      digitalWrite( SERVO(timer,Channel[timer]).Pin.nbr,HIGH); // its an active channel so pulse it high
  }
  else {
    if( (unsigned)*TCNTn <  (usToTicks(REFRESH_INTERVAL) + 4) )  // allow a few ticks to ensure the next OCR1A not missed
      *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);
    else
      *OCRnA = *TCNTn + 4;
    Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel
  }
}

#ifndef WIRING
#if defined(_useTimer1)
SIGNAL (TIMER1_COMPA_vect)
{
  handle_interrupts(_timer1, &TCNT1, &OCR1A);
}
#endif

#if defined(_useTimer3)
SIGNAL (TIMER3_COMPA_vect)
{
  handle_interrupts(_timer3, &TCNT3, &OCR3A);
}
#endif

#if defined(_useTimer4)
SIGNAL (TIMER4_COMPA_vect)
{
  handle_interrupts(_timer4, &TCNT4, &OCR4A);
}
#endif

#if defined(_useTimer5)
SIGNAL (TIMER5_COMPA_vect)
{
  handle_interrupts(_timer5, &TCNT5, &OCR5A);
}
#endif

#elif defined WIRING
#if defined(_useTimer1)
void Timer1Service()
{
  handle_interrupts(_timer1, &TCNT1, &OCR1A);
}
#endif
#if defined(_useTimer3)
void Timer3Service()
{
  handle_interrupts(_timer3, &TCNT3, &OCR3A);
}
#endif
#endif


static void initISR(timer16_Sequence_t timer)
{
#if defined (_useTimer1)
  if(timer == _timer1) {
    TCCR1A = 0;             
    TCCR1B = _BV(CS11);     
    TCNT1 = 0;              
#if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF1A);      
    TIMSK |=  _BV(OCIE1A) ; 
#else
    TIFR1 |= _BV(OCF1A);     
    TIMSK1 |=  _BV(OCIE1A) ;
#endif
#if defined(WIRING)
    timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service);
#endif
  }
#endif

#if defined (_useTimer3)
  if(timer == _timer3) {
    TCCR3A = 0;             
    TCCR3B = _BV(CS31);   
    TCNT3 = 0;             
#if defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF3A);    
	ETIMSK |= _BV(OCIE3A);  
#else
    TIFR3 = _BV(OCF3A);    
    TIMSK3 =  _BV(OCIE3A) ; 
#endif
#if defined(WIRING)
    timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service);  
#endif
  }
#endif

#if defined (_useTimer4)
  if(timer == _timer4) {
    TCCR4A = 0;             
    TCCR4B = _BV(CS41);     
    TCNT4 = 0;              
    TIFR4 = _BV(OCF4A);   
    TIMSK4 =  _BV(OCIE4A) ;
  }
#endif

#if defined (_useTimer5)
  if(timer == _timer5) {
    TCCR5A = 0;             
    TCCR5B = _BV(CS51);     
    TCNT5 = 0;              
    TIFR5 = _BV(OCF5A);     
    TIMSK5 =  _BV(OCIE5A) ;
  }
#endif
}

static void finISR(timer16_Sequence_t timer)
{
    
#if defined WIRING   
  if(timer == _timer1) {
    #if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    TIMSK1 &=  ~_BV(OCIE1A) ;  
    #else
    TIMSK &=  ~_BV(OCIE1A) ;  
    #endif
    timerDetach(TIMER1OUTCOMPAREA_INT);
  }
  else if(timer == _timer3) {
    #if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    TIMSK3 &= ~_BV(OCIE3A);    
    #else
    ETIMSK &= ~_BV(OCIE3A);    
    #endif
    timerDetach(TIMER3OUTCOMPAREA_INT);
  }
#else
#endif
}

static boolean isTimerActive(timer16_Sequence_t timer)
{
  // returns true if any servo is active on this timer
  for(uint8_t channel=0; channel < SERVOS_PER_TIMER; channel++) {
    if(SERVO(timer,channel).Pin.isActive == true)
      return true;
  }
  return false;
}


/****************** end of static functions ******************************/

VarSpeedServo::VarSpeedServo()
{
  if( ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    
	  servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   
    this->curSeqPosition = 0;
    this->curSequence = initSeq;
  }
  else
    this->servoIndex = INVALID_SERVO ;  
}

uint8_t VarSpeedServo::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t VarSpeedServo::attach(int pin, int min, int max)
{
  if(this->servoIndex < MAX_SERVOS ) {
    pinMode( pin, OUTPUT) ;                                   
    servos[this->servoIndex].Pin.nbr = pin;
    this->min  = (MIN_PULSE_WIDTH - min)/4; 
    this->max  = (MAX_PULSE_WIDTH - max)/4;
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if(isTimerActive(timer) == false)
      initISR(timer);
    servos[this->servoIndex].Pin.isActive = true;
  }
  return this->servoIndex ;
}

void VarSpeedServo::detach()
{
  servos[this->servoIndex].Pin.isActive = false;
  timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if(isTimerActive(timer) == false) {
    finISR(timer);
  }
}

void VarSpeedServo::write(int value)
{

  byte channel = this->servoIndex;
  servos[channel].value = value;

  if(value < MIN_PULSE_WIDTH)
  {
    value = constrain(value, 0, 180);
    value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
  }
  this->writeMicroseconds(value);
}

void VarSpeedServo::writeMicroseconds(int value)
{
  byte channel = this->servoIndex;
  servos[channel].value = value;

  if( (channel >= 0) && (channel < MAX_SERVOS) )   
  {
    if( value < SERVO_MIN() )          
      value = SERVO_MIN();
    else if( value > SERVO_MAX() )
      value = SERVO_MAX();

  	value -= TRIM_DURATION;
    value = usToTicks(value);  

    uint8_t oldSREG = SREG;
    cli();
    servos[channel].ticks = value;
    SREG = oldSREG;


	servos[channel].speed = 0;
  }
}


void VarSpeedServo::write(int value, uint8_t speed) {

  byte channel = this->servoIndex;
  servos[channel].value = value;

	if (speed) {

		if (value < MIN_PULSE_WIDTH) {
			value = constrain(value, 0, 180);
			value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
		}

		if( (channel >= 0) && (channel < MAX_SERVOS) ) {
			value = constrain(value, SERVO_MIN(), SERVO_MAX());

			value = value - TRIM_DURATION;
			value = usToTicks(value);

			uint8_t oldSREG = SREG;
			cli();
			servos[channel].target = value;
			servos[channel].speed = speed;
			SREG = oldSREG;
		}
	}
	else {
		write (value);
	}
}

void VarSpeedServo::write(int value, uint8_t speed, bool wait) {
  write(value, speed);

  if (wait) {
    if (value < MIN_PULSE_WIDTH) {
      while (read() != value) {
        delay(5);
      }
    } else {
      while (readMicroseconds() != value) {
        delay(5);
      }
    }
  }
}

void VarSpeedServo::stop() {
  write(read());
}

void VarSpeedServo::slowmove(int value, uint8_t speed) {
  write(value, speed);
}



int VarSpeedServo::read()
{
  return  map( this->readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int VarSpeedServo::readMicroseconds()
{
  unsigned int pulsewidth;
  if( this->servoIndex != INVALID_SERVO )
    pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION ;
  else
    pulsewidth  = 0;

  return pulsewidth;
}

bool VarSpeedServo::attached()
{
  return servos[this->servoIndex].Pin.isActive ;
}

uint8_t VarSpeedServo::sequencePlay(servoSequencePoint sequenceIn[], uint8_t numPositions, bool loop, uint8_t startPos) {
  uint8_t oldSeqPosition = this->curSeqPosition;

  if( this->curSequence != sequenceIn) {
    this->curSequence = sequenceIn;
    this->curSeqPosition = startPos;
    oldSeqPosition = 255;
  }

  if (read() == sequenceIn[this->curSeqPosition].position && this->curSeqPosition != CURRENT_SEQUENCE_STOP) {
    this->curSeqPosition++;

    if (this->curSeqPosition >= numPositions) { // at the end of the loop
      if (loop) { // reset to the beginning of the loop
        this->curSeqPosition = 0;
      } else { // stop the loop
        this->curSeqPosition = CURRENT_SEQUENCE_STOP;
      }
    }
  }

  if (this->curSeqPosition != oldSeqPosition && this->curSeqPosition != CURRENT_SEQUENCE_STOP) {
    write(sequenceIn[this->curSeqPosition].position, sequenceIn[this->curSeqPosition].speed);
  }

  return this->curSeqPosition;
}

uint8_t VarSpeedServo::sequencePlay(servoSequencePoint sequenceIn[], uint8_t numPositions) {
  return sequencePlay(sequenceIn, numPositions, true, 0);
}

void VarSpeedServo::sequenceStop() {
  write(read());
  this->curSeqPosition = CURRENT_SEQUENCE_STOP;
}

void VarSpeedServo::wait() {
  byte channel = this->servoIndex;
  int value = servos[channel].value;

  if (value < MIN_PULSE_WIDTH) {
    while (read() != value) {
      delay(5);
    }
  } else {
    while (readMicroseconds() != value) {
      delay(5);
    }
  }
}

bool VarSpeedServo::isMoving() {
  byte channel = this->servoIndex;
  int value = servos[channel].value;

  if (value < MIN_PULSE_WIDTH) {
    if (read() != value) {
      return true;
    }
  } else {
    if (readMicroseconds() != value) {
      return true;
    }
  }
  return false;
}
