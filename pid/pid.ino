//motor  right  configs
#define encoderPin1 2
#define encoderPin2 3
#define PWMPin 5
#define directionPin1 7

int proportional = 1;
float kpv = 1;
float kiv = 40;

volatile float motorPosition = 0;
float previousTime = 0;
float errorIntegral = 0;
float velocity = 0;
float pos = 0;
float errorvelocity = 0;
float controlSignal = 0;
float controlsignalvelocity = 0;
float target_pos_cm = -140;
float desiredvelocity = 27;
float prev_target_pos_cm = 0;


//motor left config
#define encoderPin1_left 21
#define encoderPin2_left 20
#define PWMPin_left 44
#define directionPin1_left 45


int proportional_left = 1;
float kpv_left = 1;
float kiv_left = 49.5;



volatile float motorPosition_left = 0;
float previousTime_left = 0;
float errorIntegral_left = 0;
float velocity_left = 0;
float pos_left = 0;
float errorvelocity_left = 0;
float controlSignal_left = 0;
float controlsignalvelocity_left = 0;
float target_pos_cm_left = -140;
float desiredvelocity_left = 27;
float prev_target_pos_cm_left = 0;









void setup() {



  Serial.begin(9600);

  //motor right
  pinMode(encoderPin1, INPUT_PULLUP);  //A
  pinMode(encoderPin2, INPUT_PULLUP);  //B
  pinMode(PWMPin, OUTPUT);
  pinMode(directionPin1, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), checkEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), checkEncoder2, RISING);


  //motor left
  pinMode(encoderPin1_left, INPUT_PULLUP);  //A
  pinMode(encoderPin2_left, INPUT_PULLUP);  //B
  pinMode(PWMPin_left, OUTPUT);
  pinMode(directionPin1_left, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin1_left), checkEncoder1_left, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2_left), checkEncoder2_left, RISING);
}

void loop() {

  previousTime = micros();//to detect real time
  previousTime_left = micros();


  for (int i = 0; i < 500; i++) {

    float currentTime = micros(),currentTime_left = micros();
    float deltaTime = ((float)(currentTime - previousTime)) / (1.0e6);  //time difference in seconds 
    previousTime = currentTime;


    
    float deltaTime_left = ((float)(currentTime_left - previousTime_left)) / (1.0e6);  //time difference in seconds
    previousTime_left = currentTime_left;


    noInterrupts();  // disable interrupts temporarily while reading
    velocity = ((motorPosition / 74.89) - pos) / deltaTime;//variable calculated by this method 1/((2*pi*r)/ppr*2(for 2 phase)) 
    velocity_left = ((motorPosition_left / 74.89)  - pos_left) / deltaTime_left;
    pos = (double(motorPosition / 74.89)) ;
     pos_left = (double(motorPosition_left / 74.89));
    interrupts();  // turn interrupts back on

    float desiredpos = target_pos_cm;

    controlSignal = (desiredpos - pos ) * proportional;


    float desiredpos_left = target_pos_cm_left;

    controlSignal_left = (desiredpos_left - pos_left) * proportional_left;



    if (fabs(controlSignal) > desiredvelocity) {
      if (controlSignal >= 0) {
        controlSignal = desiredvelocity;
      } else {
        controlSignal = -desiredvelocity;
      }
    }
    errorvelocity = (controlSignal)-velocity;
    errorIntegral = errorIntegral + errorvelocity * deltaTime;//like integration
    controlsignalvelocity = (errorvelocity * kpv) + (kiv * errorIntegral);

    int motorDirection = 1;
    if (controlsignalvelocity < 0)  //negative value: CCW
    {
      motorDirection = -1;
    }
    float PWMValue = fabs(controlsignalvelocity);  //PWM values cannot be negative and have to be integers
    if (PWMValue >= 255)                           //fabs() = floating point absolute value
    {
      //errorIntegral -= errorvelocity * deltaTime;  //antiWindup

      PWMValue = 255;  //capping the PWM signal - 8 bit
    }
if(fabs(desiredpos-pos)<1)
{
      errorIntegral -= errorvelocity * 0;  //antiWindup
      if(errorIntegral<0.05 && errorIntegral>-0.05) //bns8r el erro b3deen bnsafaro
      {
        errorIntegral=0;
      }

}
    




    if (fabs(controlSignal_left) > desiredvelocity_left) {
      if (controlSignal_left >= 0) {
        controlSignal_left = desiredvelocity_left;
      } else {
        controlSignal_left = -desiredvelocity_left;
      }
    }
    errorvelocity_left = (controlSignal_left)-velocity_left;
    errorIntegral_left = errorIntegral_left + errorvelocity_left * deltaTime_left;
    controlsignalvelocity_left = errorvelocity_left * kpv_left + kiv_left * errorIntegral_left;

    int motorDirection_left = 1;
    if (controlsignalvelocity_left < 0)  //negative value: CCW
    {
      motorDirection_left = -1;
    }
    float PWMValue_left = fabs(controlsignalvelocity_left);  //PWM values cannot be negative and have to be integers
    if (PWMValue_left >= 255)                                //fabs() = floating point absolute value
    {

      PWMValue_left = 255;  //capping the PWM signal - 8 bit
      //errorIntegral_left -= errorvelocity_left * deltaTime_left;  //antiWindup
    }
if(fabs(desiredpos_left-pos_left)<1)
{
      errorIntegral_left -= (errorvelocity_left * 0);  //antiWindup
          if(errorIntegral_left<0.05 && errorIntegral_left>-0.05)
      {
        errorIntegral_left=0;
      }

}
    
    analogWrite(PWMPin_left, PWMValue_left);
    analogWrite(PWMPin, PWMValue);
    if (motorDirection_left == 1) {
      digitalWrite(directionPin1_left, HIGH);
    } else if (motorDirection_left == -1)
      digitalWrite(directionPin1_left, LOW);
    if (motorDirection == 1) {
      digitalWrite(directionPin1, HIGH);
    } else if (motorDirection == -1)
      digitalWrite(directionPin1, LOW);    








    Serial.println(velocity);
    Serial.print("  ");
    Serial.println(desiredvelocity);
    Serial.print("  ");
  
  }
}



void checkEncoder1() {
  int encoderPin2Value = digitalRead(encoderPin2);

  if (encoderPin2Value > 0)  //CW direction
  {
    motorPosition++;
  } else {
    motorPosition--;
  }
}

void checkEncoder2() {
  int encoderPin2Value = digitalRead(encoderPin1);

  if (encoderPin2Value > 0)  //CW direction
  {
    motorPosition--;
  } else {
    motorPosition++;
  }
}






void checkEncoder1_left() {
  int encoderPin2Value_left = digitalRead(encoderPin2_left);

  if (encoderPin2Value_left > 0)  //CW direction
  {
    motorPosition_left++;
  } else {
    motorPosition_left--;
  }
}

void checkEncoder2_left() {
  int encoderPin2Value_left = digitalRead(encoderPin1_left);

  if (encoderPin2Value_left > 0)  //CW direction
  {
    motorPosition_left--;
  } else {
    motorPosition_left++;
  }
}