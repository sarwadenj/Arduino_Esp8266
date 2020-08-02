
void motor_setup()
{
  //Pinouts of PWM motors
  for(int i = 0; i < 3; i++)
    pinMode(motor[i], OUTPUT);
  
  analogWriteFreq(500);

 // pinMode(MOTOR_ARMED, OUTPUT);
 // digitalWrite(MOTOR_ARMED, LOW);

 /* for (int i = 0 ; i < 4 ; i++)
  {
    pinMode(motor[i], OUTPUT);
    esc[i].attach(motor[i], PWM_MIN, PWM_MAX); //1000 and 2000
   // digitalWrite(motor[i], LOW);
    esc[i].writeMicroseconds(0);
  }*/
}

void motor_compute_outputs()
{
  
  //Uncomment for y,p,r,t -> individual LEDs
  //for(int i=0 ; i<4 ; i++)
    //analogWrite(motor[i], rcInput[i]);
  
  motorValue[0] = double(rcInput[3]);// - pidRateOut[1] - pidRateOut[2];// - pidRateOut[0]; // motor_speed[0] = throttle - pitch - roll - yaw
  motorValue[1] = double(rcInput[3]);// + pidRateOut[1] + pidRateOut[2];// - pidRateOut[0]; // motor_speed[0] = throttle + pitch + roll - yaw
  motorValue[2] = double(rcInput[3]);// - pidRateOut[1] + pidRateOut[2];// + pidRateOut[0]; // motor_speed[0] = throttle - pitch + roll + yaw
  motorValue[3] = double(rcInput[3]);// + pidRateOut[1] - pidRateOut[2];// + pidRateOut[0]; // motor_speed[0] = throttle + pitch - roll + yaw

  for (int i = 0 ; i < 4 ; i++)
  {
    if(motorValue[i] < 0)
      motorValue[i] = 0;
  }

  armCondition = (armMotor == 0) && (yawInput >= 20) && (thrInput <= 10);
  disarmCondition = (disarmMotor == 1) && (yawInput <= -20) && (thrInput <= 10);

  if ((armCondition == 1) || (disarmCondition == 1))
    hold_3secs();
    
  else {
    t0_Millis = 0;
    flag = 0;
  }
  
  //if(imuStable && armMotor == 1) {
  if(armMotor) {
    //motor_armed();
    /*unsigned long armBlinkTn = millis(); // when motors are armed, LED blinks
    if((armBlinkTn - armBlinkT0) > 100) {
      armBlinkT0 = armBlinkTn;
      if (armLedState == LOW) {
        armLedState = HIGH;
      }
      else {
        armLedState = LOW;
      }
     // digitalWrite(MOTOR_ARMED, armLedState);
    }
  }  */
  motor_write();
  }
}

void motor_armed()
{
  if (rcInput[3] < ARMING_THR_LIMIT)
  {
    for (int i = 0 ; i < 4 ; i++)
      motorValue[i] = 0;
    //Serial.println("thrInput < ARMING_THR_LIMIT");
  }
}
/*WiFi.begin(ssid,Wrong_password);
unsigned long startMillis = millis();
unsigned long currentMillis = millis();
while (WiFi.status() != WL_CONNECTED) {
  delay(500);// refer->A
  currentMillis = millis();
  if ((currentMillis - startMillis) > 20000) {
    //return error code for not connecting even after 20 seconds.
    break;            
  }
}

//Handle Successfull connection.
if (WiFi.status() == WL_CONNECTED) {
    // Send valid ip reponse to AJAX call. 
}
else {
  // Send wifi.status() code as response to ajax.
  //WiFi.disconnect();
}
  */

void hold_3secs()
{
    unsigned long currentMillis = millis();
    if (flag == 0) {
      t0_Millis=currentMillis;
      flag = 1;
    }
    if (flag==1 && (currentMillis - t0_Millis) > 1000)
      flag = 2;

    if (flag==2 && (currentMillis - t0_Millis) > 2000)
       flag = 3;
       
    if (flag==3){
      if(armCondition == 1){
        armMotor = 1;
        disarmMotor = 1;
        Serial.println("Motors armed...............................................................................................................");
        motor_armed(); 
      //  digitalWrite(MOTOR_ARMED, HIGH);
      }
      else if(disarmCondition == 1) {
        armMotor = 0;
        disarmMotor = 0;
        Serial.println("Motors disarmed................................................................................................................");         
        for (int i = 0 ; i < 4 ; i++)
          motorValue[i] = 0;;
       // digitalWrite(MOTOR_ARMED, LOW);
      }
    }  
}

void motor_write()
{
  for (int i = 0 ; i < 4 ; i++)
  {
    analogWrite(motor[i], motorValue[i]);
    //esc[i].writeMicroseconds(motorValue[i]);
    //delay(10);
    //digitalWrite(MOTOR_ARMED, !arm);    
#ifdef DEBUG_MOTOR_PWM
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(motorValue[i]);
    Serial.print("\t");
#endif
  }
#ifdef DEBUG_MOTOR_PWM
 Serial.println();
#endif
#ifdef LOG_YPR_AND_PWM
    Serial.print(motorValue[0]);
    Serial.print(" ");
    Serial.print(motorValue[1]);
    Serial.print(" ");
    Serial.print(motorValue[2]);
    Serial.print(" ");
    Serial.print(motorValue[3]);
    
    Serial.print(" ");
    Serial.print(ypr[0]);
    Serial.print(" ");
    Serial.print(ypr[1]);
    Serial.print(" ");
    Serial.println(ypr[2]);
    //Serial.print(" ");
    
#endif
}

