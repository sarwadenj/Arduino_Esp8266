
void dmp_data_ready()
{
  imuInterrupt = true;
}

void imu_setup()
{
  //IMU Setup
  //pinMode(IMU_GROUND, OUTPUT); // pin S3
  //digitalWrite(IMU_GROUND, HIGH);
  //delay(10);
  //delay(2);
  //digitalWrite(IMU_GROUND, LOW);
  pinMode(BUILT_IN_LED, OUTPUT);
  digitalWrite(BUILT_IN_LED, HIGH);

  Wire.begin(D1,D2);
  Wire.setClock(400000); // 400kHz I2C clock (200kHz if CPU is 8MHz)

  //Initialize Device
  Serial.println(F("Initializing I2C devices..."));
  imu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(imu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  pinMode(INTERRUPT_PIN, INPUT); //pin D4

  Serial.println(F("Initializing DMP..."));
  devStatus = imu.dmpInitialize();  //Load and configure the DMP

  //Supply Gyro offsets here, scaled for minimum sensitivity:-
  imu.setXGyroOffset(220);
  imu.setYGyroOffset(76);
  imu.setZGyroOffset(-85);
  imu.setZAccelOffset(1788);

  //Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    imu.setDMPEnabled(true);  //Turn on the DMP

    //Enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmp_data_ready, RISING);
    imuIntStatus = imu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = imu.dmpGetFIFOPacketSize();
  }

  /*else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        
        Serial.println(F(")"));
    } */
  yawCompare = true;
}




void imu_ypr()
{
  //I2C and (Yaw, Pitch, Roll) code for IMU
  if (!dmpReady) return;  //Wait for imu interrupt or extra packet(s) available

  imuInterrupt = false;    //Reset interrupt flag
  imuIntStatus = imu.getIntStatus();  //Get INT_STATUS byte
  fifoCount = imu.getFIFOCount();     //Get current FIFO count

  //Check for overflow (this should never happen unless our code is too inefficient)
  if ((imuIntStatus & 0x10) || fifoCount == 1024) {
    imu.resetFIFO();   // reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  
  //Otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (imuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
      fifoCount = imu.getFIFOCount();  // wait for correct available data length

    imu.getFIFOBytes(fifoBuffer, packetSize);  // read a packet from FIFO
    imu.resetFIFO();
    fifoCount -= packetSize;  //Track FIFO count here in case there is > 1 packet available
    //This lets us immediately read more without waiting for an interrupt

    update_ypr_gyro();

    if(!imuStable)
      stabilize_imu();
      
    if (flagSetOffsets)
    {
      set_offsets(1); //Set all offsets
      flagSetOffsets = 0;
    }
    if(imuStable && abs(yawInput) < 3)
      set_offsets(0); //Set only yaw offset
      
    update_offsets();

    //Printing Gyro values
#ifdef DEBUG_IMU_GYRO
    Serial.print("gyro\t");
    Serial.print(g.x);
    Serial.print("\t");
    Serial.print(g.y);
    Serial.print("\t");
    Serial.println(g.z);
    Serial.flush();
#endif  

    //Printing Yaw, Pitch and Roll
#ifdef DEBUG_IMU_YPR
    Serial.print("YAW: ");
    Serial.print(ypr[0]);
    Serial.print("\tPITCH: ");
    Serial.print(ypr[1]);
    Serial.print("\tROLL:");
    Serial.println(ypr[2]);
    Serial.flush();
#endif
  }
}

void update_ypr_gyro()
{
  imu.dmpGetQuaternion(&q, fifoBuffer);
  imu.dmpGetGravity(&gravity, &q);
  imu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  //Converting to degrees
  ypr[0] = ypr[0] * 180/M_PI;
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;

  //ypr[0] = -ypr[0];  //Reversing Yaw
  //ypr[1] = -ypr[1];  //Reversing Pitch
  
  //Get gyro values   (uncomment declaration of 'g' to use)
  imu.dmpGetGyro(&g, fifoBuffer);
    
  //gyro_average_compute();
}

void stabilize_imu()
{
  if (millis() > 1100 && yawCompare == true)
  {
    if (yawComparePrint == 1)
    {
      Serial.print("Checking IMU Stability");
      yawComparePrint = 0;
    }

    yawCompareCounter++;
    if (yawCompareCounter == 150)
    {
      Serial.print("..");
      if (abs((ypr[0]) - (yawPrevious)) < 0.01)
      {
        yawCompare = false;
        imuStable = 1;
        Serial.println();
        Serial.println("IMU stabilized");
        digitalWrite(BUILT_IN_LED, LOW);
      }
      else
      {
        yawPrevious = ypr[0];
        yawCompareCounter = 0;
      }
    }
  }
}

void gyro_average_compute()
{
  gyroAverage = float((abs(g.x) + abs(g.y) + abs(g.z)) / 3);
#ifdef DEBUG_IMU_GYRO
  Serial.println(gyroAverage);
#endif
}

void set_offsets(byte yawOrAll)  //0 for only yaw, 1 for all
{
  if(yawOrAll)
  {
    offset[0] = ypr[0];
    offset[1] = pitchOffset;
    offset[2] = rollOffset;
  }
  else
    offset[0] = ypr[0];
}

void update_offsets()
{
  for (int i = 0 ; i < 3 ; i++)
  {
    ypr[i] -= offset[i];
#ifdef DEBUG_IMU_OFFSETS
    Serial.print("Offset ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(offset[i]);
    Serial.print("\t\t");
#endif
  }
#ifdef DEBUG_IMU_OFFSETS
  Serial.println();
#endif
}

