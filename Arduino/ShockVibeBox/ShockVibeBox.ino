// Constants
const int D_PINS[] = {22, 26, 30, 34, 38, 42, 46, 50};
const int A_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7};
const int outputPin = 53;

// Config variables
char dConfig[8]; // 0: off, 1: PNP/NPN falling edge, 2: PNP/NPN rising edge
char aConfig[4]; // 0: off, 1: analog current, 2: analog voltage
int threshHigh[4] = {0}; // Analog high threshold value (0-1023)
int threshLow[4] = {0}; // Analog low threshold value (0-1023)
unsigned long threshTime; // Time threshold for discrete level crossing detection (microseconds)
unsigned long triggerDuration; // Duration of 3.3V output when event is logged (microseconds)

// Temp variables to convert incoming serial bytes to integers
char configRead[1];
char thresholdRead[2];
char threshTimeRead[4];
char triggerDurationRead[4];

// Time-related variables
unsigned long currentTime;
unsigned long lastReadTime = 0;
unsigned long serialUpdateInterval = 100; // Time in milliseconds between serial writes
                                          // This must be longer than the interval between serial queue reads on the PC side
                                          // otherwise the GUI may lag behind the Arduino. Using 2x (i.e. write to Arduino buffer
                                          // every 100ms, read from serial queue on PC every 50ms) seems to be working well.
unsigned long eventTimer[12] = {0};
unsigned long triggerTimer = 0;
int triggerState = 0; // 0: trigger is off
                      // 1: trigger is on

// Discrete-related variables
unsigned long currentDiscrete;
int discreteState[8] = {0}; // 0: failing side of HIGH/LOW (time threshold already reached)
                            // 1: passing side of HIGH/LOW
                            // 2: failing side of HIGH/LOW (waiting for time threshold or transition back to passing side)



// Analog-related variables
unsigned long currentAnalog;
int currentPin;
int analogState[4] = {0}; // 0 = outside threshold window (time threshold already reached)
                          // 1 = inside threshold window
                          // 2 = outside threshold window (waiting for time threshold or transition back to passing side)
int hysteresis = 20; // analog hysteresis value

//      Output array
//
//   0: Time (milliseconds)
// 1-8: Event counters (discrete channels 1-8)
//   9: Analog counter 1
//  10: Analog value 1
//  11: Analog counter 2
//  12: Analog value 2
//  13: Analog counter 3
//  14: Analog value 3
//  15: Analog counter 4
//  16: Analog value 4
unsigned long outputs[17] = {0};


void setup() {
  // Set digital pin 53 to function as output
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, LOW);
   
  // Set other digital pins to receive inputs
  for(int i = 0; i < (sizeof(D_PINS)/sizeof(int)); i++) {
    pinMode(D_PINS[i], INPUT);
  }
  
  // Establish serial connection
  Serial.begin(115200);
  while(!Serial); // Wait for connection to establish

  // Wait for incoming serial data from PC
  while(Serial.available() == 0);

  // Read incoming serial discrete config value
  for(int i = 0; i < 8; i++) {
    Serial.readBytes(configRead, 1);
    dConfig[i] = configRead[0];
    char configRead[1] = {0};
  }

  // Read incoming serial analog config value
  for (int i = 0; i < 4; i++) {
    Serial.readBytes(configRead, 1); // Current or voltage
    aConfig[i] = configRead[0];
    Serial.readBytes(thresholdRead, 2); // Low threshold
    threshLow[i] = thresholdRead[0];
    threshLow[i] = threshLow[i] + (thresholdRead[1] << 8);
    Serial.readBytes(thresholdRead, 2);
    threshHigh[i] = thresholdRead[0]; // High threshold
    threshHigh[i] = threshHigh[i] + (thresholdRead[1] << 8);
  }

  // Read incoming serial time threshold value
  Serial.readBytes(threshTimeRead, 4);
  for (int i = 0; i<4; i++) {
    threshTime += threshTimeRead[i] << i*8;
  }

  // Read incoming trigger duration value
  Serial.readBytes(triggerDurationRead, 4);
  for (int i = 0; i<4; i++) {
    triggerDuration += triggerDurationRead[i] << i*8;
  }

  // Initial analog pin read
  lastReadTime = millis();
  for(int i = 0; i < 4; i++) {
    if(aConfig[i] == 0) {
      currentAnalog = 0;
    }
    else if(aConfig[i] == 1) {
      currentAnalog = analogRead(A_PINS[i*2]);
      outputs[(i*2)+10] = currentAnalog;
    }
    else {
      currentAnalog = analogRead(A_PINS[(i*2)+1]);
      outputs[(i*2)+10] = currentAnalog;
    }
    
    if(currentAnalog < threshHigh[i] && currentAnalog > threshLow[i]) {
      analogState[i] = 1;
    }
    else {
      analogState[i] = 0;
    }
  }

  outputs[0] = lastReadTime;
  Serial.write((byte*)outputs, 68); // Write data to serial buffer for output
}


void loop() {
  
  // Read discrete pins
  for(int i = 0; i < 8; i++) {
    if(dConfig[i] == 1) { // Looking for sensor rising edges == looking for Arduino falling edges
      currentDiscrete = digitalRead(D_PINS[i]);
      if(discreteState[i] == 0) {
        if(currentDiscrete == HIGH) {
          discreteState[i] = 1;
        }
      }
      else if(discreteState[i] == 1) {
        if(currentDiscrete == LOW) {
          eventTimer[i] = micros();
          discreteState[i] = 2;
        }
      }
      else if(discreteState[i] == 2) {
        if(checkTimeThresh(eventTimer[i], threshTime)) {
          outputs[i+1]++;
          discreteState[i] = 0;
          activateTrigger();
        }
        else {
          if(currentDiscrete == HIGH) {
            discreteState[i] = 1;
          }
        }
      }
    }
    else if(dConfig[i] == 2) { // Looking for sensor falling edges == looking for Arduino rising edges
      currentDiscrete = digitalRead(D_PINS[i]);
      if(discreteState[i] == 0) {
        if(currentDiscrete == LOW) {
          discreteState[i] = 1;
        }
      }
      else if(discreteState[i] == 1) {
        if(currentDiscrete == HIGH) {
          eventTimer[i] = micros();
          discreteState[i] = 2;
        }
      }
      else if(discreteState[i] == 2) {
        if(checkTimeThresh(eventTimer[i], threshTime)) {
          outputs[i+1]++;
          discreteState[i] = 0;
          activateTrigger();
        }
        else {
          if(currentDiscrete == LOW) {
            discreteState[i] = 1;
          }
        }
      }
    } 
  }

  // Read analog pins, store read values, compare to last read values, increment counters accordingly
  for(int i = 0; i < 4; i++) {
    if(aConfig[i] == 0) { // Channel is turned 'off' by user. No further action needed on this channel.
      currentAnalog = 0;
    }
    else {
      if(aConfig[i] == 1) { // Channel is analog current
        currentPin = i*2;
      }
      else { // Channel is analog voltage
        currentPin = i*2 + 1;
      }
      currentAnalog = analogRead(currentPin); // Read value from input pin
      
      if(analogState[i] == 0) { // Channel is confirmed fail, looking for first reading inside threshold window
        if((currentAnalog > (threshLow[i] + hysteresis)) && currentAnalog < (threshHigh[i] - hysteresis)) {
          analogState[i] = 1;
        }
      }
      else if(analogState[i] == 1) { // Channel is passing, lookign for first reading outside threshold window
        if(currentAnalog > threshHigh[i] || currentAnalog < threshLow[i]) {
          eventTimer[i+8] = micros();
          analogState[i] = 2;
        }
      }
      else if(analogState[i] == 2) { // Channel is outside threshold window, but not yet for the duration required to fail
        if(checkTimeThresh(eventTimer[i+8], threshTime)) { // Check time since first out of window reading, if over threshold, flag as fail
          outputs[i*2 + 9]++;
          analogState[i] = 0;
          activateTrigger();
        }
        else { // If time threshold is not yet met, check if reading is inside window. If so, reset state to 1
          if((currentAnalog > (threshLow[i] + hysteresis)) && currentAnalog < (threshHigh[i] - hysteresis)) {
            analogState[i] = 1;
          }
        }
      }
    }
    outputs[(i*2)+10] = currentAnalog; // Store current values (0-1023) for each channel
  }

  // if trigger output is on, check time and turn off if trigger duration is exceeded
  if (triggerState == 1) {
    if(checkTimeThresh(triggerTimer, triggerDuration)) {
      digitalWrite(outputPin, LOW);
      triggerState = 0;
    }
  }
  
  currentTime = millis();
  if (currentTime - lastReadTime >= 100) { // Write outputs to serial if 100 or more ms elaspsed since last write
    outputs[0] = currentTime;
    Serial.write((byte*)outputs, 68);
    lastReadTime = currentTime;
  } 
}


// This function checks if an event duration has surpassed the threshold time, returns true or false
// This solves the problem of the micros() function rolling over after 4294967295 microseconds,
// or 01:11:34.967 in HH:MM:SS
boolean checkTimeThresh(unsigned long eventStart, unsigned long eventDuration) {
  unsigned long timeToRollover = 4294967295 - eventStart; // microseconds until next rollover
  if(timeToRollover >= eventDuration) { // threshold time would be reached prior to rollover
    if(micros() - eventStart >= eventDuration) {
      return true;
    }
    else {
      return false;
    }
  }
   else { // rollover prior to threshold time
    if(micros() + timeToRollover >= eventDuration) {
      return true;
    }
    else {
      return false;
    }
  }   
}

// This function activates the 3.3V external trigger
void activateTrigger() {
  digitalWrite(outputPin, HIGH);
  triggerState = 1;
  triggerTimer = micros();
}


