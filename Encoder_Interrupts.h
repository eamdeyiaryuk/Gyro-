#define enc1_a 18  
#define enc1_b 22  
#define enc2_a 2  
#define enc2_b 26 
#define enc3_a 19
#define enc3_b 23

int32_t pulse1, pulse2, pulse3; 

void updateEnc1()
{
  if (digitalRead(enc1_b) == 1) { pulse1--; }
  else { pulse1++; } 
}

void updateEnc2()
{
  if (digitalRead(enc2_b) == 1) { pulse2--; }
  else { pulse2++; } 
}

void updateEnc3()
{
  if (digitalRead(enc3_b) == 1) { pulse3--; }
  else { pulse3++; } 
}

void encoder_init()
{
  pinMode(enc1_a, INPUT_PULLUP);
  pinMode(enc1_b, INPUT_PULLUP);
  pinMode(enc2_a, INPUT_PULLUP);
  pinMode(enc2_b, INPUT_PULLUP);
  pinMode(enc3_a, INPUT_PULLUP);
  pinMode(enc3_b, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1_a), updateEnc1, RISING);
  attachInterrupt(digitalPinToInterrupt(enc2_a), updateEnc2, RISING);
  attachInterrupt(digitalPinToInterrupt(enc3_a), updateEnc3, RISING);
}
