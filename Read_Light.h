#define readLight 21 //light


void light_init()
{
  pinMode(readLight, INPUT);
}

bool is_light()
{
  bool digitalData = digitalRead(readLight);
  
  if (digitalData == 1)
  {
    return true;
    //Serial.println("Green");
  }
  else
  {
    return false;
    //Serial.println("No green");
  }
}
