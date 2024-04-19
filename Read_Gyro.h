float serialData1, serialData2,  yaw_, roll_;

void serialEvent3_()
{
  if(Serial3.find('#'))
  {
    serialData1 = Serial3.parseFloat();
    serialData2 = Serial3.parseFloat();
    yaw_ = int(serialData1);
    roll_ = serialData2;
    Serial.print(yaw_);  Serial.print(" ");
    Serial.println(roll_);
  }
}
