float FromPC1, FromPC2, FromPC3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()>0)
  {
    FromPC1 = Serial.parseFloat();
    FromPC2 = Serial.parseFloat();   
    FromPC3 = Serial.parseFloat();   
    if (Serial.read()!= '\n')
    {
      Serial.print("Warning parsing numbers!");
      Serial.print("got: ");
      Serial.print(FromPC1);
      Serial.print(",");
      Serial.print(FromPC2);
      Serial.print(",");
      Serial.println(FromPC3);     
    }    
    else
    {
        Serial.print("Good got: ");
        Serial.print(FromPC1);
        Serial.print(",");
        Serial.print(FromPC2);
        Serial.print(",");
        Serial.println(FromPC3);     
//        float PMax = 0.5;
 /*       if (FromPC3 > PMax || FromPC2 > PMax || FromPC3 > PMax)
        {
          Serial.print("Warning, some pressures too high! Something wrong with: ");          
        }
        else
        {
          setBar(1,FromPC1);  
          setBar(2,FromPC2);  
          setBar(3,FromPC3);       
        }*/
    }
  }

}
