
#define N 6
#define MAX_PWM 150
int ports[N] = {11,12,7,8,6,9};

float FromPC1, FromPC2, FromPC3;
float PMax = 1.8; // pression maximale le 1.5 bar (soit 150kPa)
//int bool_led = 0;

void setPMWs(int value);

void setup() {
  // put your setup code here, to run once:
 // pinMode(A6, OUTPUT);     // Pin 6 en output // pour des tests avec des LED
 // pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  //digitalWrite(A6, HIGH);
  //Serial.print("TestBase"); 
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
     Serial.print("Test1");
//        Serial.print("got: ");
//        Serial.print(FromPC1);
//        Serial.print(",");
//        Serial.print(FromPC2);
//        Serial.print(",");
//        Serial.println(FromPC3);     
        if (FromPC1 > PMax || FromPC2 > PMax || FromPC3 > PMax)
        {
          Serial.print("Warning, some pressures too high! Something wrong with: ");          
        }
        else
        {
          Serial.print("Test2"); 
          setBar(1,FromPC1);  
          setBar(2,FromPC2);  
          setBar(3,FromPC3);       
        }
    }
  }
}

void setBar(int chan, float value)
{ 
  // 10V = 3bar
  // 1V = 0.3 bar
  float voltage = value/0.3;
  float PWM = voltage / 5 * 255; 
  int v = constrain(PWM,0,MAX_PWM);
  
  analogWrite(ports[chan-1],v);
}

/*
void setPMWs(int value)
{
  for(int i = 0;i<N;i++)
  {
    analogWrite(ports[i],value);
  }
}*/
