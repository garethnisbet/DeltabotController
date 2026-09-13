char str[] ="123,456,789,111,222,444";

   
void setup(){      
    Serial.begin( 9600 );
    }

void loop() {
  char str = Serial.read();
  char * pch;
  pch = strtok(str," ,.-");
  while (pch != NULL)
  {
    Serial.print(pch);Serial.print("\n");
    pch = strtok (NULL, " ,.-");
  }
}
