char Mymessage[5] = "Hello"; //String data
 
  void setup() { 
    // Begin the Serial at 115200 Baud
     Serial.begin(115200);
   }


  void loop() {    
    Serial.write(Mymessage,5); //Write the serial data
    delay(1000);   
   }
