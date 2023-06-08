#include <Servo.h>

Servo frontsteering;
Servo backsteering;

Servo frontwheel;
Servo backwheel;
//
int fs = 1500;
int bs = 1500;
int fw = 1600;
int bw = 1600;


String strs[20];
char buff[20];
String control = "1500 1500 1600 1600";
int StringCount = 0;

void setup (void)
{
  Serial.begin (9600);

  frontsteering.attach(6);
  backsteering.attach(5);

  frontwheel.attach(4);
  backwheel.attach(3);
}  

void loop () {
  frontsteering.write(fs);
  backsteering.write(bs);
  frontwheel.write(fw);
  backwheel.write(bw);

//src: https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813/8 usr:johnwasser
  if(Serial.available() > 0) {
    StringCount = 0;
    Serial.readBytes(buff, 20);
    Serial.flush();
    String str = String(buff);

    // Split the string into substrings
    while (str.length() > 0) {
      int index = str.indexOf(' ');
      if (index == -1) { // No space found 
        strs[StringCount++] = str;
        break;
      }
      else {
        strs[StringCount++] = str.substring(0, index);
        str = str.substring(index+1);
      }
    }
///////////////////////////

    fs = strs[0].toInt();
    bs = strs[1].toInt();
    fw = strs[2].toInt();
    bw = strs[3].toInt();
  }
}
