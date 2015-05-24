/*************************************************************************
* Test sketch for SIM800 library
* Distributed under GPL v2.0
* Written by Stanley Huang <stanleyhuangyc@gmail.com>
* For more information, please visit http://arduinodev.com
*************************************************************************/

#include "SIM800.h"

#define APN "connect"
static const char* url = "http://arduinodev.com/datetime.php";


CGPRS_SIM800 gprs;
uint32_t count = 0;
uint32_t errors = 0;

void setup()
{
  con.begin(9600);
  while (!con);

  con.println("SIM800 TEST");

  for (;;) {
    con.print("Resetting...");
    while (!gprs.init()) {
      con.write('.');
    }
    con.println("OK");
    
    con.print("Setting up network...");
    byte ret = gprs.setup(APN);
    if (ret == 0)
      break;
    con.print("Error code:");
    con.println(ret);
    con.println(gprs.buffer);
  }
  con.println("OK");
  delay(3000);  
  
  if (gprs.getOperatorName()) {
    con.print("Operator:");
    con.println(gprs.buffer);
  }
  int ret = gprs.getSignalQuality();
  if (ret) {
     con.print("Signal:");
     con.print(ret);
     con.println("dB");
  }
 
#if 0
  sendCommand("AT+CSQ");
  lcd.println("Dialing...");
  if (sendCommand("ATD + +61402533012;", 10000)) {
    lcd.println(buffer); 
  }
  if (sendCommand("ATH"))
    lcd.println(buffer);
  delay(1000);
#endif

  gprs.sendCommand("AT+CMGF=1");    // sets the SMS mode to text
  gprs.sendCommand("AT+CPMS=\"SM\",\"SM\",\"SM\""); // selects the memory

  for (;;) {
    if (gprs.httpInit()) break;
    con.println(gprs.buffer);
    gprs.httpUninit();
    delay(1000);
  }
  delay(3000);
}

void loop()
{
  count++;
  
  char mydata[16];
  sprintf(mydata, "t=%lu", millis());
  con.print("Requesting ");
  con.print(url);
  con.print('?');
  con.println(mydata);
  gprs.httpConnect(url, mydata);
  while (gprs.httpIsConnected() == 0) {
    // can do something here while waiting
    con.write('.');
    for (byte n = 0; n < 25 && !gprs.available(); n++) {
      delay(10);
    }
  }
  if (gprs.httpState == HTTP_ERROR) {
    con.println("error");
    errors++;
    delay(3000);
    return; 
  }
  gprs.httpRead();
  int ret;
  while ((ret = gprs.httpIsRead()) == 0) {
    // can do something here while waiting
  }
  if (gprs.httpState == HTTP_ERROR) {
    con.println("error");
    errors++;
    delay(3000);
    return; 
  }

  // now we have received payload
  con.print("\n[Payload]");
  con.println(gprs.buffer);
  
  // show stats  
  con.print("Total Requests:");
  con.print(count);
  if (errors) {
    con.print(" Errors:");
    con.print(errors);
  }
  con.println();
}

