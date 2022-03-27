import processing.serial.*;

Serial myPort;        // The serial port
int xPos = 1;         // horizontal position of the graph 

//Variables to draw a continuous line.
int lastxPos=1;
int lastheight=0;
int new_val = 0;
float rec_number =0;
float mapped_val=0;
void setup () {
  // set the window size:
  size(1000, 400);        

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[0], 115200);  //

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(1);      // set inital background:
}
void draw () {
  // everything happens in the serialEvent()
  if (new_val==1){
    new_val = 0;
    mapped_val = map(rec_number, 0, 65535, 0, height); //map to the screen height.
    stroke(127,34,255);     //stroke color
    strokeWeight(4);        //stroke wider
    line(lastxPos, lastheight, xPos, int(height - mapped_val)); 
    lastxPos= xPos;
    lastheight= int(height-mapped_val);
    if (xPos >= width) {
      xPos = 0;
      lastxPos= 0;
      background(0);  //Clear the screen.
    } 
    else {
      // increment the horizontal position:
      xPos++;
    }
  }
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  if (inString != null) {
    stroke(127,34,255);     //stroke color
    strokeWeight(4);        //stroke wider
    line(0, 100, 100, 100); 
    inString = trim(inString);                // trim off whitespaces.    
    float inByte = float(inString);           // convert to a number.
    rec_number = inByte;
    new_val = 1;
    println(rec_number);
  }
}
