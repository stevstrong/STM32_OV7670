/**
 * Serial Data Dumper 
 * 
 * Opens a serial connection and communicates with Maple Mini analog recorder.
 * Features:
 * - setup serial port parameters
 * - menu to choose one of the recording options:
 *   - setup recording parameters (recording time, sampling frequency, channels per sequence)
 *   - start recording
 *   - dump recorded data
 *
 */
import processing.serial.*;

int BUFF_SIZE = 2048;
Serial myPort;      // The serial port
String portName = "";
int bRate = 0;
int whichKey = 0; int myKey = 0; // Variable to hold keystoke value
int[] keys = new int[5];  // max 5 key possibilities
int inByte = -1;    // Incoming serial data
int serial_ok = -1;
int rec_ok = -1;
byte[] inBuffer = new byte[BUFF_SIZE];
int index = 0;
int cnt = 0;
int tim = 0;
int TIMEOUT = 100;  // millis
int rec_time = 1000, sampling_freq = 26, samples_per_seq = 4;
boolean bin_rec;
int bin_len;
byte[] data = new byte[520];
byte screen = 0;
String[] disp = new String[30];
int status = 0;
int FRAME_X = 800;
int FRAME_Y = 700;
/*********************************************************************/
void setup()
{
  size(800, 700);
  background(0);
  // create a font with the third font available to the system:
  //PFont myFont = createFont(PFont.list()[8], 14);
  PFont myFont = createFont("Courier new", 16);
  textFont(myFont);
  for (int i=0;i<disp.length; i++)  disp[i] = "";  // initialize string array
  ListSerial();  // init serial port
}
/********************************************************************/
void ListSerial()
{
  disp_line = 0;
  DisplayClearLines();
  int len = Serial.list().length;
  if ( len==0 ) {
    DisplayAddLine("No port available, cannot send or receive!",0,0);
    return;
  } else {
    DisplayAddLine("Select serial port:",0,0);
    DisplayAddLine("------------------------",-1,0);
    for ( byte i=0; i<len; i++)  DisplayAddLine("["+i+"]"+" : "+Serial.list()[i],-1,0);
    serial_ok = 0;
  }
}
/********************************************************************/
void CheckSerial()
{
	if ( serial_ok<2 ) return;  // do it only if serial setup was successful.

	if ( myPort.available()<=0 ) return;	// no data available

	if ( bin_rec ) {  // binary reception
		ParseBinaryData();
	} else {  // string reception
		ParseStringData(myPort.readString());
	}
}
/********************************************************************
arrayCopy(src, srcPosition, dst, dstPosition, length)
arrayCopy(src, dst, length)
********************************************************************/
/********************************************************************/
/* Binary packet format:
start_id	[0x01]
data_length	[len_high] [len_low]
payload		[dd] ... [dd]
crc			[crc_high] [crc_low]
/********************************************************************/
void Tick() {
	int tim1 = millis();
	print('.');
	while ( (millis()-tim1)<20 );	// wait some time
}
/********************************************************************/
//byte[] sBuf = new byte[1000];
/********************************************************************/
int GetBytesFromSerial()
{
	int c;
	int tim2 = millis();
	while ( (c=myPort.available())<=0 ) {
		Tick();
		if ( (millis()-tim2)>1000 ) {
			println("! timeout receiving data from serial!");
			return -1;
		};
	}	// wait to receive something
	//buf = new byte[c];
	return myPort.readBytes(inBuffer);
}
/********************************************************************/
/********************************************************************/
int GetPayload1(int bytes)
{
	int c=0,i;
	// reserve image buffer
	imgBuffer = new byte[bytes];
	for ( i=0; i<bytes; ) {
	//while ( bytes>0 ) {
		c = GetBytesFromSerial();
		if ( c<0 ) {
			println("! incomplete payload, received only "+i+" bytes");
			return 1;
		}
		//imgBuffer = concat(imgBuffer, iBuf);  // add new bytes to the data array
		arrayCopy(inBuffer,0,imgBuffer,i,c);
		i += c;
	}
	index += i;
	//println("got payload bytes: "+pl_index);
	//print('.');
	return 0;
}
/********************************************************************/
/********************************************************************/
int GetByteFromSerial()
{
	int tim2 = millis();
	while ( myPort.available()<=0 ) {	// wait to receive something
		Tick();
		if ( (millis()-tim2)>1000 ) {
			println("! timeout receiving data from serial!");
			return -1;
		};
	}
	return myPort.read();
}
/********************************************************************/
byte[] imgBuffer;
/********************************************************************/
int GetPayload(int bytes)
{
	int c,i;
	// reserve image buffer
	imgBuffer = new byte[bytes];
	for ( i=0; i<bytes; ) {
		c = GetByteFromSerial();
		if ( c<0 ) {
			println("! incomplete payload, received only "+i+" bytes");
			return 1;
		}
		imgBuffer[i++] = (byte)c;
	}
	index += i;
	return 0;
}
/********************************************************************/
int IMG_X = 160;
int IMG_Y = 80;
/********************************************************************/
void GetImageData()
{	// reeive data as ASCII string
	int i,dat, bytes = IMG_X*IMG_Y;
	char c;
	String s = "";
	PImage img = createImage(IMG_X, IMG_Y, ALPHA);
	img.loadPixels();
	for (i=0; i<bytes; i++) {
		c = (char)GetByteFromSerial();
		if ( c<0 ) {
			println("! incomplete image data 1");
			break;
		}
		s += c;
		c = (char)GetByteFromSerial();
		if ( c<0 ) {
			println("! incomplete image data 2");
			break;
		}
		s += c;
//		dat += unhex(c);
		dat = unhex(s);
		// store image value
		img.pixels[i] = dat;
	}
	img.updatePixels();
	println(". image received ok.");
	bin_rec = false;
}
/********************************************************************/
/********************************************************************/
void ParseBinaryData()
{
	// read the available bytes
	int a, len = 0;
	// get header ID
	if ( (a=GetByteFromSerial())<0 ) {
		return;
	}

	if ( a==0x01 ) {	// parse header info
		// get here payload length
		len = GetByteFromSerial()<<8;
		len += GetByteFromSerial();
		println("- payload length: "+len);	// debug
		// wait for entire payload reception
		if ( GetPayload(len)>0 ) return;
		//if ( GetPayload1(len)>0 ) return;
		myPort.write(0x06);	// send acknowledge
/*
		if ( (index+len)<=data.length ) {
			arrayCopy(inBuffer,0,data,index,len);
			index += len;  // update data index
			DisplayAddLine("index = "+index,-1,-1);
		} else {
			println("!data buffer overflow!");
			DisplayAddLine("!data buffer overflow!",-1,0);
		}
	*/
	} else if ( a==0x17 ) {	// end of transmission block
		// stop here the binary recording
		tim = (millis() - tim);
		//println("time diff: "+tim);
		if ( imgBuffer!=null ) saveBytes("img.raw", imgBuffer);
		println("\n. finished receiving "+index+" bytes in "+tim+" millis.");
		DisplayAddLine("finished receiving "+index+" bytes in "+tim+" millis.",-1,0);
		bin_rec = false;
		bin_len = 0;
		// display image
		ShowImage();
		//break;
	} else {
		println(". wrong binary ID received: 0x"+hex(a,2));
		//DisplayAddLine("!wrong binary ID!",-1,0); 
		return;
	}
}
/********************************************************************/
/********************************************************************/
void ShowImage()
{
	PImage img = createImage(IMG_X, IMG_Y, RGB);
	img.loadPixels();
	int dat;
	for (int i = 0; i < img.pixels.length; i++) {
		img.pixels[i] = color(int(imgBuffer[i]));
	}
	img.updatePixels();
	fill(255);
	rect(0,300,4*IMG_X+2,4*IMG_Y+2);
	image(img,1,301,4*IMG_X,4*IMG_Y);	
}
/********************************************************************/
String[] q,m,rest;
/********************************************************************/
void ParseStringData(String inStr)
{
	q = splitTokens(inStr, "\r\n");
	int len = q.length;
	//println("-> "+len+" tokens");
	for (byte i=0; i<q.length; i++) {
		//DisplayAddLine(i+": "+q[i], -1,0);  // display received serial data
		//println(i+": "+q[i]);
		println(q[i]);
		// check binary marker
		if ( q[i].indexOf(">>>")>=0 ) {
			println(". binary transmission initiated.");
			DisplayAddLine("receiving data...", -1,0);
			bin_rec = true;
			index = 0;
			// send the acknowledge byte
			//println("Sending ACK");
			//myPort.buffer(1);
			myPort.write(0x06);
			tim = millis();	// start measuring time

		}
/*		else if (bin_len==0) {
			// parse the binary length
			m = match(q[i], "binary_length:\\s*(\\d*)");
			if ( m!=null ) {
				bin_len = parseInt(m[1]);
				println("bin_len = "+bin_len);
				DisplayAddLine("bin_len = "+bin_len,-1,0);
				data = new byte[bin_len];
			}
		}*/
	}
}
/********************************************************************/
/********************************************************************/
void SetupSerial()
{
  switch (serial_ok) {
    case 0:  // show selected serial port and Baud rate options
      if (whichKey>='0' && whichKey<'0'+Serial.list().length) { // check valid selection
        // Open selected port
        portName = Serial.list()[whichKey-'0'];
        DisplayAddLine("- selected serial port: "+portName,0,0);
        String[] scr1 = {"","Select one configuration (bitrate, data bits, parity, stop bits):",
                            "-----------------------------------------------------------------",
                          "[0] : 115200, 8, N, 1","[1] : 250000, 8, N, 1","[2] : 500000, 8, N, 1"};
        DisplayAddLines(scr1,-1,0);
        serial_ok ++;  // goto next parameter selection
      }
      break;
    case 1:  // show selected Baud rate and data bits option
      switch (whichKey) {
        case '0':  bRate = 115200;  break;
        case '1':  bRate = 250000;  break;
        case '2':  bRate = 500000;  break;
        default:   bRate = 0; break;
      }
      if ( bRate>0 ) {
          myPort = new Serial(this, portName, bRate, 'N', 8, 1);  // Maple Mini analog recorder
          //myPort = new Serial(this, portName, 115200, 'E', 8, 2);  // EnergyCam
          if ( myPort!=null ) {
            DisplayAddLine("Serial port "+portName+", "+bRate+", 8, 'N', 1 opened successfully.",0,0);
            DisplayClearLines();
            //ListRecordingOptions();
            serial_ok ++;  // goto next parameter selection
			//myPort.bufferUntil('\n');
          } else {
            DisplayAddLine("Opening port "+portName+", "+bRate+", 8, 'N', 1 ... failed!",-1,-1);
            DisplayAddLine("Check serial port and try again (press 'n')",-1,0);
          }
      }
      break;
    case 2:  // get new input key
		myPort.write(whichKey);
      break;
    default:  break;
  }
}
/********************************************************************/
int disp_line = 0;
/********************************************************************/
void DisplayAddLine(String str, int line, int offset)
{
  //println("display in line "+disp_line+": "+str);  // debug
  int dLen = disp.length;
  int scroll = (disp_line+1+offset)-dLen;
  if ( scroll>0) {
    disp_line = dLen-1;
    //println("scroll is: "+scroll);
  }
  while ( line==-1 && scroll>0 ) {  // have to scroll?
    for (int i=8; i<dLen-1; i++) disp[i] = disp[i+1];  // scroll all lines one up
    scroll--;
  }
  if ( line==-1 ) {  // add to last available line
    disp[disp_line+=offset] = str;
    if ((++disp_line)>dLen) disp_line = dLen;  // limit check for line
  } else {
    disp[line] = str;
    disp_line = line+1;
  }
  ShowScreen();  // refresh display
}
/********************************************************************/
void DisplayAddLines(String[] scr, int line, int offset)
{
  DisplayAddLine(scr[0],line,offset);  // insert first line
  for ( int i=1; i<scr.length; i++) DisplayAddLine(scr[i],-1,0);
}
/********************************************************************/
void DisplayClearLines()
{
  for ( int i=disp_line; i<disp.length; i++) disp[i]="";
  //PromptReset();
}
/********************************************************************/
void ShowScreen()
{
  background(0);  // clear screen
  for (byte i=0; i<disp.length; i++) {
    if ( (disp[i]).length()>0)  text(disp[i], 10, 40+20*i);
  }
  //println("ShowScreen: disp_line = "+disp_line);
}
/********************************************************************/
void draw()
{
  if ( myKey>0 ) { // process here the new pressed key
    whichKey = myKey;
    myKey = 0;
    if (whichKey<' ') println("- pressed key: "+whichKey);  // debug
    else println("- pressed key: '"+(char)whichKey+"'");  // debug
    //if ( rec_ok>=0 )  SetupRecording();
    if ( serial_ok>=0 )  SetupSerial();
    ShowScreen();  // display text lines
    //ShowPrompt();
    whichKey = 0;  // reset received key
  }
  CheckSerial();
}
/*********************************************************************/
void keyPressed() {
  // Send the keystroke out:
  myKey = key;
}
