/*
	STM32 based OV7670 handling
	================================
	Pin connections:
	----------------
		OV7670	STM32F103C8T6 generic
	1.	Vcc		3.3V
	2.	GND		GND
	3.	SCL		PB6
	4.	SDA		PB7
	5.	Vsync		PB4	(in)
	6.	Href		PB5	(in)
	7.	Pclk		PB3	(in)
	8.	Xclk		PB0	(out)
	9.	D7..D0	PA7..PA0	(in)

 */
 
#include "libmaple/timer.h"
#include <Wire.h>
#include "ov7670.h"

// pin definitions
#define SCL_PIN		PB6
#define SDA_PIN		PB7
#define VSYNC_PIN	PB4
#define HREF_PIN	PB5
#define PCLK_PIN	PB3
#define XCLK_PIN	PB0	// timer 3 channel 3

#define LED_PIN 	PC13
#define LED_ON		( (GPIOC_BASE)->BSRR = BIT13 )
#define LED_OFF		( (GPIOC_BASE)->BRR = BIT13 )

#define IDR_P	( (GPIOB_BASE)->IDR )
#define VSYNC	(IDR_P & BIT4)	// PB4
#define HREF	(IDR_P & BIT5)	// PB5
#define PCLK	(IDR_P & BIT3)	// PB3

#define SCCB Wire

char str[150];	// for sprintf
int debug;
/*****************************************************************************/
/* Configuration: this lets you easily change between different resolutions
 * You must uncomment only one, no more no less */

//#define USE_VGA
//#define USE_QVGA
#define USE_QQVGA	// 160x120

/*****************************************************************************/
// Timer 3 channel 3 is mapped on PB0 (see Reference Manual table 44)
// Set Timer 3 channel 3 to generate PWM with 8MHz
/*****************************************************************************/
#define TIMER_RELOAD			4	// to divide the system clock
#define TIMER_FREQUENCY			( 72000000/TIMER_RELOAD )
/*****************************************************************************/
void Timer_SetupPWM()
{
	// set timer 3 channel 3 in PWM 1 mode
	timer_dev *timerDevice  = TIMER3;
	uint8 timerChannel      = TIMER_CH3;
	timer_init(timerDevice);	// feed timer 3 registers with system clock
	pinMode(XCLK_PIN, PWM);
	timer_pause(timerDevice);	// stop timer
	timer_set_reload(timerDevice, (TIMER_RELOAD-1));
	timer_set_compare(timerDevice, timerChannel, TIMER_RELOAD/2);	// duty cycle = 50%
	timer_set_mode(timerDevice, timerChannel, TIMER_PWM);
	timer_resume(timerDevice);	// let timer run
}
/*****************************************************************************/
void setup()
{
	debug = 1;	// let serial output print debug messages
	Serial.begin(500000); // USB is always 12 Mbit/sec
	Serial.println();
	delay(5000);
	Serial.println("*** STM32 application for OV7670 camera ***");
	// initialize digital pins
	pinMode(LED_PIN, OUTPUT);
	pinMode(VSYNC_PIN, INPUT);
	pinMode(HREF_PIN, INPUT);
	pinMode(PCLK_PIN, INPUT);
	//pinMode(XCLK_PIN, OUTPUT); - done in timer setup
	
	// setup data pins
	Serial.print("Configuring ports...");
	for(int p = PA0; p<PA0+8; p++) {
		pinMode(p,INPUT);
	}
	Serial.println("done."); delay(10);
	//Check_SDA_SCL();
	Serial.print("Configuring camera...");
		Timer_SetupPWM();	// start generating XCLK
		//SCCB.begin();
		Cam_Init();
	Serial.println("done."); delay(10);
	// read pixels
	//	CountLinePixels();
}

/*****************************************************************************/
/*****************************************************************************/
#define NR_LINES			120	// QQVGA resolution
#define MISSING_LINES	40	// due to RAM limitations we cannot store all lines
#define BUF_SIZE_Y	(NR_LINES-MISSING_LINES)
#define BUF_SIZE_X	160
uint8_t img_buf[BUF_SIZE_Y][BUF_SIZE_X] __attribute__((packed,aligned(1)));	// store only 100 lines a 156 bytes, packed in 32 bit wide data - to easy access
//uint16_t pxl_buf[640];
uint8_t line_buf[BUF_SIZE_X] __attribute__((packed,aligned(1)));
int hist[256];
//int hist_pixels;
#define CAM_DATA	( ((GPIOA_BASE)->IDR) & 0x000000FF )	// store only the lower 8 bits
/*****************************************************************************/
static void CountLinePixels(void)
{
uint16_t pck, line;

	int wa = 2;	// do warm-up frames, optional, set it to 0 if not used
	while ( (wa--)>0) {
		//Wait for vsync
		Serial.print(F("wait for Vsync high...\n"));
			while( !VSYNC ); //wait for high
		Serial.print(F("wait for Vsync low...\n"));
			while( VSYNC ); //wait for low
	}

	line = 0;
	while ( !VSYNC && line<BUF_SIZE_Y )	// don't process all lines - not enough RAM
	{
		pck = 0;

		noInterrupts();
		// wait for HREF
		while( !HREF );	//wait for high
		while ( HREF )	// process while high
		{
			while( !PCLK ); // wait for high
			LED_ON;	// debug
/**/
			if ( pck&0x01 ) {	// store only the Y component, the odd bytes, in case of YUV422
				img_buf[line][pck>>1] = CAM_DATA;
			}

			while( PCLK ); // wait for low
			LED_OFF;	// debug

			pck++;
		}
		interrupts();

		//pxl_buf[line++] = pck;	// store nr. of detected pixel clocks for each row - debug only
		line ++;
		// end of line, check for next line or vertical sync
		while ( !HREF && !VSYNC );
	}
	Serial.println("> image acquisition done."); //delay(10);
/*
	for (int i=0; i<line; i++) {
		sprintf(str, "line %02u\: %u\n", i, pxl_buf[i]); Serial.print(str);
	}
*/
	SendImgData();
	IMG_Parse_chars();
}
/*****************************************************************************/
/*****************************************************************************/
void Blink(int ms_on, int ms_off)
{
	LED_ON;
	delay(ms_on);
	LED_OFF;
	delay(ms_off);
}
/*****************************************************************************/
/*****************************************************************************/
void BlinkError()
{
	for (int i=0;i<5;i++) Blink(50,50);
}
/*****************************************************************************/
/*****************************************************************************/
int COM_GetAck(void)
{	// wait for 0x06 acknowledge
	byte rec;
	uint32_t tim = millis();
	while ( 1 ) {
		if ( Serial.available() ) {
			rec = Serial.read();
			//Blink(100,0);
			if ( rec==0x06 ) return 1;
			else return 0;
		}
		if ( (millis()-tim)>2000) {
			//Serial.println("ERROR: ACK reception timed out!");
			BlinkError();
			return 0;
		}
	}
}
/*****************************************************************************/
/*****************************************************************************/
void SendImgData()
{
#define BUF_SIZE (BUF_SIZE_X * BUF_SIZE_Y)

	Serial.print(">>>");
	if ( COM_GetAck()==0 ) {
		return;	//Serial.write(0x05);	// send binary enquiry
	}

	Serial.write(0x01);	// start of heading
	Serial.write((byte)(BUF_SIZE>>8));	// pack length high byte
	Serial.write((byte)(BUF_SIZE&0xFF));	// pack length low byte
#if 0
	// print the data as PGM format
	sprintf(str, "P5\n%u %u\n255\n", BUF_SIZE_Y, BUF_SIZE_X); Serial.print(str);
	for (int j=0; j<BUF_SIZE_Y; j++) {
		for (int i=0; i<BUF_SIZE_X; i++) {
			Serial.write(img_buf[j][i]);
		}
	}
#else
	// print out the data to serial
	for (int j=0; j<BUF_SIZE_Y; j++) {
		for (int i=0; i<BUF_SIZE_X; i++) {
			//sprintf(str, "%02x", img_buf[i][j]); Serial.print(str);
			Serial.write(img_buf[j][i]);
		}
	}
#endif
	if ( COM_GetAck()==0 ) {
		return;	//Serial.write(0x05);	// send binary enquiry
	}

	delay(250);	// leave time for host to process previous data
	Serial.write(0x17); 	// end of transmission block
	delay(250);	// leave time for host to switch back to ASCII reception
}
/*****************************************************************************/
// the loop function runs over and over again forever
/*****************************************************************************/
void loop()
{	// restart image data reading if serial character detected
	if ( Serial.available()>0 ) {
		//while ( Serial.available()>0 )
		char c = Serial.read();	// read all dummy bytes
		if ( c=='a')
			CountLinePixels();	// read pixels
		else if ( c=='h' )
			SendHistogram();
		else if ( c=='c' )
			IMG_Parse_chars();
		else
			SendImgData();
	}
}
/*****************************************************************************/
/*****************************************************************************/
/*
Character identification parameters used for OCR recognition:
------------------------------------------------------------------------------------------------------------
	| Wide | Upper-line | Middle-line | Bottom-line | Upper-left | Bottom-left | Upper-right | Bottom-right |
-----------------------------------------------------------------------------------------------------------|
1  |   -  |      -     |       -     |       -     |      -     |       -     |      -      |      -       |
2  |   1  |      1     |       1     |       1     |      -     |       1     |      1      |      -       |
3  |   1  |      1     |       1     |       1     |      -     |       -     |      1      |      1       |
4  |   1  |      -     |       1     |       -     |      1     |       -     |      -      |      1       |
5  |   1  |      1     |       1     |       1     |      1     |       -     |      -      |      1       |
6  |   1  |      1     |       1     |       1     |      1     |       1     |      -      |      1       |
7  |   1  |      1     |       -     |       -     |      -     |       -     |      1      |      1       |
8  |   1  |      1     |       1     |       1     |      1     |       1     |      1      |      1       |
9  |   1  |      1     |       1     |       1     |      1     |       -     |      1      |      1       |
0  |   1  |      1     |       -     |       1     |      1     |       1     |      1      |      1       |
------------------------------------------------------------------------------------------------------------
Conditions:
Wide: 			character width (chr_w) >= CHAR_WIDE
row:				number of pixels per row  ppr >= PIXELS_PER_ROW
line:				number of consecutive rows (row_w) >= ROW_WIDTH
line position:	Upper:	starting position of the line (row start, rs) < ROW_WIDTH
					Middle:	( rs >= rw ) && ( rs <= (CHAR_HEIGHT - 2*ROW_WIDTH) )
					Bottom:	rs > (CHAR_HEIGHT - 2 x ROW_WIDTH)
column:			number of pixels per column  ppc >= PIXELS_PER_COL
segment:			number of consecutive columns col_w >= COL_WIDTH ( == ROW_WIDTH ???)
segment position:	Upper:	segment starting position ss < CHAR_HEIGHT/2
					bottom:		ss >= CHAR_HEIGHT/2

					
*/
/**************************************************************************************************/
// extracts a row from the image array
/**************************************************************************************************/
void IMG_Get_row(int index_y)
{
	memcpy(line_buf, &img_buf[index_y][0], BUF_SIZE_X);
}
/**************************************************************************************************/
// extracts a column from the image array
/**************************************************************************************************/
void IMG_Get_col(int index_x)
{
	for(int y=0; y<BUF_SIZE_Y; y++) {
		line_buf[y] = img_buf[y][index_x];
	}
}
/**************************************************************************************************/
// calculates the histogram of the image
/**************************************************************************************************/
void SendHistogram(void)
{
	IMG_Get_histogram();
	// Print out histogram values
	Serial.println("Histogram:");
	for (int j=0; j<256; j++) {
		sprintf(str, "%03u: %u\n", j, hist[j]); Serial.print(str); delay(5);
	}
}
/**************************************************************************************************/
// calculates the histogram of an area of image
/**************************************************************************************************/
int IMG_Get_histogram_of_area(int x_st, int y_st, int x_end, int y_end)
{
	// clear histogram array
	for(int i=0; i<256; i++) {hist[i] = 0;}
	int hist_pixels = 0; // global
	// accumulate histogram, take only the middle part of the image
	for ( ; y_st<=y_end; y_st++) {	// columns
		for ( int ix=x_st; ix<=x_end; ) {	// lines
			hist[img_buf[y_st][ix++]] ++;
			hist_pixels++;
		}
	}
	if (debug>1) {sprintf(str, "got histogram of %i, %i, %i, %i, including %i pixels.\n", x_st, y_st, x_end, y_end, hist_pixels); Serial.print(str);}
	return hist_pixels;
}
/**************************************************************************************************/
// calculates the histogram of the image
/**************************************************************************************************/
int IMG_Get_histogram(void)
{
#define OFFSET_Y		10
#define OFFSET_X		5

	return CHAR_Get_adaptive_white_value((int)OFFSET_X, (int)OFFSET_Y, (int)(BUF_SIZE_X-OFFSET_X), (int)(BUF_SIZE_Y-OFFSET_Y));
}
/**************************************************************************************************/
// extracts the white value from the histogram
/**************************************************************************************************/
int HIST_Get_white_threshold(void)
{
}
/**************************************************************************************************/
// returns the total number of pixels for a character
/**************************************************************************************************/
int CHAR_Get_Pixels(int x_st, int y_st, int x_end, int y_end)
{
	// calculates the number of pixels of a char
	return (x_end-x_st+1)*(y_end-y_st+1);
}
/**************************************************************************************************/
// extracts the white value from the histogram
/**************************************************************************************************/
int CHAR_Get_adaptive_white_value(int x_st, int y_st, int x_end, int y_end)
{
	// find a white level at which the number of white pixels is at least ...
#define FILL_MIN	30 // percent of total pixels
	// first, get the histogram
	int hist_pixels = IMG_Get_histogram_of_area(x_st, y_st, x_end, y_end);	// histogram stored in public array hist[]

	int his_min = hist_pixels*FILL_MIN/100; // number of  white pixels must be above this value

	int his = 0, white_pixels = 0, his_max = 0;
	int h;
	for ( h=255; h>0; h--) {	// ignore black level for simplicity
		his = hist[h];
		white_pixels += his;
		// find the first peak of  brightest values
		if ( his>his_max ) {
			his_max = his;	// update peak value
		} else {	// first falling histogram value
			// check if number of white pixels is above minimum prescribed value
			if (his_max>0 && white_pixels>=his_min)
				break;
		}
	}
	if (debug>1) {sprintf(str, "Adaptive white level: %i, total white pixels: %i, total pixels: %i\n", h, white_pixels, hist_pixels); Serial.print(str);}
	return h;
}
/**************************************************************************************************/
// determines the white threshold value for a specific white peak and number of pixels involved
// T = m +k*sqrt(sum(pi^2)/NP - m^2);
// where:
// m = pixels median grey value
// pi = each pixel value
// NP = number of pixels
// k = {-0.2 ... +0.2} - to be determined empirically
/**************************************************************************************************/
int CHAR_Get_Niblack_white(int x_st, int y_st, int x_end, int y_end)
{
int r,c,m=0,cnt=0,tmp;
long int val=0;
#define K 10
	// get median pixel value
	for ( r=y_st; r<=y_end; r++) {
		for ( c=x_st; c<=x_end; c++) {
			tmp = img_buf[r][c];
			m += tmp;
			val += tmp^2;
			cnt++;
		}
	}
	m = m/cnt;
	if (debug>1) {sprintf(str, "Niblack median: %i, A: %li, pixels: %i\n", m, val, cnt); Serial.print(str);}
	val = m - (sqrt(val/cnt - m^2))/K;
	return (int)val;
}
/**************************************************************************************************/
// determines the white threshold value for a specific white peak and number of pixels involved
/**************************************************************************************************/
int CHAR_Get_white_value(int x_st, int y_st, int x_end, int y_end)
{
	// first, get the histogram
	IMG_Get_histogram_of_area(x_st, y_st, x_end, y_end);	// histogram stored in public array hist[]
	// find the first peak of the brightest values which is greater than peak_min
#define PEAK_MIN	0//10	// t.b.d. experimentally after several readings
	int h = 255, his = 0, his_max = 0, h_max = 0, h_cnt;
	while( h>0) {	// ignore black level for simplicity
		his = hist[h];
		if ( his>his_max ) {
			his_max = his;	// update peak value
			h_max = h;
			h_cnt = 10;
		} else {	// first falling histogram value
		// decrement counter, it must be the peak of last ... consecutive values
			if (h_max>0 && (--h_cnt)<=0)
				break;
		}
		h --;
	}
	if ( h==0 ) {his_max = 0; h_max = 0;}	// invalid all
	if (debug>1) {sprintf(str, "White peak: %i, counts: %i\n", h_max, his_max); Serial.print(str);}
	//return his_max;
	return (h_max*9)/10;	// lower the white threshold 10% below the peak
}
/**************************************************************************************************/
//#define WHITE_THRESHOLD	100	// threshold white level
#define MAX_CHARS	6
#define MAX_ROWS	2
//
typedef struct {	// absolute coordinates of the image buffer
	int x1;	// starting x coordinate
	int y1;	// starting y coordinate
	int x2;	// ending x coordinate
	int y2;	// ending y coordinate
	int thres; // calculated threshold value in CHAR_Trim()
	int val; // parsed numerical value
} coord_t;
static coord_t coords[MAX_ROWS][MAX_CHARS];
typedef struct {	// all values are percentage of width and height
	int x1;	// starting x coordinate
	int y1;	// starting y coordinate
	int x2;	// ending x coordinate
	int y2;	// ending y coordinate
} region_t;
const region_t regions[] = {	// for 7 segment like characters
	{40, 00,100, 20}, // region 1
	{00, 30, 30, 40}, // region 2
	{65, 20,100, 35}, // region 3
	{33, 40, 66, 60}, // region 4
	{00, 60, 33, 80}, // region 5
	{75, 60,100, 80}, // region 6
	{20, 80, 40,100}, // region 7
};
/**************************************************************************************************/
// returns the number of pixels found in the specified area
/**************************************************************************************************/
//int CHAR_Get_region_value(coord_t * my_coord, int rg) --- doesnt work dunno why ...
int CHAR_Get_region_value(int ro, int chr, int rg)
{
	coord_t my_coord = coords[ro][chr];
	int x_st = my_coord.x1;
	int y_st = my_coord.y1;
	if (y_st==0) return -1;
	int x_end = my_coord.x2;
	int y_end = my_coord.y2;
	int wid = x_end - x_st + 1;
	int hei = y_end - y_st + 1;
	int thres = my_coord.thres;
	rg--;	// regions start with 1, array index starts with 0
	x_end = x_st + (wid*regions[rg].x2-1)/100;
	y_end = y_st + (hei*regions[rg].y2-1)/100;
	x_st = x_st + (wid*regions[rg].x1-1)/100;
	y_st = y_st + (hei*regions[rg].y1-1)/100;
	int r,c,cnt = 0;
	for ( r=y_st; r<=y_end; r++) {
		for ( c=x_st; c<=x_end; c++) {
			if ( img_buf[r][c]>=thres )
				cnt++;
		}
	}
	if (debug>1) {sprintf(str, "region: %i, coord: \[%i, %i, %i, %i\] value is: %i\n", rg+1, x_st, y_st, x_end, y_end, cnt); Serial.print(str);}
	return cnt;
}
/**************************************************************************************************/
// returns the detected number dependent on the available pixels in different regions of the characters
/**************************************************************************************************/
int CHAR_Parse_numbers(void)
{
#define LOCAL_THRESHOLD	5
	int r,c,cnt=0;
	int num = -1;
	for ( r=0; r<MAX_ROWS; r++) {
		for ( c=0; c<MAX_CHARS; c++) {
			if (coords[r][c].thres<=0) continue;
			if (debug>1) {sprintf(str, "parsing row %i, char %i\n", r, c); Serial.print(str);}
			if ( CHAR_Get_region_value(r,c,1)<LOCAL_THRESHOLD ) { // only '4' has region 1 empty
				//num = 4;
				if ( CHAR_Get_region_value(r,c,7)<LOCAL_THRESHOLD ) { // sometimes '5' has also region 1=0
					num = 4;
				} else {
					num = 5;
				}
			} else if ( CHAR_Get_region_value(r,c,2)<LOCAL_THRESHOLD ) { // possible numbers: '2','3','7'
				if ( CHAR_Get_region_value(r,c,5)>LOCAL_THRESHOLD ) {
					num = 2;	// only '2' has region 5 non-empty
				} else if ( CHAR_Get_region_value(r,c,7)<LOCAL_THRESHOLD ) {
					num = 7;
				} else {
					num = 3;
				}
			} else {	// possible numbers: '0','5','6','8','9'
				if ( CHAR_Get_region_value(r,c,4)<LOCAL_THRESHOLD ) {
					num = 0;
				} else { // possible numbers: '5','6','8','9'
					if ( CHAR_Get_region_value(r,c,5)<LOCAL_THRESHOLD ) { // possible numbers '5','9'
						if ( CHAR_Get_region_value(r,c,3)<LOCAL_THRESHOLD ) {
							num = 5;
						} else {
							num = 9;
						}
					} else { // possible numbers '6','8'
						if ( CHAR_Get_region_value(r,c,3)<LOCAL_THRESHOLD ) {
							num = 6;
						} else {
							num = 8;
						}
					}
				}
			}
			coords[r][c].val = num;
			cnt++;
		}
	}
	return cnt;
}
/**************************************************************************************************/
/**************************************************************************************************/
void IMG_Parse_chars(void)
{
#define SPACE_PIXEL_COUNT_THRESHOLD_Y	27	// minimum consecutive black pixels to detect separator
#define CHAR_PIXEL_COUNT_THRESHOLD_X	10	// minimum consecutive rows to detect character
#define CHAR_MAX_WIDTH	20
#define CHAR0_OFFSET	5 // pixels from left to start at

int sp_y_st;	// detected space start position on y direction
int sp_y_end;	// detected space end position on y direction, 
				// only >0 if more than SPACE_PIXEL_COUNT_THRESHOLD_Y consecutive space pixels detected
int sp_y_cnt;
int chr_x_st, chr_x_cnt, sp_x_cnt;
int y_min, y_max;
int white_thres;
int c,r;
int rows,chars;
	// get image histogram, get right white level
	white_thres = IMG_Get_histogram();
	// reset the values to invalid
	for (rows=0; rows<MAX_ROWS; rows++) {
		for ( chars=0; chars<MAX_CHARS; chars++) {
			coords[rows][chars].thres = -1;
			coords[rows][chars].val = -1;
		}
	}

	for (rows=0; rows<MAX_ROWS; rows++) {
		chars = 0;
		// first, detect spacing separators between horizontal characters
		y_min = 200; y_max = 0;
		chr_x_st = 0; chr_x_cnt = -1;
		for (c=CHAR0_OFFSET; c<BUF_SIZE_X; c++) {
			//IMG_Get_col(c);	// load column pixel data
			sp_y_st = 0; sp_y_end = 0; sp_y_cnt = -1;
			// check character and space conditions
			for (r=(rows*(BUF_SIZE_Y/2)); r<((rows+1)*(BUF_SIZE_Y/2)); r++) {	// upper character row
				//if (img_buf[r][c]<WHITE_THRESHOLD) {
				if (img_buf[r][c]<white_thres) {
					// possible black pixel detected
					if ( (++sp_y_cnt)==0 ) {
						sp_y_st = r; // mark space starting position
					}
				} else {
					if ( sp_y_cnt>=SPACE_PIXEL_COUNT_THRESHOLD_Y ) {
						//sp_y_end = r-1;
						break;
					}
					// reset space detection
					sp_y_cnt = -1;
					y_min = 200; y_max = 0;
				}
			}
			// final check after last row in the column
			if ( sp_y_cnt>=SPACE_PIXEL_COUNT_THRESHOLD_Y ) {
				sp_y_end = r-1;
				if ( sp_y_end > y_max ) {y_max = sp_y_end;}
				if ( sp_y_st < y_min ) {y_min = sp_y_st;}
			}

			// Split characters in the x direction
			if ( sp_y_end>0 ) {
				// possible space detected
				if ( chr_x_cnt>=CHAR_PIXEL_COUNT_THRESHOLD_X ) {
					// character end detected
					coords[rows][chars].x1 = chr_x_st;
					coords[rows][chars].y1 = y_min;
					coords[rows][chars].x2 = c-1;
					coords[rows][chars].y2 = y_max;
					coords[rows][chars].thres = 0;
					chars++;
					if ( chars>=MAX_CHARS ) {
						//Serial.println("ERROR: Too many characters detected!");
						break;
					}
				}
				// reset char detection parameters if char was not entirely detected
				chr_x_cnt = -1;
				y_min = 200; y_max = 0;
			} else {
				if ( (++chr_x_cnt)==0 ) {
					chr_x_st = c;
				}
			}
		}
/*		// final check after the last column
		if ( chr_x_cnt>=CHAR_PIXEL_COUNT_THRESHOLD_X && chars<MAX_CHARS ) {
			coords[rows][chars].x1 = chr_x_st;
			coords[rows][chars].y1 = y_min;
			coords[rows][chars].x2 = c-1;
			coords[rows][chars].y2 = y_max;
			chars++;
		}*/
		if (debug>1) {sprintf(str, "Detected in row %i, chars: %i\n", rows, chars); Serial.print(str);}
			
	}

// trim the characters, remove top and bottom empty lines
	CHAR_Trim();

// and now parse the value
	CHAR_Parse_numbers();

// we are done. Print the characters
	int ix1,iy1,ix2,iy2,thres,i,sum=0,val;
	for (rows=0; rows<MAX_ROWS; rows++) {
		for ( chars=0; chars<MAX_CHARS; chars++) {
			val = coords[rows][chars].val;
			if (val<0) continue;
			str[100+(rows*10)+chars] = '0'+val;
			str[100+(rows*10)+chars+1] = '\0';
			if (debug>1) {
				//sum += val*(10^(MAX_CHARS-chars-1));
				ix1 = coords[rows][chars].x1;
				iy1 = coords[rows][chars].y1;
				ix2 = coords[rows][chars].x2;
				iy2 = coords[rows][chars].y2;
				thres = coords[rows][chars].thres;
				delay(200); // to get complete chars printed
				sprintf(str, "===== char: %i, thres: %i; value: %i =====\n", chars, thres, val); Serial.print(str);
				for ( r=iy1; r<=iy2; r++) {
					i = 0;
					for ( c=ix1; c<=ix2; c++) {
						//if ( img_buf[r][c]>WHITE_THRESHOLD )
						if ( img_buf[r][c]>=thres )
							str[i++] = 'O'; //Serial.write('O');
						else 
							str[i++] = '_'; //Serial.write('_');
					}
					if (debug>1) {str[i++] = '\n'; Serial.write(str,i);} //Serial.write('\n'); //delay(100);
				}
			}
		}
	}
	// print the final numbers
	sprintf(str, "First number: %s\nSecond number: %s", &str[100], &str[110]); Serial.print(str);
}

/*****************************************************************************/
/*****************************************************************************/
void CHAR_Trim(void)
{
#define CHAR_MIN_HEIGTH	15
	// begin in the middle line and go up and down to detect end of character by a black line
	int rows, chars, r, c;
	int ix1,iy1,ix2,iy2,thres;
	int wid, hei;
	for ( rows=0; rows<MAX_ROWS; rows++) {
		if (debug>1) {sprintf(str, "trimming row %i ...\n", rows); Serial.print(str);}
		for ( chars=0; chars<MAX_CHARS; chars++) {
			// step 0: check for reliability
			if (coords[rows][chars].thres<0) continue; // not detected character
			// step 1: trim vertically
			ix1 = coords[rows][chars].x1;
			iy1 = coords[rows][chars].y1;
			if (iy1>=BUF_SIZE_Y) continue; // invalid character
			ix2 = coords[rows][chars].x2;
			if (ix2==0) continue; // invalid character
			iy2 = coords[rows][chars].y2;
			if (iy2==0) continue; // invalid character
			if ((iy2-iy1)<CHAR_MIN_HEIGTH) {
				coords[rows][chars].thres = -1;	// set to invalid
				continue;
			}
			//thres = CHAR_Get_Niblack_white_value(ix1,iy1,ix2,iy2);
			thres = CHAR_Get_adaptive_white_value(ix1,iy1,ix2,iy2);
			coords[rows][chars].thres = thres;
			wid = (ix2+ix1+1)/2;
			hei = (iy2+iy1+1)/2;
			//sprintf(str, "trimming char: %i; using threshold: %i\n", chars, thres); Serial.print(str);
			// go up
			for ( r=hei; r>=iy1; r--) {
				for ( c=ix1; c<=ix2; c++) {
					if ( img_buf[r][c]>=thres )
						break;
				}
				if (c>=ix2)
					break;	// this is end of char in upper direction
			}
			// update character coordinate
			iy1 = r+1;
			coords[rows][chars].y1 = iy1;
			// go down
			for ( r=hei; r<=iy2; r++) {
				for ( c=ix1; c<=ix2; c++) {
					if ( img_buf[r][c]>=thres )
						break;
				}
				if (c>=ix2)
					break;	// this is end of char in lower direction
			}
			// update character coordinate
			iy2 = r-1;
			coords[rows][chars].y2 = iy2;
			hei = iy2-iy1+1;
			// step 2: trim horizontally: begin in the middle column and go first left and then right
			// go left
			for ( c=wid; c>=ix1; c--) {
				for ( r=iy1; r<=iy2; r++) {
					if ( img_buf[r][c]>=thres )
						break;
				}
				if (r>=iy2)
					break;	// this is end of char in upper direction
			}
			// update character coordinate
			ix1 = c+1;
			coords[rows][chars].x1 = ix1;
			// go right
			for ( c=wid; c<=ix2; c++) {
				for ( r=iy1; r<=iy2; r++) {
					if ( img_buf[r][c]>=thres )
						break;
				}
				if (r>=iy2)
					break;	// this is end of char in upper direction
			}
			// update character coordinate
			ix2 = c-1;
			coords[rows][chars].x2 = ix2;
			wid = ix2-ix1+1;
			if (debug>1) {sprintf(str, "trimmed char: %i, x1: %i, y1: %i, x2: %i, y2: %i, height: %i, width: %i\n", chars, ix1, iy1, ix2, iy2, hei, wid); Serial.print(str);}
		}
	}
}
/*****************************************************************************/
/*****************************************************************************/
void Cam_Init()
{
	SCCB.begin();	// start I2C bus
	
	uint8_t pid, ver;
	do {	// read PID and VER
		pid = rdReg(REG_PID);
		ver = rdReg(REG_VER);
		sprintf(str, "PID: %02X, VER: %02X", pid, ver); Serial.println(str);
	} while (pid<16 || ver<16);
	
	wrReg(REG_COM7, BIT7);	// reset all internal registers to default values
	delay(100);

	//wrRegs(ov7670_default);	// load camera default settings
	// customize settings
	wrReg(REG_COM3, BIT2);	// REG_COM3 enable scaling
	wrRegs(yuv422_ov7670);	// set color
	wrRegs(qqvga_ov7670);	// scaling
/*
	// extra scaling
	wrReg(REG_SCALING_DCWCTR, 0x22); //Down sample by 4
	wrReg(REG_SCALING_PCLK_DIV, 0x02); //Clock div8
	wrReg(REG_COM14, (BIT4|0x0a) );	// PCLK divider
*/
/* optional settings */
	//wrReg(REG_ABLC1, BIT2);	// automatic black level correction
	wrReg(REG_BRIGHT, 0xb0);	// brightness -2
	wrReg(REG_CONTRAS, 0x60);	// contrast +2
	//wrReg(REG_COM8, 0xe7);	// AWB on
	//wrReg(AWBCTR0, 0x9f);	// Simple AWB
	wrReg(REG_MVFP, BIT5|BIT4);	// mirror&flip image
	//wrReg(REG_COM10, BIT5);	//pclk does not toggle on HBLANK
	//wrReg(REG_COM7,0x02);// enable color bar - for testing only !!!
	wrReg(REG_CLKRC,3);	// clock divider
}
/*****************************************************************************/
/*****************************************************************************/
void Cam_DumpRegs(uint8_t nr)
{
	Serial.print("ADDR\tVALUE\n");
	for (int i=0; i<0xC9; i++) {
		sprintf(str, "0x%02X,\t0x%02X\n", i, (rdReg(i) & 0x00FF));
		Serial.print(str);
	}
}
/*****************************************************************************/
/*****************************************************************************/
int rdReg(uint8_t regAddr)
{
	SCCB.i2c_start();
	/* check the I2C lines, optional
	if ( digitalRead(SCL_PIN)==1 || digitalRead(SDA_PIN)==1 ) {
		Serial.print("Wire read start error! System halted!");
		while(1);
	} */
	SCCB.i2c_shift_out( CAM_ID_WR );
	if (!SCCB.i2c_get_ack()) 
	{
		Serial.println("-NACK10-");	// debug
		SCCB.i2c_stop();
		return -1;
	}
	SCCB.i2c_shift_out( regAddr );
	if (!SCCB.i2c_get_ack()) 
	{
		Serial.println("-NACK11-");	// debug
		SCCB.i2c_stop();
		return -1;
	}
	SCCB.i2c_stop();
	// restart for read access
	SCCB.i2c_start();
	SCCB.i2c_shift_out( CAM_ID_RD );
	if (!SCCB.i2c_get_ack()) 
	{
		Serial.println("-NACK12-");	// debug
		SCCB.i2c_stop();
		return -1;
	}
	// read here
	uint8_t dat = SCCB.i2c_shift_in();
	SCCB.i2c_send_nack();
	SCCB.i2c_stop();
	return dat;
}
/*****************************************************************************/
/*****************************************************************************/
int wrReg(uint8_t regAddr, uint8_t regVal)
{
	//sprintf(str, "addr: %02X, val: %02X", regAddr, regVal); Serial.println(str);
	SCCB.i2c_start();
	/* check the I2C lines, optional
	if ( digitalRead(SCL_PIN)==1 || digitalRead(SDA_PIN)==1 ) {
		Serial.println("Wire write start error! System halted!");
		while(1);
	} */
	SCCB.i2c_shift_out( CAM_ID_WR );
	if (!SCCB.i2c_get_ack()) {
		Serial.println("-NACK00-");	// debug
		SCCB.i2c_stop();
		return -1;
	}
	SCCB.i2c_shift_out( regAddr );
	if (!SCCB.i2c_get_ack()) {
		Serial.println("-NACK01-");	// debug
		SCCB.i2c_stop();
		return -1;
	}
	SCCB.i2c_shift_out(regVal);
	if (!SCCB.i2c_get_ack()) {
		Serial.println("-NACK02-");	// debug
		SCCB.i2c_stop();
		return -1;
	}
	SCCB.i2c_stop();
	return 0;
}
/*****************************************************************************/
/*****************************************************************************/
void wrRegs(const struct regval_list reglist[])
{
uint8_t reg_addr, reg_val;
const struct regval_list *next = reglist;
	while (1) {
		reg_addr = next->reg_num;
		reg_val = next->value;
		if ( (reg_addr == 0xff) & (reg_val == 0xff) ) break;
		wrReg(reg_addr, reg_val);
		next++;
	}
}
