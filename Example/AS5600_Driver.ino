/*
    Name		:	AS5600_Driver.ino
    Created		:	12-Feb-2020 21:56:00
    Author		:	Oscar goos
	Code		:	C++, Flash: , RAM: 
	Contrcoller	:	ATmega328
	Last tested	:	
	Notes		:   AS5600_reg.ml=1 when .magnitude <=~1800, .md=1 when magnitude>=480, .mh=1 when could not bedetected   
				:	Arduno is little endian. LSB is at lowest address
				:	If you program MANG MPOS will be 0, if you program MPOS MAN will be zero 
				:	Burn cmd 0x10, 0x20 are software cmd's to copy NVM to 5600 angle registers;  BURN(cmd=0x40) and BURN(cmd=0x80) are hardware commands 
				:	BURN angle   (ZPOS, MPOS)   cmd=0x80 can be executes max 3x, ZMCO is max 0x03;
				:	BURN setting (MANG, CONFIG) cmd=0x40 can be executed max 1x,  when ZMCO=0 (BURN angle never executed).   
				:	BURN CMD=0x10, 0x40, 0x80 tested but not with real I2C writes to the AS5600,  cmd=0x20 is tested and it copies the NVM values to AS5600 registers
*/
#include <AS5600.h>
enum cmd  {ZMCO=0, ZPOS=1, MPOS=3, MANG=5, CONF=7, RANG=0x0C, ANG=0x0E, STAT=0x0B, AGC=0x1A, MAGN=0x1B, BURN=0xFF, ALL=2, INIT=4};

struct AS5600_regs {
	uint8_t			init_flag;									// Flag to check f AS5600 is initialised, Call 
	uint8_t			zmco;										// Prog Counter that shows nr of time MPOS and ZPOS has been written prog cmd=0x40, max value 0x02 (3x)
	uint16_t		zpos;										// Zero postion angle 
	uint16_t		mpos;										// Max  postion angle 
	uint16_t		mang;										// Max Angle range 18...360 degree.
	struct conf {
		uint8_t		pm		: 2;								// Power Mode : 00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3		
		uint8_t		hyst	: 2;								// Hysteresis : 00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs		
		uint8_t		outs	: 2;								// Ouput Stage Mode	: 00 = analog (range 0%..100%),  01 = analog range 10%..90%, (Range GND..VDD),  10 = digital PWM  	
		uint8_t		pwmf	: 2;								// PWM Frequency: 	00 = 115 Hz; 01 = 230 Hz; 10 = 460 Hz; 11 = 920 Hz
		uint8_t		sf		: 2;								// Slow Filter	: 00 = 16x (1); 01 = 8x; 10 = 4x; 11 = 2x
		uint8_t		fth		: 3;								// Fast Filter trhreshold : 000 = slow filter only, 001 = 6 LSBs, 010 = 7 LSBs, 011 = 9 LSBs,100 = 18 LSBs, 101 = 21 LSBs, 110 = 24 LSBs, 111 = 10 LSBs
		uint8_t		wd		: 1;								// Watch Dog	:	0 = OFF, 1 = ON
		uint8_t		dummy0	: 2;								// dummy bits	
	}conf;
	uint16_t		raw_angle;									// Unscale un modified angle value
	uint16_t		angle;										// Scale modified angle value
	struct stat {
		uint8_t		dummy1	: 3;								// dummy bits	
		uint8_t		mh		: 1;								// Magnetic field to high Flag	
		uint8_t		ml		: 1;								// Magnetic field to low Flag		
		uint8_t		md		: 1;								// Magnetic field available flag
		uint8_t		dummy2	: 2;								// dummy bits		
	} stat;
	uint8_t			agc;										// Value of the AGC feeback amplifier
	uint16_t		magnitude;									// Magnetic field value
	uint8_t			burn;										// Command register to burn settings cmd=0x40 or burn angle values cmd=0x80
																// MANG can only be programmed when ZMCO=0x00, max time programable 1x
} __attribute__((packed, aligned(1))) AS5600_reg = { false,  0,0x00A,0x0FEC,0x0000, {0,2,0,2, 0,0x01,0,0}, 0,0, {0,0,0,0,0} ,0,0,0 };				


uint16_t AS5600_Driver(uint8_t cmd, uint8_t rw, uint16_t par) {
	uint16_t reg;
	if (AS5600_reg.init_flag == false) cmd = INIT;				// Call AS5600_Driver(ALL, 'r', 0); first time and it auto initialises the AR5600
	switch (cmd) {
		case INIT: {
			AS5600_reg.init_flag = true;
			AS5600_Driver(CONF, 'w', *(uint16_t*)&AS5600_reg.conf);
			if ((AS5600_reg.mang == 0x000) && ((AS5600_reg.zpos != 0x000) || (AS5600_reg.mpos != 0x000))) {
				AS5600_Driver(ZPOS, 'w', AS5600_reg.zpos);
				AS5600_Driver(MPOS, 'w', AS5600_reg.mpos);
			}
			else AS5600_Driver(MANG, 'w', AS5600_reg.mang);
			AS5600_Driver(ALL,  'r', 0);
		} break;
		case ALL: { } 
		case ZMCO: {
			if( rw=='r' ) AS5600_reg.zmco				 = (uint8_t)I2C_Read(ZMCO, sizeof(AS5600_reg.zmco)) & 0x03;
		} if (cmd!=ALL) break;
		case ZPOS: {
			if( rw=='r' ) AS5600_reg.zpos				 = I2C_Read(ZPOS, sizeof(AS5600_reg.zpos)) & 0x0FFF;
			if (rw == 'w') I2C_Write(ZPOS, sizeof(AS5600_reg.zpos), (I2C_Read(ZPOS, sizeof(AS5600_reg.zpos)) & 0xF000) | AS5600_reg.zpos);		
			delay(1);
		}  if (cmd!=ALL) break;
		case MPOS: {
			if( rw == 'r' ) AS5600_reg.mpos				 = I2C_Read(MPOS, sizeof(AS5600_reg.mpos)) & 0x0FFF;
			if (rw == 'w') I2C_Write( MPOS, sizeof(AS5600_reg.mpos), (I2C_Read(ZPOS, sizeof(AS5600_reg.mpos)) & 0xF000) | AS5600_reg.mpos );
			delay(1);
		}  if (cmd!=ALL) break;
		case MANG: {
			if( rw == 'r') AS5600_reg.mang				 = I2C_Read(MANG, sizeof(AS5600_reg.mang)) & 0x0FFF;
			if (rw == 'w') I2C_Write( MANG, sizeof(AS5600_reg.mang), (I2C_Read(ZPOS, sizeof(AS5600_reg.mang)) & 0xF000) | AS5600_reg.mang);
			delay(1);
		}  if (cmd!=ALL) break;
		case CONF: {
			if( rw == 'r') *(uint16_t*)&AS5600_reg.conf	 = I2C_Read(CONF, sizeof(AS5600_reg.conf)) & 0x3FFF;
			if (rw == 'w')  I2C_Write( CONF, sizeof(AS5600_reg.conf), (I2C_Read(ZPOS, sizeof(AS5600_reg.conf)) & 0xC000) | *(uint16_t*)&AS5600_reg.conf);
		}  if (cmd!=ALL) break;

		case RANG: {
			if( rw=='r' ) AS5600_reg.raw_angle			 = I2C_Read(RANG, sizeof(AS5600_reg.raw_angle)) & 0x0FFF;
		}  if (cmd!=ALL) break;
		case ANG: {
			if( rw=='r' ) AS5600_reg.angle				 = I2C_Read(ANG, sizeof(AS5600_reg.angle)) & 0x0FFF;
		}  if (cmd!=ALL) break;

		case STAT: { 
			if (rw == 'r')* (uint8_t*)& AS5600_reg.stat = (uint8_t)I2C_Read(STAT, sizeof(AS5600_reg.stat)) & 0x38;
		} if (cmd!=ALL) break;
		case AGC: {
			if( rw=='r' ) AS5600_reg.agc				 = (uint8_t)I2C_Read(AGC, sizeof(AS5600_reg.agc)) & 0xFF;
		}  if (cmd!=ALL) break;
		case MAGN: {
			if( rw=='r' ) AS5600_reg.magnitude			 = I2C_Read(MAGN, sizeof(AS5600_reg.magnitude)) & 0x0FFF;
		}  break;
		case BURN: {											// Burn Commands  prog Setting=Ox40, Prog Angles=0X80,   load ZPOS=0x01, Load MPOS=0x11, Load MANG=0x10 
			if ((rw == 'w') && (AS5600_reg.zmco == 0x00) && (par == 0x40 || par == 0x10))	Serial.print("I2C_Write(BURN, 1, 0x40)\r"); // I2C_Write(BURN, 1, 0x40);
			if ((rw == 'w') && (AS5600_reg.zmco  < 0x03) && (par == 0x80 || par == 0x10 ) && (AS5600_reg.stat.md == 0x01) ) Serial.print("I2C_Write(BURN, 1, 0x80)\r"); //I2C_Write(BURN, 1, 0x80);
				delay(1);
			if ( (rw == 'r') && (par == 0x20) ) {
				I2C_Write(BURN, 1, 0x01);						// Copy NVM ZPOS value to AS6500 register ZPOS
				I2C_Write(BURN, 1, 0x10);						// Copy NVM MPOS value to AS6500 register MPOS
				I2C_Write(BURN, 1, 0x11);						// Copy NVM MANG value to AS6500 register MPOS
			}
		} break;
	}
}

#define			STOP			true
#define			AS5600_ADR		0x36

uint16_t I2C_Read(uint8_t regptr, uint8_t cnt) {
	Wire.beginTransmission(AS5600_ADR);
	Wire.write(regptr);
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)AS5600_ADR, cnt, (uint8_t)STOP);
	if (cnt==1) return (uint16_t) Wire.read();
	if (cnt==2) return (Wire.read() << 8 | Wire.read()) ;
}

uint16_t I2C_Write(uint8_t regptr, uint8_t cnt, uint16_t par) {
	Wire.beginTransmission(AS5600_ADR);
	Wire.write(regptr);
	if (cnt == 2) { Wire.write(par >> 8); Wire.write(par); }
	if (cnt == 1) { Wire.write(par); }
	Wire.endTransmission();
}



AS5600 encoder;
double output;
uint8_t lcnt=0;

void setup() {
  Serial.begin(115200);
  // Show the inital setting values
  Serial.print(F("ZMCO\t\t"));   Serial.print(F("ZPOS\t\t"));  Serial.print(F("MPOS\t\t"));   Serial.print(F("MANG\t\t")); Serial.print(F("CONF\t\t")); Serial.print("\r"); 
  Serial.print(AS5600_reg.zmco);  Serial.print(F("\t\t"));Serial.print(AS5600_reg.zpos,HEX);  Serial.print(F("\t\t"));Serial.print(AS5600_reg.mpos,HEX);  Serial.print(F("\t\t"));
  Serial.print(AS5600_reg.mang,HEX);  Serial.print(F("\t\t"));Serial.print(*(uint16_t*)&AS5600_reg.conf,HEX);  Serial.print("\r"); 
  // Init AS6500 and Show the read back inital setting values
  AS5600_Driver(ALL, 'r', 0);
  Serial.print(AS5600_reg.zmco);  Serial.print(F("\t\t"));Serial.print(AS5600_reg.zpos,HEX);  Serial.print(F("\t\t"));Serial.print(AS5600_reg.mpos,HEX);  Serial.print(F("\t\t"));
  Serial.print(AS5600_reg.mang,HEX);  Serial.print(F("\t\t"));Serial.print(*(uint16_t*)&AS5600_reg.conf,HEX);   Serial.print("\n"); 
  // Both displayes lines of values should be the same.
 
  AS5600_Driver(BURN, 'w', 0x10);							//Run the Burn commands 0x10, 0x20, 0x40, 0x80
  AS5600_Driver(BURN, 'w', 0x40);
  AS5600_Driver(BURN, 'w', 0x80);
  //AS5600_Driver(BURN, 'r', 0x20);

  AS5600_Driver(ALL, 'r', 0);								// Show if the NvM was copie back to AS5600 regeisters
  Serial.print(AS5600_reg.zmco);  Serial.print(F("\t\t"));Serial.print(AS5600_reg.zpos,HEX);  Serial.print(F("\t\t"));Serial.print(AS5600_reg.mpos,HEX);  Serial.print(F("\t\t"));
  Serial.print(AS5600_reg.mang,HEX);  Serial.print(F("\t\t"));Serial.print(*(uint16_t*)&AS5600_reg.conf,HEX);   Serial.print("\n"); 

  delay(5000);
}

void loop() {
  AS5600_Driver(ALL, 'r', 0);
  if ((lcnt++) % 5 == 0) { 
	  Serial.print(F("Raw A\t\t"));   Serial.print(F("Angle\t\t"));  Serial.print(F("AGC\t\t"));   Serial.print(F("Magn Fld\t")); Serial.print(F("stats\t\t"));   
	  Serial.print(F("Conf\t\t"));  Serial.print(F("out")); Serial.print("\r"); 
  }
  Serial.print(AS5600_reg.raw_angle);  Serial.print(F("\t\t")); Serial.print(AS5600_reg.angle); Serial.print(F("\t\t")); Serial.print(AS5600_reg.agc, HEX);  Serial.print(F("\t\t")); 
  Serial.print(AS5600_reg.magnitude);  Serial.print(F("\t\t"));Serial.print(*(uint8_t*)& AS5600_reg.stat,HEX);  Serial.print(F("\t\t")); Serial.print(*(uint16_t*)& AS5600_reg.conf,HEX);
  Serial.print(F("\t\t")); Serial.print(analogRead(A2));
  Serial.print(F("\r"));
  delay(500);
}















































