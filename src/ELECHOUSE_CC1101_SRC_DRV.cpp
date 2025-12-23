/*
 * ELECHOUSE_CC1101.cpp - CC1101 module library
 * Copyright (c) 2010 Michael.
 *  Author: Michael, <www.elechouse.com>
 *  Version: November 12, 2010
 *
 * This library is designed to use CC1101/CC1100 module on Arduino platform.
 * CC1101/CC1100 module is an useful wireless module.Using the functions of the
 * library, you can easily send and receive data by the CC1101/CC1100 module.
 * Just have fun!
 * For the details, please refer to the datasheet of CC1100/CC1101.
 * ----------------------------------------------------------------------------------------------------------------
 * cc1101 Driver for RC Switch. Mod by Little Satan. With permission to modify and publish Wilson Shen (ELECHOUSE).
 * ----------------------------------------------------------------------------------------------------------------
 */
#include <SPI.h>
#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <Arduino.h>

/****************************************************************/
#define   WRITE_BURST       0x40            //write burst
#define   READ_SINGLE       0x80            //read single
#define   READ_BURST        0xC0            //read burst
#define   BYTES_IN_RXFIFO   0x7F            //byte number in RXfifo
#define   max_modul 6



byte modulation = 2;
byte frend0;
byte chan = 0;
int pa = 12;
byte last_pa;
byte SCK_PIN;
byte MISO_PIN;
byte MOSI_PIN;
byte SS_PIN;
byte GDO0;
byte GDO2;
byte SCK_PIN_M[max_modul];
byte MISO_PIN_M[max_modul];
byte MOSI_PIN_M[max_modul];
byte SS_PIN_M[max_modul];
byte GDO0_M[max_modul];
byte GDO2_M[max_modul];
byte gdo_set = 0;
bool bSpiPinsDeclared = 0;
eGDIO_MODES gdio_mode = LEGACY_0;
float gTargetFreq = 903.210;
byte m4RxBw = 0;
byte m4DaRa;
byte m2DCOFF;
byte m2MODFM;
byte m2MANCH;
byte m2SYNCM;
byte m1FEC;
byte m1PRE;
byte m1CHSP;
byte pc1PQT;
byte pc1CRC_AF;
byte pc1APP_ST;
byte pc1ADRCHK;
byte pc0WDATA;
byte pc0PktForm;
byte pc0CRC_EN;
byte pc0LenConf;
byte trxstate = 0;
byte cal300_348Mhz[2] = { 24, 28 };
byte cal378_464Mhz[2] = { 31, 38 };
byte cal779_899Mhz[2] = { 65, 76 };
byte cal900_928Mhz[2] = { 77, 79 };

SPIClass *mySPI = NULL;

static const double XTAL=26.0;
#define SAFETY_TIMER 10000  // how long to wait for tx done.

/****************************************************************/
uint8_t PA_TABLE[8]     { 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//                       -30  -20  -15  -10   0    5    7    10
uint8_t PA_TABLE_315[8] { 0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2, };                //300 - 348
uint8_t PA_TABLE_433[8] { 0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0, };                //387 - 464
//                        -30  -20  -15  -10  -6    0    5    7    10   12
uint8_t PA_TABLE_868[10] { 0x03, 0x17, 0x1D, 0x26, 0x37, 0x50, 0x86, 0xCD, 0xC5, 0xC0, };   //779 - 899.99
//                        -30  -20  -15  -10  -6    0    5    7    10   11
uint8_t PA_TABLE_915[10] { 0x03, 0x0E, 0x1E, 0x27, 0x38, 0x8E, 0x84, 0xCC, 0xC3, 0xC0, };   //900 - 928



template <typename T> T regMask( T &final, T newVal, uint8_t lhs, uint8_t rhs)
{
	T original = final;
	T wide = (lhs - rhs) + 1;
	T mask = 0;
	T oldVal;
	
	assert (lhs >= rhs);
	// make a bunch of ones
	for (int i = 0; i < wide; i++) 
	{	
		mask *= 2;
		mask |=1;
	}
	mask = mask << rhs;

	oldVal = (final & mask) >> rhs;
	
	final &= ~mask;
	final |= newVal << rhs;
#if 1
	Serial.printf("\n\t%d:%d mask=0x%02X\n\toldVal=%02X newVal=%02X\n\toriginal=%02X final=%02X\n",
					lhs, rhs, mask, oldVal, newVal, original, final);
#endif
	return final;
}


/****************************************************************
* FUNCTION NAME:SpiStart
* FUNCTION     :spi communication start
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setupSPIhw(void)
{
    // initialize the SPI pins
    pinMode(SCK_PIN, OUTPUT);
    pinMode(MOSI_PIN, OUTPUT);
    pinMode(MISO_PIN, INPUT);
    pinMode(SS_PIN, OUTPUT);

    digitalWrite(SS_PIN, HIGH);
    digitalWrite(SCK_PIN, HIGH);
    digitalWrite(MOSI_PIN, LOW);
    

#if defined(ARDUINO_M5STACK_CORES3)
	mySPI = new SPIClass(FSPI);
#else
	mySPI = new SPIClass(VSPI);
#endif
	mySPI->begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

}
/****************************************************************
* FUNCTION NAME: GDO_Set()
* FUNCTION     : set GDO0,GDO2 pin for serial pinmode.
* INPUT        : none
* OUTPUT       : none
****************************************************************/
void ELECHOUSE_CC1101::GDO_Set(void)
{
    pinMode(GDO0, OUTPUT);
    pinMode(GDO2, INPUT);
    Serial.printf("\n%s: GDO0 %d set to OUTPUT ... GD02 %d set to INPUT\n\n", GDO0, GDO2);
    delay(3000);
}


/****************************************************************
* FUNCTION NAME: GDO_Set()
* FUNCTION     : set GDO0 for internal transmission mode.
* INPUT        : none
* OUTPUT       : none
****************************************************************/
uint32_t GDO0_risingCtr;
uint32_t GDO0_fallingCtr;
uint32_t GDO0_timeout;
uint32_t GDO0_sempass;

SemaphoreHandle_t sem_GGO0_UP = NULL;
SemaphoreHandle_t sem_GGO0_DN = NULL;

uint32_t GDO2_risingCtr;
uint32_t GDO2_fallingCtr;
uint32_t GDO2_timeout;
uint32_t GDO2_sempass;

SemaphoreHandle_t sem_GGO2_UP = NULL;
SemaphoreHandle_t sem_GGO2_DN = NULL;

void IRAM_ATTR GDO0_ISR()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t pin = digitalRead(GDO0);
	
	pin ? GDO0_risingCtr++ : GDO0_fallingCtr++;
	
	xSemaphoreGiveFromISR( pin ? sem_GGO0_UP: sem_GGO0_DN, &xHigherPriorityTaskWoken );

	// wake up task that need it.
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void IRAM_ATTR GDO2_ISR()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t pin = digitalRead(GDO2);
	
	pin ? GDO2_risingCtr++ : GDO2_fallingCtr++;
	
	xSemaphoreGiveFromISR( pin ? sem_GGO2_UP : sem_GGO2_DN, &xHigherPriorityTaskWoken );

	// wake up task that need it.
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/****************************************************************
* FUNCTION NAME:Reset
* FUNCTION     :CC1101 reset //details refer datasheet of CC1101/CC1100//
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Reset(void)
{
    digitalWrite(SS_PIN, LOW);
    delay(1);
    digitalWrite(SS_PIN, HIGH);
    delay(1);
    digitalWrite(SS_PIN, LOW);

    mySPI->transfer(CC1101_SRES);

    digitalWrite(SS_PIN, HIGH);
}


/****************************************************************
* FUNCTION NAME:Init
* FUNCTION     :CC1101 initialization
* INPUT        :none
* OUTPUT       :none
****************************************************************/
bool ELECHOUSE_CC1101::Init(void)
{

	if (bSpiPinsDeclared)
	{
		setupSPIhw();
		
	    if (SpiReadStatus(0x31) > 0)
	    {
		    Reset();                    //CC1101 reset
		    RegConfigSettings();        //CC1101 register config
		    mySPI->endTransaction();
		    return true;
		}
		else
		{
			Serial.printf("init failed\n");
		}
	}
	else
		Serial.printf("need to call setSPIhw first\n");

	delay(2000);
	return false;
	
}

void bin (unsigned char byte) {
    for (int i = 7; i >= 0; i--) {
        // Use bitwise AND (&) and right shift (>>) to check each bit
        Serial.printf("%d", (byte >> i) & 1);
    }
    
}

void ELECHOUSE_CC1101::DumpRegs(void)
{
	int8_t regs;
	Serial.println("-----------------------------------------");
	
	for (regs = 0 ; regs < 0x30; regs++)
	{	
		uint8_t read = SpiReadReg(regs);
		Serial.printf("\t0x%02X    0x%02X  ", regs, read);
		bin(read);
		Serial.println();
	}
}
/****************************************************************
* FUNCTION NAME:SpiWriteReg
* FUNCTION     :CC1101 write data to register
* INPUT        :addr: register address; value: register value
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiWriteReg(byte addr, byte value)
{
    digitalWrite(SS_PIN, LOW);

    mySPI->transfer(addr);
    mySPI->transfer(value);

    digitalWrite(SS_PIN, HIGH);
    mySPI->endTransaction();
}


/****************************************************************
* FUNCTION NAME:SpiWriteBurstReg
* FUNCTION     :CC1101 write burst data to register
* INPUT        :addr: register address; buffer:register value array; num:number to write
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiWriteBurstReg(byte addr, byte *buffer, byte num)
{
    byte i, temp;

    temp = addr | WRITE_BURST;
    digitalWrite(SS_PIN, LOW);

    mySPI->transfer(temp);

    for (i = 0; i < num; i++)
        mySPI->transfer(buffer[i]);

    digitalWrite(SS_PIN, HIGH);
    mySPI->endTransaction();
}


/****************************************************************
* FUNCTION NAME:SpiStrobe
* FUNCTION     :CC1101 Strobe
* INPUT        :strobe: command; //refer define in CC1101.h//
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiStrobe(byte strobe)
{
    digitalWrite(SS_PIN, LOW);
	    mySPI->transfer(strobe);
    digitalWrite(SS_PIN, HIGH);

    mySPI->endTransaction();
}


/****************************************************************
* FUNCTION NAME:SpiReadReg
* FUNCTION     :CC1101 read data from register
* INPUT        :addr: register address
* OUTPUT       :register value
****************************************************************/
byte ELECHOUSE_CC1101::SpiReadReg(byte addr)
{
    byte temp, value;

    temp = addr | READ_SINGLE;
    digitalWrite(SS_PIN, LOW);

    mySPI->transfer(temp);
    value = mySPI->transfer(0);
    digitalWrite(SS_PIN, HIGH);
    mySPI->endTransaction();
    return value;
}


/****************************************************************
* FUNCTION NAME:SpiReadBurstReg
* FUNCTION     :CC1101 read burst data from register
* INPUT        :addr: register address; buffer:array to store register value; num: number to read
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SpiReadBurstReg(byte addr, byte *buffer, byte num)
{
    byte i, temp;

    temp = addr | READ_BURST;
    digitalWrite(SS_PIN, LOW);

    mySPI->transfer(temp);

    for (i = 0; i < num; i++)
        buffer[i] = mySPI->transfer(0);

    digitalWrite(SS_PIN, HIGH);
    mySPI->endTransaction();
}


/****************************************************************
* FUNCTION NAME:SpiReadStatus
* FUNCTION     :CC1101 read status register
* INPUT        :addr: register address
* OUTPUT       :status value
****************************************************************/
byte ELECHOUSE_CC1101::SpiReadStatus(byte addr)
{
    byte value, temp;

    temp = addr | READ_BURST;
    digitalWrite(SS_PIN, LOW);

    mySPI->transfer(temp);
    value = mySPI->transfer(0);
    digitalWrite(SS_PIN, HIGH);
    mySPI->endTransaction();
    return value;
}


/****************************************************************
* FUNCTION NAME:COSTUM SPI
* FUNCTION     :set costum spi pins.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::declareSpiPins(byte sck, byte miso, byte mosi, byte ss)
{
    Serial.printf("%s : pins SET IN THIS FUNCTION\n", __FUNCTION__);
	
    bSpiPinsDeclared = 1;
    SCK_PIN = sck;
    MISO_PIN = miso;
    MOSI_PIN = mosi;
    SS_PIN = ss;
}

/****************************************************************
* FUNCTION NAME:GDO0 Pin setting
* FUNCTION     :set GDO0 Pin
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setGDO0(int8_t gdPinNo)
{
	static bool bNeedInit = true;
    GDO0 = gdPinNo;
    
    pinMode(GDO0, INPUT);
    Serial.printf("\n%s: GDO0 %d set to INPUT\n\n", __FUNCTION__, GDO0);
    delay(3000);

    if (bNeedInit)
    {
    	sem_GGO0_UP = xSemaphoreCreateBinary();
    	sem_GGO0_DN = xSemaphoreCreateBinary();
    	attachInterrupt(GDO0, GDO0_ISR, TRIG_BOTH); 
    	bNeedInit = false;
    }

    if (gdPinNo < 0 )
    {
    	digitalPinToInterrupt(-GDO0);
    	bNeedInit = true;
    }
}


/****************************************************************
* FUNCTION NAME:GDO0 Pin setting
* FUNCTION     :set GDO2 Pin
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setGDO2(int8_t gdPinNo)
{
	static bool bNeedInit = true;
    GDO2 = gdPinNo;
    
    pinMode(GDO2, INPUT);
    Serial.printf("\n%s: GDO2 %d set to INPUT\n\n", __FUNCTION__, GDO2);
    delay(3000);

    if (bNeedInit)
    {
    	attachInterrupt(GDO2, GDO2_ISR, TRIG_BOTH);
    	sem_GGO2_UP = xSemaphoreCreateBinary();
    	sem_GGO2_DN = xSemaphoreCreateBinary();
    	bNeedInit = false;
    }

    if (gdPinNo < 0 )
    {
    	digitalPinToInterrupt(-GDO2);
    }
}

/****************************************************************
* FUNCTION NAME:CCMode
* FUNCTION     :Format of RX and TX data
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setCCMode(eGDIO_MODES select)
{
    gdio_mode = select;

    if (gdio_mode == LEGACY_1)
    {
    	//page 62
    	Serial.printf("%s: GDO0=det-sync tx or rx   GDO2=mdm clock in/out\n", __FUNCTION__);
        SpiWriteReg(CC1101_IOCFG2, 0x0B);  // GDO2 serial data clock
        SpiWriteReg(CC1101_IOCFG0, 0x06);  // GD00 sync word detect rx or tx

		// page 74
		
    	Serial.printf("%s: CRC=ON pktLen=inPacket rx/tx-Fifos=ON\n" , __FUNCTION__);
        SpiWriteReg(CC1101_PKTCTRL0, 0x05); 
        
        SpiWriteReg(CC1101_MDMCFG3, 0xF8);
        SpiWriteReg(CC1101_MDMCFG4, 11 + m4RxBw);
    }
    else if (gdio_mode == LEGACY_0)
    {
    	//page 62
    	Serial.printf("%s: GDO0=mdm data in/out GDO2=mdm data in/out \n", __FUNCTION__);
        SpiWriteReg(CC1101_IOCFG2, 0x0D);	// gd02 serial data out
        SpiWriteReg(CC1101_IOCFG0, 0x0D);   // gd00 serial data out

		// page 74
    	Serial.printf("%s: CRC=OFF pktLen=notinPacket GDx=data+clk\n" , __FUNCTION__);
        SpiWriteReg(CC1101_PKTCTRL0, 0x32);
        
        SpiWriteReg(CC1101_MDMCFG3, 0x93);
        SpiWriteReg(CC1101_MDMCFG4, 7 + m4RxBw);
    }
    else if (gdio_mode == DONS_MODE)
    {
    	//page 62
    	Serial.printf("%s: DON GDO0=fifo thresh  GDO2=mdm clock in/out\n", __FUNCTION__);
        SpiWriteReg(CC1101_IOCFG2, 0x0B);  // GDO2 serial data clock
        SpiWriteReg(CC1101_IOCFG0, 0x02);  // GD00 signal on tx getting low?

		// page 74
		
    	Serial.printf("%s: DON CRC=ON pktLen=inPacket rx/tx-Fifos=ON\n" , __FUNCTION__);
        SpiWriteReg(CC1101_PKTCTRL0, 0x05); 
        
        SpiWriteReg(CC1101_MDMCFG3, 0xF8);
        SpiWriteReg(CC1101_MDMCFG4, 11 + m4RxBw);
    }
    else 
    {
    	assert(gdio_mode == !gdio_mode);
    }

    setModulation(modulation);
}


/****************************************************************
* FUNCTION NAME:Modulation
* FUNCTION     :set CC1101 Modulation
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setModulation(byte m)
{
    if (m > 4)
        m = 4;

    modulation = m;
    Split_MDMCFG2();

    switch (m)
    {
    case 0: m2MODFM = 0x00; frend0 = 0x10; break;   // 2-FSK

    case 1: m2MODFM = 0x10; frend0 = 0x10; break;   // GFSK

    case 2: m2MODFM = 0x30; frend0 = 0x11; break;   // ASK

    case 3: m2MODFM = 0x40; frend0 = 0x10; break;   // 4-FSK

    case 4: m2MODFM = 0x70; frend0 = 0x10; break;   // MSK
    }

    SpiWriteReg(CC1101_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
    SpiWriteReg(CC1101_FREND0, frend0);
    setPA(pa);
}


/****************************************************************
* FUNCTION NAME:PA Power
* FUNCTION     :set CC1101 PA Power
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPA(int p)
{
    int a;

    pa = p;

    if (gTargetFreq >= 300 && gTargetFreq <= 348)
    {
        if (pa <= -30)
            a = PA_TABLE_315[0];
        else if (pa > -30 && pa <= -20)
            a = PA_TABLE_315[1];
        else if (pa > -20 && pa <= -15)
            a = PA_TABLE_315[2];
        else if (pa > -15 && pa <= -10)
            a = PA_TABLE_315[3];
        else if (pa > -10 && pa <= 0)
            a = PA_TABLE_315[4];
        else if (pa > 0 && pa <= 5)
            a = PA_TABLE_315[5];
        else if (pa > 5 && pa <= 7)
            a = PA_TABLE_315[6];
        else if (pa > 7)
            a = PA_TABLE_315[7];

        last_pa = 1;
    }
    else if (gTargetFreq >= 378 && gTargetFreq <= 464)
    {
        if (pa <= -30)
            a = PA_TABLE_433[0];
        else if (pa > -30 && pa <= -20)
            a = PA_TABLE_433[1];
        else if (pa > -20 && pa <= -15)
            a = PA_TABLE_433[2];
        else if (pa > -15 && pa <= -10)
            a = PA_TABLE_433[3];
        else if (pa > -10 && pa <= 0)
            a = PA_TABLE_433[4];
        else if (pa > 0 && pa <= 5)
            a = PA_TABLE_433[5];
        else if (pa > 5 && pa <= 7)
            a = PA_TABLE_433[6];
        else if (pa > 7)
            a = PA_TABLE_433[7];

        last_pa = 2;
    }
    else if (gTargetFreq >= 779 && gTargetFreq <= 899.99)
    {
        if (pa <= -30)
            a = PA_TABLE_868[0];
        else if (pa > -30 && pa <= -20)
            a = PA_TABLE_868[1];
        else if (pa > -20 && pa <= -15)
            a = PA_TABLE_868[2];
        else if (pa > -15 && pa <= -10)
            a = PA_TABLE_868[3];
        else if (pa > -10 && pa <= -6)
            a = PA_TABLE_868[4];
        else if (pa > -6 && pa <= 0)
            a = PA_TABLE_868[5];
        else if (pa > 0 && pa <= 5)
            a = PA_TABLE_868[6];
        else if (pa > 5 && pa <= 7)
            a = PA_TABLE_868[7];
        else if (pa > 7 && pa <= 10)
            a = PA_TABLE_868[8];
        else if (pa > 10)
            a = PA_TABLE_868[9];

        last_pa = 3;
    }
    else if (gTargetFreq >= 900 && gTargetFreq <= 928)
    {
        if (pa <= -30)
            a = PA_TABLE_915[0];
        else if (pa > -30 && pa <= -20)
            a = PA_TABLE_915[1];
        else if (pa > -20 && pa <= -15)
            a = PA_TABLE_915[2];
        else if (pa > -15 && pa <= -10)
            a = PA_TABLE_915[3];
        else if (pa > -10 && pa <= -6)
            a = PA_TABLE_915[4];
        else if (pa > -6 && pa <= 0)
            a = PA_TABLE_915[5];
        else if (pa > 0 && pa <= 5)
            a = PA_TABLE_915[6];
        else if (pa > 5 && pa <= 7)
            a = PA_TABLE_915[7];
        else if (pa > 7 && pa <= 10)
            a = PA_TABLE_915[8];
        else if (pa > 10)
            a = PA_TABLE_915[9];

        last_pa = 4;
    }

    if (modulation == 2)
    {
        PA_TABLE[0] = 0;
        PA_TABLE[1] = a;
    }
    else
    {
        PA_TABLE[0] = a;
        PA_TABLE[1] = 0;
    }

    SpiWriteBurstReg(CC1101_PATABLE, PA_TABLE, 8);
}


/****************************************************************
* FUNCTION NAME:Frequency Calculator
* FUNCTION     :Calculate the basic frequency.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setMHZ(float mhz)
{
	//pg. 75
	unsigned long test;
    byte freq2 = 0;
    byte freq1 = 0;
    byte freq0 = 0;

	Serial.printf("%s: setting freq to %f mHz\n",__FUNCTION__, mhz);
	
    gTargetFreq = mhz;

    for (bool i = 0; i == 0;)
    {
        if (mhz >= 26)
        {
            mhz -= 26;
            freq2 += 1;
        }
        else if (mhz >= 0.1015625)
        {
            mhz -= 0.1015625;
            freq1 += 1;
        }
        else if (mhz >= 0.00039675)
        {
            mhz -= 0.00039675;
            freq0 += 1;
        }
        else
        {
            i = 1;
        }
    }

    if (freq0 > 255)
    {
        freq1 += 1; freq0 -= 256;
    }

    SpiWriteReg(CC1101_FREQ2, freq2);
    SpiWriteReg(CC1101_FREQ1, freq1);
    SpiWriteReg(CC1101_FREQ0, freq0);
/*
	test = freq2 << 16 | freq1 << 8 | freq0;
	
    Serial.printf("%s: freq =%f %X reg1=%X reg2=%08X reg3=%08X\n",
    			__FUNCTION__, gTargetFreq, 
    			test,
				freq2,freq1,freq0);
	double retest;
	retest = (XTAL / (double)(1<<16)) * (double) test;
	Serial.printf("%s retest = %f mhz \n", __FUNCTION__, (float) retest);

	double err = gTargetFreq - retest;

	Serial.printf("%s error = %f\n", __FUNCTION__, err);
*/

    Calibrate(); //disabled in call, it makes things worse.
}


/****************************************************************
* FUNCTION NAME:Calibrate
* FUNCTION     :Calibrate frequency
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Calibrate(void)
{
#if 0
    if (gTargetFreq >= 300 && gTargetFreq <= 348)
    {
        SpiWriteReg(CC1101_FSCTRL0, map(gTargetFreq, 300, 348, cal300_348Mhz[0], cal300_348Mhz[1]));

        if (gTargetFreq < 322.88)
        {
            SpiWriteReg(CC1101_TEST0, 0x0B);
        }
        else
        {
            SpiWriteReg(CC1101_TEST0, 0x09);
            int s = ELECHOUSE_cc1101.SpiReadStatus(CC1101_FSCAL2);

            if (s < 32)
                SpiWriteReg(CC1101_FSCAL2, s + 32);

            if (last_pa != 1)
                setPA(pa);
        }
    }
    else if (gTargetFreq >= 378 && gTargetFreq <= 464)
    {
        SpiWriteReg(CC1101_FSCTRL0, map(gTargetFreq, 378, 464, cal378_464Mhz[0], cal378_464Mhz[1]));

        if (gTargetFreq < 430.5)
        {
            SpiWriteReg(CC1101_TEST0, 0x0B);
        }
        else
        {
            SpiWriteReg(CC1101_TEST0, 0x09);
            int s = ELECHOUSE_cc1101.SpiReadStatus(CC1101_FSCAL2);

            if (s < 32)
                SpiWriteReg(CC1101_FSCAL2, s + 32);

            if (last_pa != 2)
                setPA(pa);
        }
    }
    else if (gTargetFreq >= 779 && gTargetFreq <= 899.99)
    {
        SpiWriteReg(CC1101_FSCTRL0, map(gTargetFreq, 779, 899, cal779_899Mhz[0], cal779_899Mhz[1]));

        if (gTargetFreq < 861)
        {
            SpiWriteReg(CC1101_TEST0, 0x0B);
        }
        else
        {
            SpiWriteReg(CC1101_TEST0, 0x09);
            int s = ELECHOUSE_cc1101.SpiReadStatus(CC1101_FSCAL2);

            if (s < 32)
                SpiWriteReg(CC1101_FSCAL2, s + 32);

            if (last_pa != 3)
                setPA(pa);
        }
    }
    else if (gTargetFreq >= 900 && gTargetFreq <= 928)
    {
        SpiWriteReg(CC1101_FSCTRL0, map(gTargetFreq, 900, 928, cal900_928Mhz[0], cal900_928Mhz[1]));
        SpiWriteReg(CC1101_TEST0, 0x09);
        int s = ELECHOUSE_cc1101.SpiReadStatus(CC1101_FSCAL2);

        if (s < 32)
            SpiWriteReg(CC1101_FSCAL2, s + 32);

        if (last_pa != 4)
            setPA(pa);
    }
#else
	Serial.printf("%s: function disbled due to bad calc\n", __FUNCTION__);
#endif
}


/****************************************************************
* FUNCTION NAME:Calibration offset
* FUNCTION     :Set calibration offset
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setClb(byte b, byte s, byte e)
{
    if (b == 1)
    {
        cal300_348Mhz[0] = s;
        cal300_348Mhz[1] = e;
    }
    else if (b == 2)
    {
        cal378_464Mhz[0] = s;
        cal378_464Mhz[1] = e;
    }
    else if (b == 3)
    {
        cal779_899Mhz[0] = s;
        cal779_899Mhz[1] = e;
    }
    else if (b == 4)
    {
        cal900_928Mhz[0] = s;
        cal900_928Mhz[1] = e;
    }
}

/****************************************************************
* FUNCTION NAME:getMode
* FUNCTION     :Return the Mode. Sidle = 0, TX = 1, Rx = 2.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
byte ELECHOUSE_CC1101::getMode(void)
{
    return trxstate;
}

/****************************************************************
* FUNCTION NAME:Set Num Preamblebits
* FUNCTION     :PreambleBits
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPreambleBitLen(uint8_t in)
{
	const uint8_t mapx[] = { 2,3,4,6,8,12,16,24 };
	
	uint8_t index;
	for (index = 1; index < 8; index++)
	{
		if (in < mapx[index]) break;
	}
	index -=1;
	
	Serial.printf("%s in=%d index=%d\n", __FUNCTION__, in, index);
	
	uint8_t test = SpiReadReg(CC1101_MDMCFG1);    //SpiWriteReg(CC1101_SYNC1, sh);

	regMask (test, index, 6,4); // yes it wants the index number, not the value.
    SpiWriteReg(CC1101_MDMCFG1, test);
   

}


/****************************************************************
* FUNCTION NAME:Set Sync_Word
* FUNCTION     :Sync Word
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSyncWord(byte sh, byte sl)
{
    SpiWriteReg(CC1101_SYNC1, sh);
    SpiWriteReg(CC1101_SYNC0, sl);
}


/****************************************************************
* FUNCTION NAME:Set ADDR
* FUNCTION     :Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setAddr(byte v)
{
    SpiWriteReg(CC1101_ADDR, v);
}


/****************************************************************
* FUNCTION NAME:Set PQT
* FUNCTION     :Preamble quality estimator threshold
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPQT(byte v)
{
    Split_PKTCTRL1();
    pc1PQT = 0;

    if (v > 7)
        v = 7;

    pc1PQT = v * 32;
    SpiWriteReg(CC1101_PKTCTRL1, pc1PQT + pc1CRC_AF + pc1APP_ST + pc1ADRCHK);
}


/****************************************************************
* FUNCTION NAME:Set CRC_AUTOFLUSH
* FUNCTION     :Enable automatic flush of RX FIFO when CRC is not OK
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setCRC_AF(bool v)
{
    Split_PKTCTRL1();
    pc1CRC_AF = 0;

    if (v == 1)
        pc1CRC_AF = 8;

    SpiWriteReg(CC1101_PKTCTRL1, pc1PQT + pc1CRC_AF + pc1APP_ST + pc1ADRCHK);
}


/****************************************************************
* FUNCTION NAME:Set APPEND_STATUS
* FUNCTION     :When enabled, two status bytes will be appended to the payload of the packet
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setAppendStatus(bool v)
{
    Split_PKTCTRL1();
    pc1APP_ST = 0;

    if (v == 1)
        pc1APP_ST = 4;

    SpiWriteReg(CC1101_PKTCTRL1, pc1PQT + pc1CRC_AF + pc1APP_ST + pc1ADRCHK);
}


/****************************************************************
* FUNCTION NAME:Set ADR_CHK
* FUNCTION     :Controls address check configuration of received packages
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setAdrChk(byte v)
{
    Split_PKTCTRL1();
    pc1ADRCHK = 0;

    if (v > 3)
        v = 3;

    pc1ADRCHK = v;
    SpiWriteReg(CC1101_PKTCTRL1, pc1PQT + pc1CRC_AF + pc1APP_ST + pc1ADRCHK);
}


/****************************************************************
* FUNCTION NAME:Set WHITE_DATA
* FUNCTION     :Turn data whitening on / off.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setWhiteData(bool v)
{
    Split_PKTCTRL0();
    pc0WDATA = 0;

    if (v == 1)
        pc0WDATA = 64;

    SpiWriteReg(CC1101_PKTCTRL0, pc0WDATA + pc0PktForm + pc0CRC_EN + pc0LenConf);
}


/****************************************************************
* FUNCTION NAME:Set PKT_FORMAT
* FUNCTION     :Format of RX and TX data
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPktFormat(byte v)
{
    Split_PKTCTRL0();
    pc0PktForm = 0;

    if (v > 3)
        v = 3;

    pc0PktForm = v * 16;
    SpiWriteReg(CC1101_PKTCTRL0, pc0WDATA + pc0PktForm + pc0CRC_EN + pc0LenConf);
}


/****************************************************************
* FUNCTION NAME:Set CRC
* FUNCTION     :CRC calculation in TX and CRC check in RX
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setCrc(bool v)
{
    Split_PKTCTRL0();
    pc0CRC_EN = 0;

    if (v == 1)
        pc0CRC_EN = 4;

    SpiWriteReg(CC1101_PKTCTRL0, pc0WDATA + pc0PktForm + pc0CRC_EN + pc0LenConf);
}


/****************************************************************
* FUNCTION NAME:Set LENGTH_CONFIG
* FUNCTION     :Configure the packet length
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setLengthConfig(byte v)
{
    Split_PKTCTRL0();
    pc0LenConf = 0;

    if (v > 3)
        v = 3;

    pc0LenConf = v;
    SpiWriteReg(CC1101_PKTCTRL0, pc0WDATA + pc0PktForm + pc0CRC_EN + pc0LenConf);
}


/****************************************************************
* FUNCTION NAME:Set PACKET_LENGTH
* FUNCTION     :Indicates the packet length
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPacketLength(byte v)
{
    SpiWriteReg(CC1101_PKTLEN, v);
}

/****************************************************************
* FUNCTION NAME:Set fifo trigger level
* FUNCTION     :bytes before tx underflow or rx overflow 
* INPUT        :none
* OUTPUT       :none
		  TX   RX
0 (0000)  61    4
1 (0001)  57    8
2 (0010)  53   12
3 (0011)  49   16
4 (0100)  45   20
5 (0101)  41   24
6 (0110)  37   28
8 (1000)  29   36
9 (1001)  25   40
10 (1010) 21   44
11 (1011) 17   48
12 (1100) 13   52
13 (1101) 9    56
14 (1110) 5    60
15 (1111) 1    64
*/
const uint8_t tx_lvl[] = {61,57,53,49,45,41,37,33,29,25,21,17,13, 9, 5, 1};

const uint8_t rx_lvl[] = { 4, 8,12,16,20,24,28,32,36,40,44,48,52,56,60,64};


/****************************************************************/
void ELECHOUSE_CC1101::setTxFifoThreshold(uint8_t v)
{
	int i;
	int test;
	for (i = 0; i < sizeof(tx_lvl); i++) 
	{
		test = v - tx_lvl[i];
		//Serial.printf("%d  %d > %d x %d\n", i, v, tx_lvl[i], test);
		if ( v > tx_lvl[i] ) break;
	}
	i = i - 1;
	
	Serial.printf("%s : tx fifo warn wants %d gets %d {%d}\n", __FUNCTION__, v, tx_lvl[i], i);
	delay(1000);
    SpiWriteReg(CC1101_FIFOTHR, i);
    SpiWriteReg(CC1101_IOCFG0, 2);  // GD00 signal on tx low
}


/****************************************************************
* FUNCTION NAME:Set DCFILT_OFF
* FUNCTION     :Disable digital DC blocking filter before demodulator
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setDcFilterOff(bool v)
{
    Split_MDMCFG2();
    m2DCOFF = 0;

    if (v == 1)
        m2DCOFF = 128;

    SpiWriteReg(CC1101_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
}


/****************************************************************
* FUNCTION NAME:Set MANCHESTER
* FUNCTION     :Enables Manchester encoding/decoding
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setManchester(bool v)
{
    Split_MDMCFG2();
    m2MANCH = 0;

    if (v == 1)
        m2MANCH = 8;

    SpiWriteReg(CC1101_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
}


/****************************************************************
* FUNCTION NAME:Set SYNC_MODE
* FUNCTION     :Combined sync-word qualifier mode
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSyncMode(byte v)
{
    Split_MDMCFG2();
    m2SYNCM = 0;

    if (v > 7)
        v = 7;

    m2SYNCM = v;
    SpiWriteReg(CC1101_MDMCFG2, m2DCOFF + m2MODFM + m2MANCH + m2SYNCM);
}


/****************************************************************
* FUNCTION NAME:Set FEC
* FUNCTION     :Enable Forward Error Correction (FEC)
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setFEC(bool v)
{
    Split_MDMCFG1();
    m1FEC = 0;

    if (v == 1)
        m1FEC = 128;

    SpiWriteReg(CC1101_MDMCFG1, m1FEC + m1PRE + m1CHSP);
}


/****************************************************************
* FUNCTION NAME:Set PRE
* FUNCTION     :Sets the minimum number of preamble bytes to be transmitted.
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setPRE(byte v)
{
    Split_MDMCFG1();
    m1PRE = 0;

    if (v > 7)
        v = 7;

    m1PRE = v * 16;
    SpiWriteReg(CC1101_MDMCFG1, m1FEC + m1PRE + m1CHSP);
}


/****************************************************************
* FUNCTION NAME:Set Channel
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setChannel(byte ch)
{
    chan = ch;
    SpiWriteReg(CC1101_CHANNR, chan);
}


/****************************************************************
* FUNCTION NAME:Set Channel spacing
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setChsp(float f)
{
    Split_MDMCFG1();
    byte MDMCFG0 = 0;
    m1CHSP = 0;

    if (f > 405.456543)
        f = 405.456543;

    if (f < 25.390625)
        f = 25.390625;

    for (int i = 0; i < 5; i++)
    {
        if (f <= 50.682068)
        {
            f -= 25.390625;
            f /= 0.0991825;
            MDMCFG0 = f;
            float s1 = (f - MDMCFG0) * 10;

            if (s1 >= 5)
                MDMCFG0++;

            i = 5;
        }
        else
        {
            m1CHSP++;
            f /= 2;
        }
    }

    SpiWriteReg(19, m1CHSP + m1FEC + m1PRE);
    SpiWriteReg(20, MDMCFG0);
}


/****************************************************************
* FUNCTION NAME:Set Receive bandwidth
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setRxBW(float f)
{
    Split_MDMCFG4();
    int s1 = 3;
    int s2 = 3;

    for (int i = 0; i < 3; i++)
    {
        if (f > 101.5625)
        {
            f /= 2; s1--;
        }
        else
        {
            i = 3;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (f > 58.1)
        {
            f /= 1.25; s2--;
        }
        else
        {
            i = 3;
        }
    }

    s1 *= 64;
    s2 *= 16;
    m4RxBw = s1 + s2;
    SpiWriteReg(16, m4RxBw + m4DaRa);
}


/****************************************************************
* FUNCTION NAME:Set Data Rate
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setDRate(float d)
{
    Split_MDMCFG4();
    float c = d;
    byte MDMCFG3 = 0;

    if (c > 1621.83)
        c = 1621.83;

    if (c < 0.0247955)
        c = 0.0247955;

    m4DaRa = 0;

    for (int i = 0; i < 20; i++)
    {
        if (c <= 0.0494942)
        {
            c = c - 0.0247955;
            c = c / 0.00009685;
            MDMCFG3 = c;
            float s1 = (c - MDMCFG3) * 10;

            if (s1 >= 5)
                MDMCFG3++;

            i = 20;
        }
        else
        {
            m4DaRa++;
            c = c / 2;
        }
    }

    SpiWriteReg(16, m4RxBw + m4DaRa);
    SpiWriteReg(17, MDMCFG3);
}


/****************************************************************
* FUNCTION NAME:Set Devitation
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setDeviation(float d)
{
    float f = 1.586914;
    float v = 0.19836425;
    int c = 0;

    if (d > 380.859375)
        d = 380.859375;

    if (d < 1.586914)
        d = 1.586914;

    for (int i = 0; i < 255; i++)
    {
        f += v;

        if (c == 7)
        {
            v *= 2; c = -1; i += 8;
        }

        if (f >= d)
        {
            c = i; i = 255;
        }

        c++;
    }

    SpiWriteReg(21, c);
}


/****************************************************************
* FUNCTION NAME:Split PKTCTRL0
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Split_PKTCTRL1(void)
{
    int calc = SpiReadStatus(7);

    pc1PQT = 0;
    pc1CRC_AF = 0;
    pc1APP_ST = 0;
    pc1ADRCHK = 0;

    for (bool i = 0; i == 0;)
    {
        if (calc >= 32)
        {
            calc -= 32; pc1PQT += 32;
        }
        else if (calc >= 8)
        {
            calc -= 8; pc1CRC_AF += 8;
        }
        else if (calc >= 4)
        {
            calc -= 4; pc1APP_ST += 4;
        }
        else
        {
            pc1ADRCHK = calc; i = 1;
        }
    }
}


/****************************************************************
* FUNCTION NAME:Split PKTCTRL0
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Split_PKTCTRL0(void)
{
    int calc = SpiReadStatus(8);

    pc0WDATA = 0;
    pc0PktForm = 0;
    pc0CRC_EN = 0;
    pc0LenConf = 0;

    for (bool i = 0; i == 0;)
    {
        if (calc >= 64)
        {
            calc -= 64; pc0WDATA += 64;
        }
        else if (calc >= 16)
        {
            calc -= 16; pc0PktForm += 16;
        }
        else if (calc >= 4)
        {
            calc -= 4; pc0CRC_EN += 4;
        }
        else
        {
            pc0LenConf = calc; i = 1;
        }
    }
}


/****************************************************************
* FUNCTION NAME:Split MDMCFG1
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Split_MDMCFG1(void)
{
    int calc = SpiReadStatus(19);

    m1FEC = 0;
    m1PRE = 0;
    m1CHSP = 0;
    int s2 = 0;

    for (bool i = 0; i == 0;)
    {
        if (calc >= 128)
        {
            calc -= 128; m1FEC += 128;
        }
        else if (calc >= 16)
        {
            calc -= 16; m1PRE += 16;
        }
        else
        {
            m1CHSP = calc; i = 1;
        }
    }
}


/****************************************************************
* FUNCTION NAME:Split MDMCFG2
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Split_MDMCFG2(void)
{
    int calc = SpiReadStatus(18);

    m2DCOFF = 0;
    m2MODFM = 0;
    m2MANCH = 0;
    m2SYNCM = 0;

    for (bool i = 0; i == 0;)
    {
        if (calc >= 128)
        {
            calc -= 128; m2DCOFF += 128;
        }
        else if (calc >= 16)
        {
            calc -= 16; m2MODFM += 16;
        }
        else if (calc >= 8)
        {
            calc -= 8; m2MANCH += 8;
        }
        else
        {
            m2SYNCM = calc; i = 1;
        }
    }
}


/****************************************************************
* FUNCTION NAME:Split MDMCFG4
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::Split_MDMCFG4(void)
{
    int calc = SpiReadStatus(16);

    m4RxBw = 0;
    m4DaRa = 0;

    for (bool i = 0; i == 0;)
    {
        if (calc >= 64)
        {
            calc -= 64; m4RxBw += 64;
        }
        else if (calc >= 16)
        {
            calc -= 16; m4RxBw += 16;
        }
        else
        {
            m4DaRa = calc; i = 1;
        }
    }
}


/****************************************************************
* FUNCTION NAME:RegConfigSettings
* FUNCTION     :CC1101 register config //details refer datasheet of CC1101/CC1100//
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::RegConfigSettings(void)
{
    SpiWriteReg(CC1101_FSCTRL1, 0x06);

    setCCMode(gdio_mode);
    setMHZ(gTargetFreq);

    SpiWriteReg(CC1101_MDMCFG1, 0x02);
    SpiWriteReg(CC1101_MDMCFG0, 0xF8);
    SpiWriteReg(CC1101_CHANNR, chan);
    SpiWriteReg(CC1101_DEVIATN, 0x47); 	// pg. 42 tweak freq dev fsk/c4.
    SpiWriteReg(CC1101_FREND1, 0x56);
    SpiWriteReg(CC1101_MCSM0, 0x18);   //
    SpiWriteReg(CC1101_FOCCFG, 0x16); // fine freq compensation
    SpiWriteReg(CC1101_BSCFG, 0x1C);	// clock recovery params on sync det	
    SpiWriteReg(CC1101_AGCCTRL2, 0xC7);	// pg. 85 highest gain limit = 33db 
    SpiWriteReg(CC1101_AGCCTRL1, 0x00); // pg. 86 carrier sense setting
    SpiWriteReg(CC1101_AGCCTRL0, 0xB2); // pg. 87 agc settings ampl, ticks 
    SpiWriteReg(CC1101_FSCAL3, 0xE9);	// pg. 89 freq call ops
    SpiWriteReg(CC1101_FSCAL2, 0x2A);	// cal
    SpiWriteReg(CC1101_FSCAL1, 0x00);	// cal
    SpiWriteReg(CC1101_FSCAL0, 0x1F);	// cal
    SpiWriteReg(CC1101_FSTEST, 0x59);	// factory always set to 0x59	
    SpiWriteReg(CC1101_TEST2, 0x81);	// sleep control
    SpiWriteReg(CC1101_TEST1, 0x35);	// wake up control
    SpiWriteReg(CC1101_TEST0, 0x09);	// who knows.
    SpiWriteReg(CC1101_PKTCTRL1, 0x04);	
    SpiWriteReg(CC1101_ADDR, 0x00);		// device address
    SpiWriteReg(CC1101_PKTLEN, 0x00);	// packet style, fixed, inpacket, infite
}

#define NOTE(x) Serial.println(x);
/****************************************************************
* FUNCTION NAME:SetTx
* FUNCTION     :set CC1101 send data
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SetTx(void)
{
	Serial.printf("************* Enter tx mode \n");
    SpiStrobe(CC1101_SIDLE);
    SpiStrobe(CC1101_STX);      //start send
    trxstate = 1;
}


/****************************************************************
* FUNCTION NAME:SetRx
* FUNCTION     :set CC1101 to receive state
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SetRx(void)
{
	Serial.printf("************** EnterRxMode ****\n");
    SpiStrobe(CC1101_SIDLE);
    SpiStrobe(CC1101_SRX);      //start receive
    trxstate = 2;
}


/****************************************************************
* FUNCTION NAME:SetTx
* FUNCTION     :set CC1101 send data and change frequency
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SetTx(float mhz)
{
	Serial.printf("************* Enter tx mode with freq = %f\n", mhz);
	
    SpiStrobe(CC1101_SIDLE);
    setMHZ(mhz);
    SpiStrobe(CC1101_STX);      //start send
    trxstate = 1;
}


/****************************************************************
* FUNCTION NAME:SetRx
* FUNCTION     :set CC1101 to receive state and change frequency
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SetRx(float mhz)
{
	Serial.printf("************* EnterRxMode + FREQ = %f ****\n", mhz);
	NOTE("hop and RX");
    SpiStrobe(CC1101_SIDLE);
    setMHZ(mhz);
    SpiStrobe(CC1101_SRX);      //start receive
    trxstate = 2;
}


/****************************************************************
* FUNCTION NAME:RSSI Level
* FUNCTION     :Calculating the RSSI Level
* INPUT        :none
* OUTPUT       :none
****************************************************************/
int ELECHOUSE_CC1101::getRssi(void)
{
    int rssi;

    rssi = SpiReadStatus(CC1101_RSSI);

    if (rssi >= 128)
        rssi = (rssi - 256) / 2 - 74;
    else
        rssi = (rssi / 2) - 74;

    return rssi;
}


/****************************************************************
* FUNCTION NAME:LQI Level
* FUNCTION     :get Lqi state
* INPUT        :none
* OUTPUT       :none
****************************************************************/
byte ELECHOUSE_CC1101::getLqi(void)
{
    byte lqi;

    lqi = SpiReadStatus(CC1101_LQI);
    return lqi;
}


/****************************************************************
* FUNCTION NAME:SetSres
* FUNCTION     :Reset CC1101
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSres(void)
{
    SpiStrobe(CC1101_SRES);
    trxstate = 0;
}


/****************************************************************
* FUNCTION NAME:setSidle
* FUNCTION     :set Rx / TX Off
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::setSidle(void)
{
    SpiStrobe(CC1101_SIDLE);
    trxstate = 0;
}


/****************************************************************
* FUNCTION NAME:goSleep
* FUNCTION     :set cc1101 Sleep on
* INPUT        :none
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::goSleep(void)
{
    trxstate = 0;
    SpiStrobe(0x36);    //Exit RX / TX, turn off frequency synthesizer and exit
    SpiStrobe(0x39);    //Enter power down mode when CSn goes high.
}


/****************************************************************
* FUNCTION NAME:Char direct SendData
* FUNCTION     :use CC1101 send data
* INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SendData(char *txchar)
{
    int len = strlen(txchar);
    byte chartobyte[len];

    for (int i = 0; i < len; i++)
        chartobyte[i] = txchar[i];

    SendData(chartobyte, len);
}

#include <string>
#include <cstring> 


void ELECHOUSE_CC1101::SendData(String &txchar)
{
    int len = txchar.length();
    char chartobyte[len+1];

	strcpy (chartobyte, txchar.c_str());

    SendData((byte*)chartobyte, len);
}


/****************************************************************
* FUNCTION NAME:SendData
* FUNCTION     :use CC1101 send data
* INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SendData(byte *txBuffer, byte size)
{
	uint32_t ctr;
	uint8_t count;
	
    SpiWriteReg(CC1101_TXFIFO, size);
    SpiWriteBurstReg(CC1101_TXFIFO, txBuffer, size);    //write data to send
    
    SpiStrobe(CC1101_SIDLE);
	uint32_t then = millis();
    SpiStrobe(CC1101_STX);                              //start send

    int ret = xSemaphoreTake( sem_GGO0_UP, pdMS_TO_TICKS(SAFETY_TIMER) );
	ret == pdTRUE ? GDO0_sempass++ : GDO0_timeout++;
	
	if (ret != pdTRUE) 
	{	
		Serial.printf("%s timeout on send! ret=%d \n", __FUNCTION__, ret);
	}
	else
	{
		// GOOD tx. last 4 bytes to go as GDO0 is set to alert on byte 4.
		// docs recommend wait until last byte is out.
		while(true)
		{
			count = SpiReadStatus(CC1101_TXBYTES);
			Serial.printf("bytes left in TxQ = %d\n", count);
			if (!count) break;
			delay(1);
		}
		//Serial.printf("bytes left in Q = %d\n", count);
	}

    SpiStrobe(CC1101_SFTX); //should be zero but anyhow ... flush TXfifo

    trxstate = 1;
}


/****************************************************************
* FUNCTION NAME:Char direct SendData
* FUNCTION     :use CC1101 send data without GDO
* INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SendData(char *txchar, int t)
{
    int len = strlen(txchar);
    byte chartobyte[len];

    for (int i = 0; i < len; i++)
        chartobyte[i] = txchar[i];

    SendData(chartobyte, len, t);
}


/****************************************************************
* FUNCTION NAME:SendData
* FUNCTION     :use CC1101 send data without GDO
* INPUT        :txBuffer: data array to send; size: number of data to send, no more than 61
* OUTPUT       :none
****************************************************************/
void ELECHOUSE_CC1101::SendData(byte *txBuffer, byte size, int t)
{
    SpiWriteReg(CC1101_TXFIFO, size);
    SpiWriteBurstReg(CC1101_TXFIFO, txBuffer, size);    //write data to send
    SpiStrobe(CC1101_SIDLE);
    SpiStrobe(CC1101_STX);                              //start send
    delay(t);
    SpiStrobe(CC1101_SFTX);                             //flush TXfifo
    trxstate = 1;
}


/****************************************************************
* FUNCTION NAME:Check CRC
* FUNCTION     :none
* INPUT        :none
* OUTPUT       :none
****************************************************************/
bool ELECHOUSE_CC1101::CheckCRC(void)
{
    byte lqi = SpiReadStatus(CC1101_LQI);
    bool crc_ok = bitRead(lqi, 7);

    if (crc_ok == 1)
    {
        return 1;
    }
    else
    {
        SpiStrobe(CC1101_SFRX);
        SpiStrobe(CC1101_SRX);
        return 0;
    }
}


/****************************************************************
* FUNCTION NAME:CheckRxFifo
* FUNCTION     :check receive data or not
* INPUT        :none
* OUTPUT       :flag: 0 no data; 1 receive data
****************************************************************/
bool ELECHOUSE_CC1101::CheckRxFifo(int t)
{
    if (trxstate != 2)
        SetRx();

    if (SpiReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO)
    {
        delay(t);
        return 1;
    }
    else
    {
        return 0;
    }
}


/****************************************************************
* FUNCTION NAME:CheckReceiveFlag
* FUNCTION     :check receive data or not
* INPUT        :none
* OUTPUT       :flag: 0 no data; 1 receive data
****************************************************************/
byte ELECHOUSE_CC1101::CheckReceiveFlag(void)
{
    if (trxstate != 2)
        SetRx();

    if (digitalRead(GDO0))                      //receive data
    {
        while (digitalRead(GDO0))
            ;

        return 1;
    }
    else                                                        // no data
    {
        return 0;
    }
}


/****************************************************************
* FUNCTION NAME:ReceiveData
* FUNCTION     :read data received from RXfifo
* INPUT        :rxBuffer: buffer to store data
* OUTPUT       :size of data received
****************************************************************/
byte ELECHOUSE_CC1101::ReceiveData(byte *rxBuffer)
{
    byte size;
    byte status[2];

    if (SpiReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO)
    {
        size = SpiReadReg(CC1101_RXFIFO);
        SpiReadBurstReg(CC1101_RXFIFO, rxBuffer, size);
        SpiReadBurstReg(CC1101_RXFIFO, status, 2);
        SpiStrobe(CC1101_SFRX);
        SpiStrobe(CC1101_SRX);
        return size;
    }
    else
    {
        SpiStrobe(CC1101_SFRX);
        SpiStrobe(CC1101_SRX);
        return 0;
    }
}


ELECHOUSE_CC1101 ELECHOUSE_cc1101;
