//==============================================================================
//
//
//  CRYSTALFONTZ
//
//  This code drives the CFAL12856A0-0151-B display
//  https://www.crystalfontz.com/product/cfal12856a00151b
//
//  The controller is a Solomon Systech SSD1309
//    https://www.crystalfontz.com/controllers/SolomonSystech/SSD1309/
//
//  Seeeduino v4.2, an open-source 3.3v capable Arduino clone.
//    https://www.seeedstudio.com/Seeeduino-V4.2-p-2517.html
//    https://github.com/SeeedDocument/SeeeduinoV4/raw/master/resources/Seeeduino_v4.2_sch.pdf
//==============================================================================
//
//==============================================================================
//This is free and unencumbered software released into the public domain.
//
//Anyone is free to copy, modify, publish, use, compile, sell, or
//distribute this software, either in source code form or as a compiled
//binary, for any purpose, commercial or non-commercial, and by any
//means.
//
//In jurisdictions that recognize copyright laws, the author or authors
//of this software dedicate any and all copyright interest in the
//software to the public domain. We make this dedication for the benefit
//of the public at large and to the detriment of our heirs and
//successors. We intend this dedication to be an overt act of
//relinquishment in perpetuity of all present and future rights to this
//software under copyright law.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
//OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
//OTHER DEALINGS IN THE SOFTWARE.
//
//For more information, please refer to <http://unlicense.org/>
//
//
//==============================================================================
//
// Interface and Pinnout
//
//==============================================================================
//
//------------+-------+------------+-----------------------------
// LCD control lines
//------------+-------+------------+-----------------------------
//   ARD      | Port  | Display    |  Function - 8080 Parallel
//------------+-------+------------+-----------------------------
//  N/A	      |       | 33-35      |  BL POWER 3.2V
//  3.3V      |       | 24,26      |  POWER 3.3V
//  GND	      |       | 1,25,32,37 |  GROUND
// -----------+-------+------------+-----------------------------
//  A0        | PORTC | 22         |  Data/Command        (DC)
//  A1        | PORTC | 21         |  Write               (WR)
//  A2        | PORTC | 20         |  Read                (RD)
//  D8        | PORTB | 23         |  Chip Enable Signal  (CS)
//  D9        | PORTB | 3          |  Reset            (RESET)
// -----------+-------+------------+-----------------------------
// Data Bits
// -----------+-------+------------+-----------------------------
//  D0        | PORTD | 12         |  DB0
//  D1        | PORTD | 13         |  DB1
//  D2        | PORTD | 14         |  DB2
//  D3        | PORTD | 15         |  DB3
//  D4        | PORTD | 16         |  DB4
//  D5        | PORTD | 17         |  DB5
//  D6        | PORTD | 18         |  DB6
//  D7        | PORTD | 19         |  DB7
//
// To use SD:
#define SD_CS 10
// -----------+-------+------------+-----------------------------
// uSD control lines
// -----------+-------+------------+-----------------------------
//   ARD      | Port  | Function - SPI
//------------+-------+----------------------------
//  3.3V      |       | POWER 3.3V
//  GND	      |       | GROUND
// -----------+-------+----------------------------
//  SD_CS     | PORTB | uSD CS
//  D11       | PORTB | MOSI
//  D12       | PORTB | MISO
//  D13       | PORTB | SCL
//
//==============================================================================

#define CLR_CS (PORTB &= ~(0x01))    //pin #8  - Chip Enable Signal
#define SET_CS (PORTB |= (0x01))     //pin #8  - Chip Enable Signal
#define CLR_RESET (PORTB &= ~(0x02)) //pin #9  - Reset
#define SET_RESET (PORTB |= (0x02))  //pin #9  - Reset
#define CLR_DC (PORTC &= ~(0x01))    //pin #14 - Data/Instruction
#define SET_DC (PORTC |= (0x01))     //pin #14 - Data/Instruction
#define CLR_WR (PORTC &= ~(0x02))    //pin #15 - Write
#define SET_WR (PORTC |= (0x02))     //pin #15 - Write
#define CLR_RD (PORTC &= ~(0x04))    //pin #16 - Read
#define SET_RD (PORTC |= (0x04))     //pin #16 - Read

#include <SD.h>
#include <avr/io.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#define HRES 176
#define VRES 220

//================================================================================
// Set up SoftwareSerial for debugging purposes
SoftwareSerial mySerial(19, 18);


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Write Command and Data Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void write_command(unsigned char command)
{
    CLR_CS; // chip selected
    CLR_DC; // set to write to the control registers
    SET_RD; // not reading
    SET_WR; // not writing yet

    PORTD = 0x00; // write the command to the port

    CLR_WR; // bring write line low
    SET_WR; // clock in the command

    PORTD = command; // write the command to the port

    CLR_WR; // bring write line low
    SET_WR; // clock in the command

    SET_CS; // unselect chip
    SET_DC; // set for data
}

void write_data(unsigned char data_h, unsigned char data_l)
{
    CLR_CS; // chip selected
    SET_DC; // set to write data
    SET_RD; // not reading
    SET_WR; // not writing yet

    PORTD = data_h; // write the command to the port

    CLR_WR; // bring write line low
    SET_WR; // clock in the command

    PORTD = data_l; // write the command to the port

    CLR_WR; // bring write line low
    SET_WR; // clock in the command

    SET_CS; // unselect chip
    SET_DC; // set for data
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialize The Display
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Init()
{
    SET_RESET;
    delay(10);
    CLR_RESET;
    delay(100);
    SET_RESET;
    delay(500);

    //************* Start Initial Sequence **********//
    write_command(0x01);    // Driver Output Control Register (P14, OTM2201A Datasheet)
    write_data(0x01, 0x1C); // Decrement Address counter
    // 528x220 dots (176xRGBx220)

    write_command(0x02);
    write_data(0x01, 0x00);

    write_command(0x03); //Entry Mode
    // write_data(0x10,0x30); //This will orient the display so that it updates top -> bottom, left -> right
    write_data(0x00, 0x10); //This will orient the display so that it updated bottom -> top, left -> right (for bmp files)

    write_command(0x08);
    write_data(0x08, 0x08);

    write_command(0x0c);
    write_data(0x00, 0x00);

    write_command(0x0f);
    write_data(0x00, 0x01);

    write_command(0x20);
    write_data(0x00, 0x00);

    write_command(0x21);
    write_data(0x00, 0x00);

    //*************Power On sequence ****************//
    write_command(0x10);
    write_data(0x00, 0x00);

    write_command(0x11);
    write_data(0x10, 0x00);
    _delay_ms(100);

    //------------------------ Set GRAM area --------------------------------//
    write_command(0x30);
    write_data(0x00, 0x00);

    write_command(0x31);
    write_data(0x00, 0xdb);

    write_command(0x32);
    write_data(0x00, 0x00);

    write_command(0x33);
    write_data(0x00, 0x00);

    write_command(0x34);
    write_data(0x00, 0xdb);

    write_command(0x35);
    write_data(0x00, 0x00);

    write_command(0x36);
    write_data(0x00, 0xaf);

    write_command(0x37);
    write_data(0x00, 0x00);

    write_command(0x38);
    write_data(0x00, 0xdb);

    write_command(0x39);
    write_data(0x00, 0x00);
    _delay_ms(10);
    write_command(0xff);
    write_data(0x00, 0x03);

    // ----------- Adjust the Gamma  Curve ----------//
    write_command(0x50);
    write_data(0x02, 0x03);

    write_command(0x051);
    write_data(0x0a, 0x09);

    write_command(0x52);
    write_data(0x00, 0x05);

    write_command(0x53);
    write_data(0x10, 0x21);

    write_command(0x54);
    write_data(0x06, 0x02);

    write_command(0x55);
    write_data(0x00, 0x03);

    write_command(0x56);
    write_data(0x07, 0x03);

    write_command(0x57);
    write_data(0x05, 0x07);

    write_command(0x58);
    write_data(0x10, 0x21);

    write_command(0x59);
    write_data(0x07, 0x03);

    //***************************************
    write_command(0xB0);    //VCOM
    write_data(0x25, 0x01); //??????????

    //********************************************************
    write_command(0xFF);
    write_data(0x00, 0x00);

    write_command(0x07);
    write_data(0x10, 0x17);
    _delay_ms(200);
    write_command(0x22);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void display_color(unsigned char data_h, unsigned char data_l)
{
    unsigned int i, j;
    write_command(0x20);
    write_data(0x00, 0x00);
    write_command(0x21);
    write_data(0x00, 0x00);
    write_command(0x22);

    for (i = 0; i < 220; i++)
    {
        for (j = 0; j < 176; j++)
        {
            write_data(data_h, data_l);
        }
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Image From SD Card
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define BMP_FLIP 0 /* enabling this draws BMP images the right way up */
void show_BMPs_in_root(void)
{

    write_command(0x20);
    write_data(0x00, 0x00);
    write_command(0x21);
    write_data(0x00, 0x00);
    write_command(0x22);

    File root_dir;
    root_dir = SD.open("/");
    if (0 == root_dir)
    {
        Serial.println("show_BMPs_in_root: Can't open \"root\"");
        return;
    }
    File bmp_file;

    while (1)
    {

        bmp_file = root_dir.openNextFile();
        if (0 == bmp_file)
        {
            // no more files, break out of while()
            // root_dir will be closed below.
            break;
        }
        //Skip directories (what about volume name?)
        if (0 == bmp_file.isDirectory())
        {
            //The file name must include ".BMP"
            if (0 != strstr(bmp_file.name(), ".BMP"))
            {
                //The BMP must be ~116214 long
                //(this is correct for 240x320, 24-bit)
                if (116210 <= bmp_file.size() <= 116220)
                {
                    //Jump over BMP header. BMP must be 240x320 24-bit
                    bmp_file.seek(54);

                    //Since we are limited in memory, break the line up from
                    // 176*3 = 528 bytes into two chunks of 88 pixels
                    // each 88*3 = 264 bytes.
                    //Making this static speeds it up slightly (10ms)
                    //Reduces flash size by 114 bytes, and uses 240 bytes.
                    static uint8_t third_of_a_line[88 * 3];
                    uint8_t upperByte;
                    uint8_t lowerByte;

                    for (uint16_t line = 0; line < 220; line++)
                    {
                        for (uint8_t line_section = 0; line_section < 2; line_section++)
                        {
                            //Get a third of the line
                            bmp_file.read(third_of_a_line, sizeof(third_of_a_line));
                            //Now write this third to the TFT, doing the BGR -> RGB
                            //color fixup interlaced with the SPI transfers.
                            // SPI_send_pixels(80 * 3, third_of_a_line);
                            for (int i = 0; i < sizeof(third_of_a_line); i += 3)
                            {
                                lowerByte = (third_of_a_line[i] & 0xf8) | ((third_of_a_line[i + 1] >> 5) & 0x07);
                                upperByte = ((third_of_a_line[i + 1] << 5) & 0xe0) | ((third_of_a_line[i + 2] >> 3) & 0x1f);
                                write_data(lowerByte, upperByte);
                            }
                        }
                    }
                }
            }
        }
        //Release the BMP file handle
        bmp_file.close();

        delay(100);
    }
    //Release the root directory file handle
    root_dir.close();
}

//================================================================================
void setup()
{
    //setup ports
    DDRD = 0xff;
    DDRC = 0xff;
    DDRB = 0x03;

    PORTD = 0xff;

    SET_RD;
    SET_WR;
    SET_CS;
    mySerial.begin(9600);
    delay(100);
    for (int i = 0; i < 20; i++)
        mySerial.println();
    mySerial.println("Serial Initialized");
    //If an SD card is connected, do the SD setup
    if (!SD.begin(SD_CS))
    {
        mySerial.println("SD could not initialize");
    }
    Init();
}

void loop()
{
    mySerial.println("top of loop");
    display_color(0xFF, 0x00);
    show_BMPs_in_root();
    while (1)
        ;
}
