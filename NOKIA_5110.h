// Project: Fuzzy LCD Voltmeter
// File: NOKIA_5110.h
// Author: Chris Yan
// Created: January, 2012
// Revised: 
//  Desc: Commands, fonts, and class for using a
//      Nokia 5110 LCD via the Phillips 8554 LCD driver.
// 
//  Typical Usage: User must fill the LcdPins struct with the pinout used to control the LCD and
//      instantiate the NokiaLcd class - passing the created LcdPins struct to the constructor.
//      The class function NokiaLcd::InitLcd may then be called to reset and start the LCD driver.
//      A simple 6x6 font (6x8 in LCD space and ~5x5 character space) is included to facilitate 
//      the NokiaLcd::DrawChar( char character ) function, which will copy the character 8 bits 
//      at a time for 6 clock cycles.
//                 Commands may be sent to the LCD via the NokiaLcd::SendFunction(char cmd) 
//      function, but be aware that certain commands require the Function Set register's H-value
//      to be either 1 or 0, depending on the command. This class does not check to see whether
//      the H-value is of proper status. The Function Set register /may/ be changed via the 
//      NokiaLcd::SendFunction(char cmd), but the code uses this internally and expects that
//      most function registers have not been changed by the user.
//
//      Example:
//          #include "mbed.h"
//          #include "NOKIA_5110.h"
//
//          int main() {
//              LcdPins myLcdPins = { p11, NC, p13, p10, p8, p9 };
//              NokiaLcd myLcd( myLcdPins );    // SPI is started here (8-bits, mode 1)
//              myLcd.InitLcd();                // LCD is reset and DDRAM is cleared
//              myLcd.TestLcd( 0xAA );          // Draws a vertical pattern where every other pixel is on 
//              wait(10);                       
//              myLcd.ShutdownLcd();            // Clears the LCD's DDRAM and powers it down via CMD_FS_POWER_DOWN_MODE, H=0
//              while(1)
//              {   };
//          }

#ifndef Included_NokiaLcd_H
#define Included_NokiaLcd_H

#include "mbed.h"

class NokiaLcd
{
    public:
        NokiaLcd(SPI *spi, PinName dc, PinName sce, PinName rst);
        ~NokiaLcd();
        
    public:
        void InitLcd();
        void ClearLcdMem();
        void ShutdownLcd();
        void SendCmd(char cmd);
 
        void TestLcd(char test_pattern);
        void Char(char character, char invert);
        void String(char *str, char invert);
        void SendData(char data);
        void setDispPos(int DispX, int DispY);
        void setCharPos(int x, int y);
        void SetLine(int line, char *str, bool invert);
        void SetLineX(int x, int line, char *str, bool invert);
        void Update();
        void Bitmap(const char *bmap, int bmap_len, int DispX = 0, int DispY = 0);
        void Refresh();
        void ShowError(const char *str_cons, const char *l1, const char *l2, const char *l3);
        void ShowSplash(const char *l1, const char *l2, const char *l3);
    private:
        void ResetLcd();
        
    private:
        DigitalOut*     RST;
        DigitalOut*     SCE;
        DigitalOut*     DC;
        SPI*            LcdSpi;
        char  curr[6][14];
        char  next[6][14];
        char  invert[6];
        int   posx;
        int   posy;
        
};

#endif // Included_NokiaLcd_H
