
#ifndef MAGALPHAPARTPROPERTIES_H
#define MAGALPHAPARTPROPERTIES_H

#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include <SPI.h>
#include <Wire.h>

typedef enum MagAlphaInterface{
    UNKNOWN_INTERFACE = 0,
    SPI_INTERFACE = 1,
    I2C_INTERFACE = 2,
    SSI_INTERFACE = 3,
    ABZ_INTERFACE = 4,
    UVW_INTERFACE = 5,
    SENT_INTERFACE = 6
}MagAlphaInterface;

typedef enum MagAlphaGeneration{
    UNKNOWN_GEN = 0,
    MA_GEN_1 = 1,
    MA_GEN_2 = 2,
    MA_GEN_3 = 3,
    MA_GEN_4 = 4,
    MA_GEN_5 = 5,
    MA_GEN_6 = 6,
    MA_GEN_7 = 7,
    MA_GEN_8 = 8
}MagAlphaGeneration;

typedef enum MagAlphaSPIMode{
    MODE_0 = SPI_MODE0,
    MODE_3 = SPI_MODE3
}MagAlphaSPIMode;

typedef enum MagAlphaSSIMode{
    MODE_A = SPI_MODE2,
    MODE_B = SPI_MODE1
}MagAlphaSSIMode;

typedef enum MagAlphaPartNumber{
    UNKNOWN_PART_NUMBER = 0,
    MA100 = 1,      //MA_GEN_1
    MA120 = 2,      //MA_GEN_1
    MA300 = 3,      //MA_GEN_1
    MA301 = 4,      //MA_GEN_1
    MA700 = 5,      //MA_GEN_1
    MA750 = 6,      //MA_GEN_1
    MA700A = 7,     //MA_GEN_2
    MA102 = 8,      //MA_GEN_3
    MA302 = 9,      //MA_GEN_3 
    MA310 = 10,     //MA_GEN_3
    MA330 = 11,     //MA_GEN_3
    MA702 = 12,     //MA_GEN_3
    MA704 = 13,     //MA_GEN_3
    MA710 = 14,     //MA_GEN_3
    MA730 = 15,     //MA_GEN_3
    MA731 = 16,     //MA_GEN_3
    MA732 = 17,     //MA_GEN_3
    MA800 = 18,     //MA_GEN_3
    MA820 = 19,     //MA_GEN_3
    MA850 = 20,     //MA_GEN_3
    MAQ430 = 21,    //MA_GEN_3
    MAQ470 = 22,    //MA_GEN_3
    MAQ473 = 23,    //MA_GEN_3
    MAP790 = 24,    //MA_GEN_3
    MAP791 = 25,    //MA_GEN_3
    MA780 = 26,     //MA_GEN_4
    MA781 = 27,     //MA_GEN_4
    MA734 = 28,     //MA_GEN_4
    MA760 = 29,     //MA_GEN_5
    MA600 = 30,     //MA_GEN_6
    MA900 = 31,     //MA_GEN_7
    MAQ79010FS = 32,//MA_GEN_7
    MA980 = 33,     //MA_GEN_8
    MA981 = 34,     //MA_GEN_8

}MagAlphaPartNumber;

#endif //MAGALPHAPARTPROPERTIES_H