/*******************************************************************************************
  Font name: Courier New
  Font width: 8 (monospaced font)
  Font height: 8
  Data length: 8 bits
  Data format: Big Endian, Row based, Row preferred, Unpacked

  Create time: 13:34 03-11-2024  by BitFontCreator (e-mail: support@iseatech.com)
 *******************************************************************************************/

#include "fonts.h"

/*******************************************************************************************
  Data table provides the bitmap data of each character.

  To get the starting data offset of character 'A', you can use the following expression:

     const unsigned char index = index_table['A'];
     const unsigned int offset = offset_table[index];
     const unsigned char *pData = data_table[offset];

 *******************************************************************************************/
const unsigned char FixedSys_8x8_data_table[] = {

/* character 0x20 (' '): (width=8, offset=0) */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 

/* character 0x21 ('!'): (width=8, offset=8) */
0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x10, 0x00, 

/* character 0x22 ('"'): (width=8, offset=16) */
0x28, 0x28, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 

/* character 0x23 ('#'): (width=8, offset=24) */
0x14, 0x28, 0x7C, 0x28, 0x28, 0x7C, 0x28, 0x50, 

/* character 0x24 ('$'): (width=8, offset=32) */
0x10, 0x38, 0x40, 0x30, 0x08, 0x70, 0x20, 0x00, 

/* character 0x25 ('%'): (width=8, offset=40) */
0x20, 0x50, 0x20, 0x78, 0x10, 0x28, 0x10, 0x00, 

/* character 0x26 ('&'): (width=8, offset=48) */
0x00, 0x18, 0x20, 0x20, 0x58, 0x48, 0x3C, 0x00, 

/* character 0x27 ('''): (width=8, offset=56) */
0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 

/* character 0x28 ('('): (width=8, offset=64) */
0x08, 0x08, 0x10, 0x10, 0x10, 0x08, 0x08, 0x00, 

/* character 0x29 (')'): (width=8, offset=72) */
0x20, 0x20, 0x10, 0x10, 0x10, 0x20, 0x20, 0x00, 

/* character 0x2A ('*'): (width=8, offset=80) */
0x10, 0x7C, 0x10, 0x28, 0x00, 0x00, 0x00, 0x00, 

/* character 0x2B ('+'): (width=8, offset=88) */
0x10, 0x10, 0x10, 0x7C, 0x10, 0x10, 0x10, 0x00, 

/* character 0x2C (','): (width=8, offset=96) */
0x00, 0x00, 0x00, 0x00, 0x18, 0x30, 0x20, 0x00, 

/* character 0x2D ('-'): (width=8, offset=104) */
0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 

/* character 0x2E ('.'): (width=8, offset=112) */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 

/* character 0x2F ('/'): (width=8, offset=120) */
0x04, 0x08, 0x08, 0x10, 0x10, 0x20, 0x40, 0x00, 

/* character 0x30 ('0'): (width=8, offset=128) */
0x38, 0x44, 0x4C, 0x54, 0x64, 0x44, 0x38, 0x00,

/* character 0x31 ('1'): (width=8, offset=136) */
0x30, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 

/* character 0x32 ('2'): (width=8, offset=144) */
0x38, 0x44, 0x08, 0x10, 0x20, 0x40, 0x7C, 0x00, 

/* character 0x33 ('3'): (width=8, offset=152) */
0x38, 0x44, 0x04, 0x18, 0x04, 0x44, 0x38, 0x00, 

/* character 0x34 ('4'): (width=8, offset=160) */
0x08, 0x18, 0x28, 0x48, 0x7C, 0x08, 0x1C, 0x00, 

/* character 0x35 ('5'): (width=8, offset=168) */
0x3C, 0x20, 0x20, 0x38, 0x04, 0x44, 0x38, 0x00, 

/* character 0x36 ('6'): (width=8, offset=176) */
0x1C, 0x20, 0x40, 0x78, 0x44, 0x44, 0x38, 0x00, 

/* character 0x37 ('7'): (width=8, offset=184) */
0x7C, 0x44, 0x04, 0x08, 0x08, 0x10, 0x10, 0x00, 

/* character 0x38 ('8'): (width=8, offset=192) */
0x38, 0x44, 0x44, 0x38, 0x44, 0x44, 0x38, 0x00, 

/* character 0x39 ('9'): (width=8, offset=200) */
0x38, 0x44, 0x44, 0x3C, 0x04, 0x08, 0x70, 0x00, 

/* character 0x3A (':'): (width=8, offset=208) */
0x00, 0x30, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 

/* character 0x3B (';'): (width=8, offset=216) */
0x00, 0x18, 0x00, 0x00, 0x18, 0x30, 0x20, 0x00, 

/* character 0x3C ('<'): (width=8, offset=224) */
0x00, 0x04, 0x18, 0x60, 0x18, 0x04, 0x00, 0x00, 

/* character 0x3D ('='): (width=8, offset=232) */
0x00, 0x00, 0x7C, 0x00, 0x7C, 0x00, 0x00, 0x00, 

/* character 0x3E ('>'): (width=8, offset=240) */
0x00, 0x40, 0x30, 0x0C, 0x30, 0x40, 0x00, 0x00, 

/* character 0x3F ('?'): (width=8, offset=248) */
0x30, 0x48, 0x08, 0x10, 0x10, 0x00, 0x30, 0x00, 

/* character 0x40 ('@'): (width=8, offset=256) */
0x38, 0x44, 0x4C, 0x54, 0x5C, 0x40, 0x3C, 0x00, 

/* character 0x41 ('A'): (width=8, offset=264) */
0x30, 0x10, 0x28, 0x28, 0x38, 0x44, 0xEE, 0x00, 

/* character 0x42 ('B'): (width=8, offset=272) */
0xF8, 0x44, 0x44, 0x78, 0x44, 0x44, 0xF8, 0x00, 

/* character 0x43 ('C'): (width=8, offset=280) */
0x3C, 0x44, 0x40, 0x40, 0x40, 0x44, 0x38, 0x00, 

/* character 0x44 ('D'): (width=8, offset=288) */
0xF0, 0x48, 0x44, 0x44, 0x44, 0x48, 0xF0, 0x00, 

/* character 0x45 ('E'): (width=8, offset=296) */
0x7C, 0x24, 0x28, 0x38, 0x28, 0x24, 0x7C, 0x00, 

/* character 0x46 ('F'): (width=8, offset=304) */
0x7C, 0x24, 0x28, 0x38, 0x28, 0x20, 0x70, 0x00, 

/* character 0x47 ('G'): (width=8, offset=312) */
0x3C, 0x44, 0x40, 0x40, 0x4E, 0x44, 0x38, 0x00, 

/* character 0x48 ('H'): (width=8, offset=320) */
0xEE, 0x44, 0x44, 0x7C, 0x44, 0x44, 0xEE, 0x00, 

/* character 0x49 ('I'): (width=8, offset=328) */
0x7C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 

/* character 0x4A ('J'): (width=8, offset=336) */
0x3C, 0x08, 0x08, 0x08, 0x48, 0x48, 0x30, 0x00, 

/* character 0x4B ('K'): (width=8, offset=344) */
0xEE, 0x44, 0x48, 0x70, 0x48, 0x44, 0xE6, 0x00, 

/* character 0x4C ('L'): (width=8, offset=352) */
0x70, 0x20, 0x20, 0x20, 0x24, 0x24, 0x7C, 0x00, 

/* character 0x4D ('M'): (width=8, offset=360) */
0xEE, 0x6C, 0x6C, 0x54, 0x44, 0x44, 0xEE, 0x00, 

/* character 0x4E ('N'): (width=8, offset=368) */
0xEE, 0x64, 0x64, 0x54, 0x54, 0x4C, 0xEC, 0x00, 

/* character 0x4F ('O'): (width=8, offset=376) */
0x38, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38, 0x00, 

/* character 0x50 ('P'): (width=8, offset=384) */
0x78, 0x24, 0x24, 0x24, 0x38, 0x20, 0x70, 0x00, 

/* character 0x51 ('Q'): (width=8, offset=392) */
0x38, 0x44, 0x44, 0x44, 0x44, 0x38, 0x1C, 0x00, 

/* character 0x52 ('R'): (width=8, offset=400) */
0x78, 0x24, 0x24, 0x24, 0x38, 0x24, 0x72, 0x00, 

/* character 0x53 ('S'): (width=8, offset=408) */
0x34, 0x4C, 0x40, 0x38, 0x04, 0x44, 0x78, 0x00, 

/* character 0x54 ('T'): (width=8, offset=416) */
0x7C, 0x54, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 

/* character 0x55 ('U'): (width=8, offset=424) */
0xEE, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38, 0x00, 

/* character 0x56 ('V'): (width=8, offset=432) */
0xEE, 0x44, 0x44, 0x48, 0x28, 0x28, 0x30, 0x00, 

/* character 0x57 ('W'): (width=8, offset=440) */
0xEE, 0x44, 0x54, 0x54, 0x54, 0x54, 0x28, 0x00, 

/* character 0x58 ('X'): (width=8, offset=448) */
0xC6, 0x44, 0x28, 0x10, 0x28, 0x44, 0xC6, 0x00, 

/* character 0x59 ('Y'): (width=8, offset=456) */
0xEE, 0x44, 0x28, 0x10, 0x10, 0x10, 0x38, 0x00, 

/* character 0x5A ('Z'): (width=8, offset=464) */
0x7C, 0x44, 0x08, 0x10, 0x20, 0x44, 0x7C, 0x00, 

/* character 0x5B ('['): (width=8, offset=472) */
0x18, 0x10, 0x10, 0x10, 0x10, 0x10, 0x18, 0x00, 

/* character 0x5C ('\'): (width=8, offset=480) */
0x40, 0x20, 0x20, 0x10, 0x10, 0x08, 0x04, 0x00, 

/* character 0x5D (']'): (width=8, offset=488) */
0x30, 0x10, 0x10, 0x10, 0x10, 0x10, 0x30, 0x00, 

/* character 0x5E ('^'): (width=8, offset=496) */
0x10, 0x28, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 

/* character 0x5F ('_'): (width=8, offset=504) */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 

/* character 0x60 ('`'): (width=8, offset=512) */
0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 

/* character 0x61 ('a'): (width=8, offset=520) */
0x00, 0x00, 0x38, 0x44, 0x3C, 0x44, 0x7E, 0x00, 

/* character 0x62 ('b'): (width=8, offset=528) */
0xC0, 0x40, 0x58, 0x64, 0x44, 0x44, 0xF8, 0x00, 

/* character 0x63 ('c'): (width=8, offset=536) */
0x00, 0x00, 0x3C, 0x44, 0x40, 0x40, 0x3C, 0x00, 

/* character 0x64 ('d'): (width=8, offset=544) */
0x0C, 0x04, 0x34, 0x4C, 0x44, 0x44, 0x3E, 0x00, 

/* character 0x65 ('e'): (width=8, offset=552) */
0x00, 0x00, 0x38, 0x44, 0x7C, 0x40, 0x3C, 0x00, 

/* character 0x66 ('f'): (width=8, offset=560) */
0x0C, 0x10, 0x3C, 0x10, 0x10, 0x10, 0x3C, 0x00, 

/* character 0x67 ('g'): (width=8, offset=568) */
0x36, 0x4C, 0x44, 0x44, 0x3C, 0x04, 0x38, 0x00, 

/* character 0x68 ('h'): (width=8, offset=576) */
0xC0, 0x40, 0x58, 0x64, 0x44, 0x44, 0xEE, 0x00, 

/* character 0x69 ('i'): (width=8, offset=584) */
0x10, 0x00, 0x70, 0x10, 0x10, 0x10, 0x7C, 0x00, 

/* character 0x6A ('j'): (width=8, offset=592) */
0x10, 0x00, 0x78, 0x08, 0x08, 0x08, 0x70, 0x00, 

/* character 0x6B ('k'): (width=8, offset=600) */
0xC0, 0x40, 0x5C, 0x48, 0x70, 0x48, 0xDC, 0x00, 

/* character 0x6C ('l'): (width=8, offset=608) */
0x30, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 

/* character 0x6D ('m'): (width=8, offset=616) */
0x00, 0x00, 0xE8, 0x54, 0x54, 0x54, 0xFE, 0x00, 

/* character 0x6E ('n'): (width=8, offset=624) */
0x00, 0x00, 0xD8, 0x64, 0x44, 0x44, 0xEE, 0x00, 

/* character 0x6F ('o'): (width=8, offset=632) */
0x00, 0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 

/* character 0x70 ('p'): (width=8, offset=640) */
0xD8, 0x64, 0x44, 0x44, 0x78, 0x40, 0xE0, 0x00, 

/* character 0x71 ('q'): (width=8, offset=648) */
0x36, 0x4C, 0x44, 0x44, 0x3C, 0x04, 0x0E, 0x00, 

/* character 0x72 ('r'): (width=8, offset=656) */
0x00, 0x00, 0x6C, 0x30, 0x20, 0x20, 0x78, 0x00, 

/* character 0x73 ('s'): (width=8, offset=664) */
0x00, 0x00, 0x3C, 0x40, 0x38, 0x04, 0x78, 0x00, 

/* character 0x74 ('t'): (width=8, offset=672) */
0x00, 0x40, 0xF8, 0x40, 0x40, 0x44, 0x38, 0x00, 

/* character 0x75 ('u'): (width=8, offset=680) */
0x00, 0x00, 0xCC, 0x44, 0x44, 0x4C, 0x36, 0x00, 

/* character 0x76 ('v'): (width=8, offset=688) */
0x00, 0x00, 0xEE, 0x44, 0x48, 0x28, 0x30, 0x00, 

/* character 0x77 ('w'): (width=8, offset=696) */
0x00, 0x00, 0xEE, 0x44, 0x54, 0x54, 0x28, 0x00, 

/* character 0x78 ('x'): (width=8, offset=704) */
0x00, 0x00, 0x6C, 0x28, 0x10, 0x28, 0x6C, 0x00, 

/* character 0x79 ('y'): (width=8, offset=712) */
0xEE, 0x44, 0x28, 0x28, 0x10, 0x10, 0x70, 0x00, 

/* character 0x7A ('z'): (width=8, offset=720) */
0x00, 0x00, 0x7C, 0x48, 0x10, 0x24, 0x7C, 0x00, 

/* character 0x7B ('{'): (width=8, offset=728) */
0x08, 0x10, 0x10, 0x30, 0x10, 0x10, 0x08, 0x00, 

/* character 0x7C ('|'): (width=8, offset=736) */
0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 

/* character 0x7D ('}'): (width=8, offset=744) */
0x20, 0x10, 0x10, 0x18, 0x10, 0x10, 0x20, 0x00, 

/* character 0x7E ('~'): (width=8, offset=752) */
0x00, 0x00, 0x00, 0x34, 0x58, 0x00, 0x00, 0x00, 

/* character 0x7F (''): (width=8, offset=760) */
0xE0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xE0, 0x00, 

};

/*******************************************************************************************
  The offset talbe is skipped.

  This is a monospaced font, the bitmap data size of each character is the same.
  You can calculate the offset of each character easily.

  For example:

  To get the starting offset of character 'A', you can use the following expression:

     const unsigned char index = index_table['A'];
     const unsigned int offset = index * sizeGlyph;
                                 (here sizeGlyph is the data size of single character)
 *******************************************************************************************/
static const uint16_t FixedSys_8x8_offset_table[] = {
		0,
		8,
		16,
		24,
		32,
		40,
		48,
		56,
		64,
		72,
		80,
		88,
		96,
		104,
		112,
		120,
		128,
		136,
		144,
		152,
		160,
		168,
		176,
		184,
		192,
		200,
		208,
		216,
		224,
		232,
		240,
		248,
		256,
		264,
		272,
		280,
		288,
		296,
		304,
		312,
		320,
		328,
		336,
		344,
		352,
		360,
		368,
		376,
		384,
		392,
		400,
		408,
		416,
		424,
		432,
		440,
		448,
		456,
		464,
		472,
		480,
		488,
		496,
		504,
		512,
		520,
		528,
		536,
		544,
		552,
		560,
		568,
		576,
		584,
		592,
		600,
		608,
		616,
		624,
		632,
		640,
		648,
		656,
		664,
		672,
		680,
		688,
		696,
		704,
		712,
		720,
		728,
		736,
		744,
		752,
		760,
};

/*******************************************************************************************
  Index table is used to find the mapping index of a character.

  If you can find a simple mathematical expression for index mapping, you can use that.
  If you do not have such a mathematical expression, this index table is just for you.

  To get the index of character 'A', you can use the following expression:

     const unsigned char index = index_table['A'];

 *******************************************************************************************/
static const unsigned char FixedSys_8x8_index_table[] = {
/*		index   hexcode   decimal  char */
/*		=====   =======   =======  ==== */
  		  0, /*   00          0     .   */
  		  0, /*   01          1     .   */
  		  0, /*   02          2     .   */
  		  0, /*   03          3     .   */
  		  0, /*   04          4     .   */
  		  0, /*   05          5     .   */
  		  0, /*   06          6     .   */
  		  0, /*   07          7     .   */
  		  0, /*   08          8     .   */
  		  0, /*   09          9     .   */
  		  0, /*   0A         10     .   */
  		  0, /*   0B         11     .   */
  		  0, /*   0C         12     .   */
  		  0, /*   0D         13     .   */
  		  0, /*   0E         14     .   */
  		  0, /*   0F         15     .   */
  		  0, /*   10         16     .   */
  		  0, /*   11         17     .   */
  		  0, /*   12         18     .   */
  		  0, /*   13         19     .   */
  		  0, /*   14         20     .   */
  		  0, /*   15         21     .   */
  		  0, /*   16         22     .   */
  		  0, /*   17         23     .   */
  		  0, /*   18         24     .   */
  		  0, /*   19         25     .   */
  		  0, /*   1A         26     .   */
  		  0, /*   1B         27     .   */
  		  0, /*   1C         28     .   */
  		  0, /*   1D         29     .   */
  		  0, /*   1E         30     .   */
  		  0, /*   1F         31     .   */
  		  0, /*   20         32         */
  		  1, /*   21         33     !   */
  		  2, /*   22         34     "   */
  		  3, /*   23         35     #   */
  		  4, /*   24         36     $   */
  		  5, /*   25         37     %   */
  		  6, /*   26         38     &   */
  		  7, /*   27         39     '   */
  		  8, /*   28         40     (   */
  		  9, /*   29         41     )   */
  		 10, /*   2A         42     *   */
  		 11, /*   2B         43     +   */
  		 12, /*   2C         44     ,   */
  		 13, /*   2D         45     -   */
  		 14, /*   2E         46     .   */
  		 15, /*   2F         47     /   */
  		 16, /*   30         48     0   */
  		 17, /*   31         49     1   */
  		 18, /*   32         50     2   */
  		 19, /*   33         51     3   */
  		 20, /*   34         52     4   */
  		 21, /*   35         53     5   */
  		 22, /*   36         54     6   */
  		 23, /*   37         55     7   */
  		 24, /*   38         56     8   */
  		 25, /*   39         57     9   */
  		 26, /*   3A         58     :   */
  		 27, /*   3B         59     ;   */
  		 28, /*   3C         60     <   */
  		 29, /*   3D         61     =   */
  		 30, /*   3E         62     >   */
  		 31, /*   3F         63     ?   */
  		 32, /*   40         64     @   */
  		 33, /*   41         65     A   */
  		 34, /*   42         66     B   */
  		 35, /*   43         67     C   */
  		 36, /*   44         68     D   */
  		 37, /*   45         69     E   */
  		 38, /*   46         70     F   */
  		 39, /*   47         71     G   */
  		 40, /*   48         72     H   */
  		 41, /*   49         73     I   */
  		 42, /*   4A         74     J   */
  		 43, /*   4B         75     K   */
  		 44, /*   4C         76     L   */
  		 45, /*   4D         77     M   */
  		 46, /*   4E         78     N   */
  		 47, /*   4F         79     O   */
  		 48, /*   50         80     P   */
  		 49, /*   51         81     Q   */
  		 50, /*   52         82     R   */
  		 51, /*   53         83     S   */
  		 52, /*   54         84     T   */
  		 53, /*   55         85     U   */
  		 54, /*   56         86     V   */
  		 55, /*   57         87     W   */
  		 56, /*   58         88     X   */
  		 57, /*   59         89     Y   */
  		 58, /*   5A         90     Z   */
  		 59, /*   5B         91     [   */
  		 60, /*   5C         92     \   */
  		 61, /*   5D         93     ]   */
  		 62, /*   5E         94     ^   */
  		 63, /*   5F         95     _   */
  		 64, /*   60         96     `   */
  		 65, /*   61         97     a   */
  		 66, /*   62         98     b   */
  		 67, /*   63         99     c   */
  		 68, /*   64        100     d   */
  		 69, /*   65        101     e   */
  		 70, /*   66        102     f   */
  		 71, /*   67        103     g   */
  		 72, /*   68        104     h   */
  		 73, /*   69        105     i   */
  		 74, /*   6A        106     j   */
  		 75, /*   6B        107     k   */
  		 76, /*   6C        108     l   */
  		 77, /*   6D        109     m   */
  		 78, /*   6E        110     n   */
  		 79, /*   6F        111     o   */
  		 80, /*   70        112     p   */
  		 81, /*   71        113     q   */
  		 82, /*   72        114     r   */
  		 83, /*   73        115     s   */
  		 84, /*   74        116     t   */
  		 85, /*   75        117     u   */
  		 86, /*   76        118     v   */
  		 87, /*   77        119     w   */
  		 88, /*   78        120     x   */
  		 89, /*   79        121     y   */
  		 90, /*   7A        122     z   */
  		 91, /*   7B        123     {   */
  		 92, /*   7C        124     |   */
  		 93, /*   7D        125     }   */
  		 94, /*   7E        126     ~   */
  		 95, /*   7F        127        */
};

/*******************************************************************************************
  The width talbe is skipped.

  This is a monospaced font, the width of each character is the same.
  width = 8 
 *******************************************************************************************/

static const uint8_t FixedSys_8x8_width_table[] = {
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
		8,
};


const sFONT FONT_FixedSys_8x8 =
{
		FixedSys_8x8_data_table,
		FixedSys_8x8_offset_table,
		FixedSys_8x8_index_table,
		FixedSys_8x8_width_table,

		8,
		128
};
