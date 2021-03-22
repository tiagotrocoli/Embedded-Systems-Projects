// BmpConvert.cpp : Defines the entry point for the console application.
//
// ****************************** BmpConvert16.c ****************************
// Purpose: convert bmp into C constants for ST7735 displays
// simple binary to ASCII hex converter
// Last modified 7/10/13 by Jonathan W. Valvano

// Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
//    You may use, edit, run or distribute this file
//    as long as the above copyright notice remains
/* To create an image for ST7735 displays on LM4F120 or TM4C123G
1) Create a bmp file
   width less than or equal to 160 pixels
   height less than or equal 128 pixels
   save the image as a 24-bit bmp file
   store in same directory as BmpConvert16-bit.exe
2) Execute BmpConvert.exe
   Type the image name 
   E.g., if the file is horse.bmp, then type horse
3) Open the corresponding txt file, select all, copy
4) Open uVision compiler
   paste new image as a data constant
5) Draw the image by calling drawBitmap
   E.g., center a 160x120 image
         drawBitmap(4, 0, horse);
*/

#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//--------------------------outUHex----------------------------
// Output a 16-bit number in unsigned hexadecimal format to the
// specified file.  The file is assumed to be opened and closed
// elsewhere.  An invalid file may cause unpredictable results.
// Input: number       16-bit number to be transferred
//        destination  file that receives the number
// Output: none
// 4 digits with no space before or after
void static outUHex(unsigned short number, FILE *destination){
  // print the first digit
  if((number/0x1000) < 0xA){
    fputc(number/0x1000 + '0', destination);
  } else{
    fputc((number/0x1000 - 0xA) + 'A', destination);
  }
  number = number%0x1000;
  // print the second digit
  if((number/0x100) < 0xA){
    fputc(number/0x100 + '0', destination);
  } else{
    fputc((number/0x100 - 0xA) + 'A', destination);
  }
  number = number%0x100;
  // print the third digit
  if((number/0x10) < 0xA){
    fputc(number/0x10 + '0', destination);
  } else{
    fputc((number/0x10 - 0xA) + 'A', destination);
  }
  number = number%0x10;
  // print the fourth digit
  if(number < 0xA){
    fputc(number + '0', destination);
  } else{
    fputc((number - 0xA) + 'A', destination);
  }
}
//--------------------------outUHex8----------------------------
// Output an 8-bit number in unsigned hexadecimal format to the
// specified file.  The file is assumed to be opened and closed
// elsewhere.  An invalid file may cause unpredictable results.
// Input: number       8-bit number to be transferred
//        destination  file that receives the number
// Output: none
// 2 digits with no space before or after
void static outUHex8(unsigned char number, FILE *destination){
   // print the first digit
  if((number/0x10) < 0xA){
    fputc(number/0x10 + '0', destination);
  } else{
    fputc((number/0x10 - 0xA) + 'A', destination);
  }
  number = number%0x10;
  // print the second digit
  if(number < 0xA){
    fputc(number + '0', destination);
  } else{
    fputc((number - 0xA) + 'A', destination);
  }
}
// 24-bit color depth bmp files
char BMPname[48];
char TXTname[48];
char name[48];
#define NUMBERS_PER_LINE 16
void main(void){ int i, j; int red, green, blue;
  long bmpTotalSize, bmpDataOff, bmpDIBSize, bmpWidth, bmpHeight, bmpDataSize;
  short bmpPxlBits;
  unsigned short color;
  unsigned char data;
  FILE *in, *out;
  printf("This program converts a 24-bit color BMP into C source code for ST7735 displays running in 16-bit color mode\n");
  printf("Enter file: ");
  scanf("%s", BMPname);
  strcpy(name,"const unsigned short ");
  strcat(name,BMPname);
  strcat(name,"[] = {");
  strcpy(TXTname,BMPname);
  strcat(BMPname,".bmp");
  strcat(TXTname,".txt");
  printf("Converting file %s to file %s\n",BMPname, TXTname);

  if((in = fopen(BMPname, "rb"))== NULL){
    fprintf(stderr, "Cannot open bmp file.\n");
    scanf("%d",&data);
    return;
  }
  if((out = fopen(TXTname, "wt")) == NULL){
    fprintf(stderr, "Cannot open txt file.\n");
    scanf("%d",&data);
    return;
  }
  i = 0;
  while(name[i]){
    fputc(name[i], out);
    i++;
  }
  fputc(10, out);     // Line feed
/*
//exact copy code for debug purposes
  i=1;
  while (!feof(in)){
    blue = fgetc(in);
    fputc(' ', out);
    fputc('0', out);
    fputc('x', out);
    outUHex8(blue, out);
    fputc(',', out);
    if((i%NUMBERS_PER_LINE)==0){
      fputc(10, out);     // Line feed
    }
    i++;
  }*/
  // Process the BMP header and initialize variables.
  // The first two bytes are the ID field and are ignored in this context.
  fgetc(in);
  fgetc(in);
  // The next four bytes are the size of the entire bitmap file.
  bmpTotalSize = fgetc(in);
  bmpTotalSize = bmpTotalSize + (fgetc(in) << 8);
  bmpTotalSize = bmpTotalSize + (fgetc(in) << 16);
  bmpTotalSize = bmpTotalSize + (fgetc(in) << 24);
  // The next four bytes are application specific and are ignored in this context.
  fgetc(in);
  fgetc(in);
  fgetc(in);
  fgetc(in);
  // The next four bytes are the offset where the data can be found after the header.
  bmpDataOff = fgetc(in);
  bmpDataOff = bmpDataOff + (fgetc(in) << 8);
  bmpDataOff = bmpDataOff + (fgetc(in) << 16);
  bmpDataOff = bmpDataOff + (fgetc(in) << 24);
  // Process the DIB header and initialize variables.
  // The next four bytes are the number of bytes in the DIB header, including these four.
  bmpDIBSize = fgetc(in);
  bmpDIBSize = bmpDIBSize + (fgetc(in) << 8);
  bmpDIBSize = bmpDIBSize + (fgetc(in) << 16);
  bmpDIBSize = bmpDIBSize + (fgetc(in) << 24);
  if((bmpDataOff - bmpDIBSize) != 14){
    fprintf(stderr, "Warning: This file's data offset is %d bytes (expected %d bytes).\n", bmpDataOff, (bmpDIBSize + 14));
  }
  if(bmpDIBSize > 40){
    fprintf(stderr, "Warning: This file's header is %d bytes (expected 40 bytes), which may result in other errors.\n", bmpDIBSize);
  }
  if(bmpDIBSize < 40){
    fprintf(stderr, "Error: This file's header is %d bytes (expected 40 bytes).\n", bmpDIBSize);
    fclose(in);
    fclose(out);
    scanf("%d",&data);
    return;
  }
  // The next four bytes are the number of pixels wide.
  bmpWidth = fgetc(in);
  bmpWidth = bmpWidth + (fgetc(in) << 8);
  bmpWidth = bmpWidth + (fgetc(in) << 16);
  bmpWidth = bmpWidth + (fgetc(in) << 24);
  if(bmpWidth > 128){
    fprintf(stderr, "Warning: This image's width of %d exceeds the ST7735's 128 pixel width.\n", bmpWidth);
  }
  // The next four bytes are the number of pixels high.
  bmpHeight = fgetc(in);
  bmpHeight = bmpHeight + (fgetc(in) << 8);
  bmpHeight = bmpHeight + (fgetc(in) << 16);
  bmpHeight = bmpHeight + (fgetc(in) << 24);
  if(bmpHeight > 160){
    fprintf(stderr, "Warning: This image's height of %d exceeds the ST7735's 160 pixel height.\n", bmpHeight);
  }
  if(bmpHeight < 0){
    fprintf(stderr, "Warning: This image is encoded in top-to-bottom pixel order, a format not currently supported by this converter.\n");
  }
  // The next two bytes are the number of color panes and are ignored in this context.
  fgetc(in);
  fgetc(in);
  // The next two bytes are the number of bits per pixel.
  bmpPxlBits = fgetc(in);
  bmpPxlBits = bmpPxlBits + (fgetc(in) << 8);
  if(bmpPxlBits != 24){
    fprintf(stderr, "Error: This file is not a bitmap with 24 bits per pixel.\n");
    fclose(in);
    fclose(out);
    scanf("%d",&data);
    return;
  }
  // The next four bytes are related to pixel array compression and are ignored in this context.
  fgetc(in);
  fgetc(in);
  fgetc(in);
  fgetc(in);
  // The next four bytes are the size of the raw data including padding.
  bmpDataSize = fgetc(in);
  bmpDataSize = bmpDataSize + (fgetc(in) << 8);
  bmpDataSize = bmpDataSize + (fgetc(in) << 16);
  bmpDataSize = bmpDataSize + (fgetc(in) << 24);
  // The next four bytes are the horizontal resolution of the image and are ignored in this context.
  fgetc(in);
  fgetc(in);
  fgetc(in);
  fgetc(in);
  // The next four bytes are the vertical resolution of the image and are ignored in this context.
  fgetc(in);
  fgetc(in);
  fgetc(in);
  fgetc(in);
  // The next four bytes are the number of colors in the palette of the image and are ignored in this context.
  fgetc(in);
  fgetc(in);
  fgetc(in);
  fgetc(in);
  // The next four bytes are the number of "important" colors of the image and are ignored in this context.
  fgetc(in);
  fgetc(in);
  fgetc(in);
  fgetc(in);
  // If the DIB header is somehow longer than the 40 bytes that have already been read, read and discard the rest of them.
  for(i=1; i<=(bmpDIBSize-40); i=i+1){
    fgetc(in);
  }
  printf("Processing image %s:\n", BMPname);
  printf("w=%d h=%d\n", bmpWidth, bmpHeight);
  printf("%d total bytes\n", bmpTotalSize);
  printf("%d data bytes\n", bmpDataSize);
  // The remaining contents are color data and need to be converted from 24-bit color to 16-bit color.
  for(i=1; i<=(bmpWidth*bmpHeight); i=i+1){
    blue = fgetc(in);
    green = fgetc(in);
    red = fgetc(in);
    color = ((blue & 0xF8) << 8) | ((green & 0xFC) << 3) | ((red & 0xF8) >> 3);
    fputc(' ', out);
    fputc('0', out);
    fputc('x', out);
    outUHex(color, out);
    fputc(',', out);
    if((i%NUMBERS_PER_LINE)==0){
      fputc(10, out);     // Line feed
    }
    if((i%bmpWidth) == 0){// At the end of a row
      for(j=0; j<(bmpWidth%4); j=j+1){
        fgetc(in);        // Get and discard padding
      }
    }
  }
  fputc(13, out);
  fputc(10, out);
  fputc('}', out);
  fputc(';', out);
  fclose(in);
  fputc(13, out);
  fputc(10, out);
  fclose(out);
  printf("Done.\nSafe to close window or enter text and hit return.");
  scanf("%d",&data);
}
