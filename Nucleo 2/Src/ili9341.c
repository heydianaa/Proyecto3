/*
 * ili9341.c - biblioteca del ejemplo ILI9341SPI
 * Created on: Aug 20, 2024
 * Author: Pablo Mazariegos
 * Adaptacion de pines, transferencia HAL y rutinas graficas para este montaje.
 * Se conserva la secuencia de inicializacion del ejemplo (area 320x240).
 */
#include "ili9341.h"
#include <stddef.h>
#include <string.h>
/* Comandos usados aqui; no requieren un archivo de registros separado. */
#define ILI9341_INVOFF 0x20u
#define ILI9341_SLPOUT 0x11u
#define ILI9341_DISPON 0x29u
#define LCD_DC_L() HAL_GPIO_WritePin(LCD_DC_PORT,LCD_DC_PIN,GPIO_PIN_RESET)
#define LCD_DC_H() HAL_GPIO_WritePin(LCD_DC_PORT,LCD_DC_PIN,GPIO_PIN_SET)
#define LCD_CS_L() HAL_GPIO_WritePin(LCD_CS_PORT,LCD_CS_PIN,GPIO_PIN_RESET)
#define LCD_CS_H() HAL_GPIO_WritePin(LCD_CS_PORT,LCD_CS_PIN,GPIO_PIN_SET)
extern SPI_HandleTypeDef hspi1;
extern const uint8_t smallFont[1140];
extern const uint16_t bigFont[1520];
static uint8_t lcd_ok;
uint8_t LCD_IsReady(void){return lcd_ok;}
static void lcd_tx(const uint8_t *data,uint16_t n)
{
    if(lcd_ok && HAL_SPI_Transmit(&hspi1,(uint8_t *)data,n,100)!=HAL_OK)lcd_ok=0;
}
void LCD_Init(void) {
    lcd_ok=1;

	//****************************************
	// Secuencia de Inicialización
	//****************************************
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
	/*Configure GPIO pin Output Level */
	//HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin | LCD_WR_Pin | LCD_RS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
	HAL_Delay(150);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);

	//****************************************
	LCD_CMD(0xE9);  // SETPANELRELATED
	LCD_DATA(0x20);
	//****************************************
	LCD_CMD(0x11); // Exit Sleep SLEEP OUT (SLPOUT)
	HAL_Delay(100);
	//****************************************
	LCD_CMD(0xD1);    // (SETVCOM)
	LCD_DATA(0x00);
	LCD_DATA(0x71);
	LCD_DATA(0x19);
	//****************************************
	LCD_CMD(0xD0);   // (SETPOWER)
	LCD_DATA(0x07);
	LCD_DATA(0x01);
	LCD_DATA(0x08);
	//****************************************
	LCD_CMD(0x36);  // (MEMORYACCESS)
	LCD_DATA(0x40 | 0x80 | 0x20 | 0x08); // LCD_DATA(0x19);
	//****************************************
	LCD_CMD(0x3A); // Set_pixel_format (PIXELFORMAT)
	LCD_DATA(0x55); // color setings, 05h - 16bit pixel, 11h - 3bit pixel
	//****************************************
	LCD_CMD(0xC1);    // (POWERCONTROL2)
	LCD_DATA(0x10);
	LCD_DATA(0x10);
	LCD_DATA(0x02);
	LCD_DATA(0x02);
	//****************************************
	LCD_CMD(0xC0); // Set Default Gamma (POWERCONTROL1)
	LCD_DATA(0x00);
	LCD_DATA(0x35);
	LCD_DATA(0x00);
	LCD_DATA(0x00);
	LCD_DATA(0x01);
	LCD_DATA(0x02);
	//****************************************
	LCD_CMD(0xC5); // Set Frame Rate (VCOMCONTROL1)
	LCD_DATA(0x04); // 72Hz
	//****************************************
	LCD_CMD(0xD2); // Power Settings  (SETPWRNORMAL)
	LCD_DATA(0x01);
	LCD_DATA(0x44);
	//****************************************
	LCD_CMD(0xC8); //Set Gamma  (GAMMASET)
	LCD_DATA(0x04);
	LCD_DATA(0x67);
	LCD_DATA(0x35);
	LCD_DATA(0x04);
	LCD_DATA(0x08);
	LCD_DATA(0x06);
	LCD_DATA(0x24);
	LCD_DATA(0x01);
	LCD_DATA(0x37);
	LCD_DATA(0x40);
	LCD_DATA(0x03);
	LCD_DATA(0x10);
	LCD_DATA(0x08);
	LCD_DATA(0x80);
	LCD_DATA(0x00);
	//****************************************
	LCD_CMD(0x2A); // Set_column_address 320px (CASET)
	LCD_DATA(0x00);
	LCD_DATA(0x00);
	LCD_DATA(0x01);
	LCD_DATA(0x3F);
	//****************************************
	LCD_CMD(0x2B); // Set_page_address 240px (PASET)
	LCD_DATA(0x00);
	LCD_DATA(0x00);
	LCD_DATA(0x00);
	LCD_DATA(0xEF);
	//  LCD_DATA(0x8F);
	LCD_CMD(0x29); //display on
	LCD_CMD(0x2C); //display on

	LCD_CMD(ILI9341_INVOFF); //Invert Off
	HAL_Delay(120);
	LCD_CMD(ILI9341_SLPOUT);    //Exit Sleep
	HAL_Delay(120);
	LCD_CMD(ILI9341_DISPON);    //Display on
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

/* Transporte SPI con timeout; CS conserva D10/PB6. */
void LCD_CMD(uint8_t cmd)
{
    LCD_CS_L();LCD_DC_L();lcd_tx(&cmd,1);LCD_CS_H();
}
void LCD_DATA(uint8_t data)
{
    LCD_CS_L();LCD_DC_H();lcd_tx(&data,1);LCD_CS_H();
}
void SetWindows(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{
    LCD_CMD(0x2A);LCD_DATA((uint8_t)(x1>>8));LCD_DATA((uint8_t)x1);
    LCD_DATA((uint8_t)(x2>>8));LCD_DATA((uint8_t)x2);
    LCD_CMD(0x2B);LCD_DATA((uint8_t)(y1>>8));LCD_DATA((uint8_t)y1);
    LCD_DATA((uint8_t)(y2>>8));LCD_DATA((uint8_t)y2);
    LCD_CMD(0x2C);
}
void FillRect(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int color)
{
    if(!lcd_ok || x>=320 || y>=240 || !w || !h)return;
    if(w>320-x)w=320-x;
    if(h>240-y)h=240-y;
    uint8_t block[256];
    for(unsigned i=0;i<sizeof(block);i+=2){block[i]=(uint8_t)(color>>8);block[i+1]=(uint8_t)color;}
    SetWindows(x,y,x+w-1,y+h-1);
    LCD_CS_L();LCD_DC_H();
    uint32_t count=(uint32_t)w*h;
    while(count && lcd_ok){unsigned n=count>128?128:(unsigned)count;lcd_tx(block,(uint16_t)(2*n));count-=n;}
    LCD_CS_H();
}
void LCD_Clear(unsigned int color){FillRect(0,0,320,240,color);}
void H_line(unsigned int x,unsigned int y,unsigned int l,unsigned int c){FillRect(x,y,l,1,c);}
void V_line(unsigned int x,unsigned int y,unsigned int l,unsigned int c){FillRect(x,y,1,l,c);}
void Rect(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int c)
{
    if(x>=320 || y>=240 || !w || !h)return;
    if(w>320-x)w=320-x;
    if(h>240-y)h=240-y;
    H_line(x,y,w,c);H_line(x,y+h-1,w,c);
    V_line(x,y,h,c);V_line(x+w-1,y,h,c);
}
void LCD_Print(char *text,int x,int y,int fontSize,int color,int background)
{
    if(!text || !lcd_ok || x<0 || y<0 || fontSize<1 || fontSize>3)return;
    unsigned sx=fontSize==1?8u:16u,sy=fontSize==1?12u:16u;
    unsigned scale=fontSize==3?2u:1u;
    unsigned w=sx*scale,h=sy*scale;
    if((unsigned)y+h>240)return;
    for(size_t i=0;text[i] && lcd_ok;i++){
        unsigned px=(unsigned)x+(unsigned)i*w;
        if(px>=320 || w>320-px)return;
        unsigned ch=(uint8_t)text[i];if(ch<32 || ch>126)ch='?';
        SetWindows(px,(unsigned)y,px+w-1,(unsigned)y+h-1);
        LCD_CS_L();LCD_DC_H();
        uint8_t row_pixels[64]; /* max 32 pixels, RGB565 */
        for(unsigned row=0;row<sy && lcd_ok;row++){
            uint16_t bits=fontSize==1?smallFont[(ch-32u)*12u+row]:bigFont[(ch-32u)*16u+row];
            unsigned out=0;
            for(unsigned col=0;col<sx;col++){
                uint16_t c=(bits&(1u<<(sx-col-1)))?(uint16_t)color:(uint16_t)background;
                for(unsigned n=0;n<scale;n++){row_pixels[out++]=(uint8_t)(c>>8);row_pixels[out++]=(uint8_t)c;}
            }
            for(unsigned n=0;n<scale;n++)lcd_tx(row_pixels,(uint16_t)out);
        }
        LCD_CS_H();
    }
}
void LCD_Bitmap(unsigned int x,unsigned int y,unsigned int w,unsigned int h,const uint16_t *bitmap)
{
    if(!bitmap || !lcd_ok || x>=320 || y>=240 || !w || !h || w>320-x || h>240-y)return;
    SetWindows(x,y,x+w-1,y+h-1);LCD_CS_L();LCD_DC_H();
    uint8_t buffer[256];uint32_t total=(uint32_t)w*h,offset=0;
    while(offset<total && lcd_ok){
        unsigned n=total-offset>128u?128u:(unsigned)(total-offset);
        for(unsigned i=0;i<n;i++){uint16_t c=bitmap[offset+i];buffer[2*i]=(uint8_t)(c>>8);buffer[2*i+1]=(uint8_t)c;}
        lcd_tx(buffer,(uint16_t)(2*n));offset+=n;
    }LCD_CS_H();
}
void LCD_BitmapTransparent(uint16_t x,uint16_t y,uint16_t w,uint16_t h,const uint16_t *bitmap,uint16_t transparentColor)
{
    if(!bitmap || !lcd_ok || x>=320 || y>=240 || !w || !h || w>320-x || h>240-y)return;
    for(unsigned row=0;row<h && lcd_ok;row++){
        unsigned col=0;
        while(col<w && lcd_ok){
            while(col<w && bitmap[(uint32_t)row*w+col]==transparentColor)col++;
            unsigned start=col;
            while(col<w && bitmap[(uint32_t)row*w+col]!=transparentColor)col++;
            if(col>start)LCD_Bitmap(x+start,y+row,col-start,1,&bitmap[(uint32_t)row*w+start]);
        }
    }
}
void LCD_BitmapFast(unsigned int x,unsigned int y,unsigned int w,unsigned int h,const uint8_t *bitmap)
{
    /* bitmap byte stream must already be RGB565, MSB first. */
    if(!bitmap || !lcd_ok || x>=320 || y>=240 || !w || !h || w>320-x || h>240-y)return;
    SetWindows(x,y,x+w-1,y+h-1);LCD_CS_L();LCD_DC_H();
    uint32_t count=(uint32_t)w*h*2u,offset=0;
    while(offset<count && lcd_ok){unsigned n=count-offset>256u?256u:(unsigned)(count-offset);lcd_tx(bitmap+offset,(uint16_t)n);offset+=n;}
    LCD_CS_H();
}
void LCD_Sprite(int x,int y,int w,int h,const uint16_t *bitmap,int columns,int index,char flip,char offset)
{
    if(!bitmap || !lcd_ok || x<0 || y<0 || w<=0 || h<=0 || columns<=0 || index<0 || index>=columns ||
       x>=320 || y>=240 || w>320-x || h>240-y || offset<0 || offset>=w)return;
    unsigned stride=(unsigned)w*(unsigned)columns;
    unsigned width=(unsigned)w-(unsigned)offset;
    for(unsigned row=0;row<(unsigned)h && lcd_ok;row++){
        SetWindows((unsigned)x,(unsigned)y+row,(unsigned)x+width-1,(unsigned)y+row);
        LCD_CS_L();LCD_DC_H();
        for(unsigned col=0;col<width && lcd_ok;col++){
            unsigned local=flip?(unsigned)w-1u-(col+(unsigned)offset):col+(unsigned)offset;
            uint16_t c=bitmap[row*stride+(unsigned)index*(unsigned)w+local];
            uint8_t b[2]={(uint8_t)(c>>8),(uint8_t)c};lcd_tx(b,2);
        }LCD_CS_H();
    }
}
