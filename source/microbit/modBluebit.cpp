/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Mark Shannon
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "MicroBitPin.h"
#include "string.h"
//#include "MicroBit.h"
#include "i2c_api.h"
#include <math.h>
#include "microbit/microbitdal.h"

extern "C" {
#include <errno.h>
#include "microbit/modmicrobit.h"
#include "gpio_api.h"
#include "device.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_delay.h"

#include "lib/ticker.h"
#include "py/runtime0.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/mphal.h"
#include "py/gc.h"
#include "microbit/modaudio.h"
//#include "microbit/microbitobj.h"

#include "serial_api.h"
#include "py/stream.h"

#define DEBUG_BLUEBIT 0
#if DEBUG_BLUEBIT
#include <stdio.h>
#define DEBUG(s) printf s
#else
#define DEBUG(s) (void)0
#endif
// extern MicroPythonI2C ubit_i2c;

#define LCD1602_ADDR (24<<1)
#define ULTRASONIC_ADDR (0x0b<<1)
#define TM1650_BASE_ADDR (0x34<<1)   
#define PCA9554_ADDR (0x20<<1)
#define COLOR_ADDR (0x0a<<1)
#define MIDI_ADDR  (0x0c<<1)
#define SHT20_ADDR (0x40<<1)
#define BH1750_ADDR (0x23<<1)
#define PCA9685_ADDR (0x41<<1)
#define HT16K33_ADDR (0x70<<1)
  
#define HT16K33_BLINK_CMD  0x80
#define HT16K33_BLINK_DISPLAYON  0x01
#define HT16K33_BLINK_OFF  0
#define HT16K33_BLINK_2HZ   1
#define HT16K33_BLINK_1HZ   2
#define HT16K33_BLINK_HALFHZ   3
#define HT16K33_CMD_BRIGHTNESS  0xE0

//define pca9685 register
#define MODE1  0x00
#define MODE2  0x01
#define SUBADR1  0x02
#define SUBADR2  0x03
#define SUBADR3  0x04
#define PRESCALE  0xFE
#define LED0_ON_L  0x06
#define LED0_ON_H  0x07
#define LED0_OFF_L  0x08
#define LED0_OFF_H  0x09
#define ALL_LED_ON_L  0xFA
#define ALL_LED_ON_H  0xFB
#define ALL_LED_OFF_L  0xFC
#define ALL_LED_OFF_H  0xFD
  
#define STP_CHA_L  2047
#define STP_CHA_H  4095
#define STP_CHB_L  1
#define STP_CHB_H  2047
#define STP_CHC_L  1023
#define STP_CHC_H  3071
#define STP_CHD_L  3071
#define STP_CHD_H  1023
   
const char TM1650_CDigits[128] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x82, 0x21, 0x00, 0x00, 0x00, 0x00, 0x02, 0x39, 0x0F, 0x00, 0x00, 0x00, 0x40, 0x80, 0x00, 
  0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7f, 0x6f, 0x00, 0x00, 0x00, 0x48, 0x00, 0x53, 
  0x00, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, 0x6F, 0x76, 0x06, 0x1E, 0x00, 0x38, 0x00, 0x54, 0x3F, 
  0x73, 0x67, 0x50, 0x6D, 0x78, 0x3E, 0x00, 0x00, 0x00, 0x6E, 0x00, 0x39, 0x00, 0x0F, 0x00, 0x08, 
  0x63, 0x5F, 0x7C, 0x58, 0x5E, 0x7B, 0x71, 0x6F, 0x74, 0x02, 0x1E, 0x00, 0x06, 0x00, 0x54, 0x5C, 
  0x73, 0x67, 0x50, 0x6D, 0x78, 0x1C, 0x00, 0x00, 0x00, 0x6E, 0x00, 0x39, 0x30, 0x0F, 0x00, 0x00  
};
  
unsigned char matrix_buf[8];
byte rowOffsets[4] = {0x00, 0x40, 0x10, 0x50};
    
STATIC void mp3WriteCmd(unsigned char *buf_in);
STATIC mp_uint_t bluebit_uart_read(void *buf_in, mp_uint_t size);
STATIC mp_uint_t bluebit_uart_write(const void *buf_in, mp_uint_t size);
STATIC void _pca9685_set_freq(mp_uint_t freq);
STATIC void _pca9685_set_pwm(unsigned char channel,  mp_uint_t on, mp_uint_t off);
STATIC void _matrix_clear(void);

STATIC mp_obj_t mp3_init(void) {
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mp3_init_obj, mp3_init);

STATIC mp_obj_t mp3_play(mp_obj_t music_num) {
  unsigned char val = (unsigned char)mp_obj_get_int(music_num);
	unsigned char cmd[6] = {0xFF,0x06,0x03,0x01,0x00,val};
	mp3WriteCmd(cmd);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mp3_play_obj, mp3_play);

STATIC mp_obj_t mp3_stop(void) {
    unsigned char cmd[6] = {0xff,0x06,0x16,0x00,0x00,0x00};
    mp3WriteCmd(cmd);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mp3_stop_obj, mp3_stop);

STATIC mp_obj_t mp3_volume(mp_obj_t volume) {
    unsigned char val = (unsigned char)mp_obj_get_int(volume);
	unsigned char cmd[6] = {0xFF,0x06,0x06,0x00,0x00,val};    
	mp3WriteCmd(cmd);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mp3_volume_obj, mp3_volume);
    
STATIC mp_obj_t lcd_init(void) {
    char data[2] = {0x01, 0x0c};
    ubit_i2c.write(LCD1602_ADDR, data, 2, false); 
   nrf_delay_ms(2);
    data[1] = 0x01;
    ubit_i2c.write(LCD1602_ADDR, data, 2, false); 
    nrf_delay_ms(2);
    data[1] = 0x06;
    ubit_i2c.write(LCD1602_ADDR, data, 2, false); 
    nrf_delay_ms(2);
    
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(lcd_init_obj, lcd_init);
    
STATIC mp_obj_t lcd_set_curser(mp_obj_t col, mp_obj_t row) {
    char data[2] = {0x01, 0};
    unsigned char _col = (unsigned char)mp_obj_get_int(col); 
    unsigned char _row = (unsigned char)mp_obj_get_int(row);
    data[1] = 0x80 | (_col + rowOffsets[_row]);
    ubit_i2c.write(LCD1602_ADDR, (char *)data, 2, false); 
    nrf_delay_ms(2);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(lcd_set_cursor_obj, lcd_set_curser);
    
STATIC mp_obj_t lcd_print(const mp_obj_t strs) {
    char data[2] = {0x02, 0};
    mp_uint_t len;
    const char* str = mp_obj_str_get_data(strs , &len);
    
    for(mp_uint_t i = 0; i < len; i++)
    {
        data[1] = str[i];
        ubit_i2c.write(LCD1602_ADDR, (char *)data, 2, false);
        nrf_delay_ms(2);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(lcd_print_obj, lcd_print);

STATIC mp_obj_t lcd_clear(void) {
    char data[2] = {0x01, 0x01};
    ubit_i2c.write(LCD1602_ADDR, data, 2, false); //clear display    
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(lcd_clear_obj, lcd_clear);
    
STATIC mp_obj_t lcd_cmd(mp_obj_t cmd) {
    unsigned char val = (unsigned char)mp_obj_get_int(cmd);
    char data[2] = {0x01, val};
    ubit_i2c.write(LCD1602_ADDR, data, 2, false);  
    nrf_delay_ms(2);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(lcd_cmd_obj, lcd_cmd);
      
STATIC mp_obj_t get_distance(void) {
    char data[2] = {1,0};
    mp_int_t distance;
    ubit_i2c.write(ULTRASONIC_ADDR, data, 1, false);
    nrf_delay_ms(2);
    ubit_i2c.read(ULTRASONIC_ADDR, data, 2, false);
    distance = ((int)(data[1]))<<8;
    distance += data[0];
    return mp_obj_new_int(distance);
}
MP_DEFINE_CONST_FUN_OBJ_0(get_distance_obj, get_distance);
    
STATIC mp_obj_t tm1650_init(void) {
    char data = 0x01;
    ubit_i2c.write(0x48, &data, 1, false); 
    nrf_delay_ms(2);
    
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(tm1650_init_obj, tm1650_init);
    
STATIC mp_obj_t tm1650_print(mp_obj_t str) {
    mp_uint_t len;
    char str2[4];
    char dat = 0x3f;
    char j;
	char data;
    const char *str1 = mp_obj_str_get_data(str, &len);
	
	dat = 0;
    for(char i = 0; i < 4; i++){
        ubit_i2c.write(TM1650_BASE_ADDR+i*2, (char *)&data, 1, false); 
        nrf_delay_ms(2);
    }
    
    if(len > 4)
        strncpy(str2, str1, 4);
    else
        strncpy(str2, str1, len);
    for(char i = 0; i < len; i++){
        j = char(str2[i]);
        ubit_i2c.write(TM1650_BASE_ADDR+i*2, (char *)&TM1650_CDigits[j], 1, false);
        nrf_delay_ms(2);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(tm1650_print_obj, tm1650_print);
    
STATIC mp_obj_t tm1650_clear(void) {
    char data = 0;
    for(char i = 0; i < 4; i++){
        ubit_i2c.write(TM1650_BASE_ADDR+i*2, (char *)&data, 1, false); 
        nrf_delay_ms(2);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(tm1650_clear_obj, tm1650_clear);
    
STATIC mp_obj_t get_color(void) {
    char data[6];
    mp_uint_t color[3];
    unsigned char color1[3];
    mp_int_t max_color;
    int err;
    float scale;
    data[0] = 1;
    err = ubit_i2c.write(COLOR_ADDR, (char *)&data, 1, false); 
    if (err != MICROBIT_OK) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }   
    nrf_delay_ms(80);
    data[0] = 2;
    err = ubit_i2c.write(COLOR_ADDR, (char *)&data, 1, false); 

    nrf_delay_ms(5);
    err = ubit_i2c.read(COLOR_ADDR, (char *)data, 1, false);
    if (err != MICROBIT_OK) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
    if(data[0] ==3){
      data[0] = 3;
      err  = ubit_i2c.write(COLOR_ADDR, (char *)&data, 1, false);
      if (err != MICROBIT_OK) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
      }
      nrf_delay_ms(5);
      vstr_t vstr;
      vstr_init_len(&vstr, 3);
      err = ubit_i2c.read(COLOR_ADDR, data, 6, false);
      if (err != MICROBIT_OK) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
      }
      color[0] = ((mp_uint_t(data[5]))<<8) + (mp_uint_t)(data[4]);
      color[1] = ((mp_uint_t(data[1]))<<8) + (mp_uint_t)(data[0]);
      color[2] = ((mp_uint_t(data[3]))<<8) + (mp_uint_t)(data[2]);
      max_color = max(max(color[0],color[1]),color[2]);
      if(max_color > 255){
        scale = 255/max_color;
        color1[0] = (unsigned char)(color[0]*scale);
        color1[1] = (unsigned char)(color[1]*scale);
        color1[2] = (unsigned char)(color[2]*scale);
      }else{
	    color1[0] = (unsigned char)(color[0]);
        color1[1] = (unsigned char)(color[1]);
        color1[2] = (unsigned char)(color[2]);
	  }
      for(mp_uint_t i = 0; i < 3; i++)
        vstr.buf[i] = color[i];
      return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(get_color_obj, get_color);
    
STATIC mp_obj_t extIO_mode(mp_obj_t pin, mp_obj_t mode) {
	int err;
	char _pin = (char)mp_obj_get_int(pin);
    char mod = (char)mp_obj_get_int(mode);
	char mode_old;
	char mode_new;
    char data[2];
	
	data[0] = 3;
    err = ubit_i2c.write(PCA9554_ADDR, (char *)data, 1, false); 
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }
	err = ubit_i2c.read(PCA9554_ADDR, (char *)data, 1, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
	mode_old = data[0];
	if (mod == 1) {
		mode_new = mode_old | (1 << _pin);
	} else if (mod == 0) {
		mode_new = mode_old & (~(1 << _pin));
	}
	data[0] = 3;
	data[1] = mode_new;
    err = ubit_i2c.write(PCA9554_ADDR, (char *)data, 2, false); 
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(extIO_mode_obj, extIO_mode);
    
STATIC mp_obj_t extIO_read(mp_obj_t pin) {
	int err;
    char data[4];
	char _pin = (char)mp_obj_get_int(pin);
    data[0] = 0;
	
    err = ubit_i2c.write(PCA9554_ADDR, (char *)data, 1, false);
	if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }	
	err = ubit_i2c.read(PCA9554_ADDR, (char *)data, 4, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }

    return mp_obj_new_int((data[0] >> _pin) & 0x01);
}
MP_DEFINE_CONST_FUN_OBJ_1(extIO_read_obj, extIO_read);
    
STATIC mp_obj_t extIO_write(mp_obj_t pin, mp_obj_t dat) {
	int err;
	char _pin = (char)mp_obj_get_int(pin);
    char dt = (char)mp_obj_get_int(dat);
	char state_old;
	char state_new;
    char data[2];
	
	data[0] = 1;
    err = ubit_i2c.write(PCA9554_ADDR, (char *)data, 1, false);
	if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }
	err = ubit_i2c.read(PCA9554_ADDR, (char *)data, 3, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }	
	state_old = data[0];
	if (dt == 1){
		state_new = state_old | (1 << _pin);
	} else if (dt == 0) {
		state_new = state_old & (~(1 << _pin));
	}	
	data[0] = 1;
	data[1] = state_new;
    err = ubit_i2c.write(PCA9554_ADDR, (char *)data, 2, false); 
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(extIO_write_obj, extIO_write);
  
STATIC mp_obj_t midi_init(void) {
    unsigned char buf[3];
    buf[0] = 0xb0;
    buf[1] = 0x78;
    buf[2] = 0x00;
    bluebit_uart_write(buf, 3); //all sound off
    nrf_delay_ms(4);
    buf[0] = 0xb0;
    buf[1] = 0x79;
    buf[2] = 0x7f;
    bluebit_uart_write(buf, 3); //reset all controller
    nrf_delay_ms(15); 

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(midi_init_obj, midi_init);
  
STATIC mp_obj_t midi_set_volume(mp_obj_t vol) {
    char volume = (char)mp_obj_get_int(vol);
    unsigned char buf[3] = {0xb0, 0x07, volume};
    bluebit_uart_write(buf, 3); 
    nrf_delay_ms(10);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(midi_set_volume_obj, midi_set_volume);
  
STATIC mp_obj_t midi_set_instrument(mp_obj_t instrument) {
    char ins = (char)mp_obj_get_int(instrument);
    unsigned char buf[2] = {0xc0, ins};
    bluebit_uart_write(buf, 2); 
    nrf_delay_ms(10);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(midi_set_instrument_obj, midi_set_instrument);
  
STATIC mp_obj_t midi_note(mp_obj_t  note, mp_obj_t on_off) {
    char _note = (char)mp_obj_get_int(note);
    char onOff = (char)mp_obj_get_int(on_off);
    unsigned char buf[3] = {0x90, _note, 0x7f};
    if(onOff == 0){
      buf[0] = 0x80;
      buf[2] = 0;
    }      
    bluebit_uart_write(buf, 3);
    nrf_delay_ms(10);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(midi_note_obj, midi_note);
  
STATIC mp_obj_t matrix_init(void) {
    int err;
    char data[2];
    data[0] = 0x21;
    err = ubit_i2c.write(HT16K33_ADDR, (char *)&data, 1, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
    nrf_delay_ms(2);
    data[0] = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON;
    err = ubit_i2c.write(HT16K33_ADDR, (char *)&data, 1, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
    nrf_delay_ms(2);
    data[0] = HT16K33_CMD_BRIGHTNESS | 15;
    err = ubit_i2c.write(HT16K33_ADDR, (char *)&data, 1, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
    nrf_delay_ms(2);
	
   _matrix_clear();
	
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(matrix_init_obj, matrix_init); 
  
STATIC mp_obj_t matrix_print(mp_obj_t str) {
  
    return mp_const_none;
} 
MP_DEFINE_CONST_FUN_OBJ_1(matrix_print_obj, matrix_print); 
  
STATIC mp_obj_t matrix_draw_bmp(mp_obj_t bmp) {
    mp_buffer_info_t bufinfo;
	if(mp_get_buffer(bmp, &bufinfo, MP_BUFFER_READ));
    	memcpy(matrix_buf, bufinfo.buf, bufinfo.len);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(matrix_draw_bmp_obj, matrix_draw_bmp);
  
STATIC mp_obj_t matrix_draw_pixel(mp_obj_t x, mp_obj_t y) {
    unsigned char _x = (unsigned char)mp_obj_get_int(x);
    unsigned char _y = (unsigned char)mp_obj_get_int(y);
    matrix_buf[_y] |= (1 <<_x);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(matrix_draw_pixel_obj, matrix_draw_pixel);
  
STATIC mp_obj_t matrix_clear(void) {	
    _matrix_clear();
	
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(matrix_clear_obj, matrix_clear);
  
STATIC mp_obj_t matrix_show(void) {
    int err;
    unsigned char data[17];
    data[0] = 0;
    for(mp_uint_t i = 0; i < 8; i++)
      data[i*2+1] = matrix_buf[i];
    err = ubit_i2c.write(HT16K33_ADDR, (char *)&data, 17, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
    nrf_delay_ms(2);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(matrix_show_obj, matrix_show);
  
STATIC mp_obj_t get_ntc_temp(mp_obj_t pin) {
    analogin_t obj; 
    float tmp;
    microbit_obj_pin_acquire((microbit_pin_obj_t*)pin, microbit_pin_mode_unused);
	nrf_gpio_cfg_input((PinName)microbit_obj_get_pin_name(pin), NRF_GPIO_PIN_NOPULL);
    analogin_init(&obj, (PinName)microbit_obj_get_pin_name(pin));
    int val = analogin_read_u16(&obj);
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;
    tmp = log((1023.0-val)/val);
    tmp = 298.15*tmp +3935;
    tmp = (1173220.25/tmp - 273.15);
    return mp_obj_new_int((int)(tmp*10));
}
MP_DEFINE_CONST_FUN_OBJ_1(get_ntc_temp_obj, get_ntc_temp);
  
STATIC mp_obj_t get_sht20_temp(void) {
    int err;
    unsigned char data[2];
    data[0] = 0xe3;
    err = ubit_i2c.write(SHT20_ADDR, (char *)&data, 1, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }
    nrf_delay_ms(85);
    err = ubit_i2c.read(SHT20_ADDR, (char *)&data, 2, false); 
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
    float var = (-46.86+175.72*(data[0]*256+data[1])/65535)*10;
     nrf_delay_ms(2);
   return mp_obj_new_int((int)var);
}
MP_DEFINE_CONST_FUN_OBJ_0(get_sht20_temp_obj, get_sht20_temp);
  
STATIC mp_obj_t get_sht20_humi(void) {
    int err;
    unsigned char data[2];
    data[0] = 0xe5;
    err = ubit_i2c.write(SHT20_ADDR, (char *)&data, 1, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }
    nrf_delay_ms(40);
    err = ubit_i2c.read(SHT20_ADDR, (char *)&data, 2, false); 
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
    float var = -6+125*(data[0]*256+data[1])/65535;
    nrf_delay_ms(2);
   return mp_obj_new_int((int)var);
}
MP_DEFINE_CONST_FUN_OBJ_0(get_sht20_humi_obj, get_sht20_humi);
  
STATIC mp_obj_t get_bh1750_light(void) {
    int err;
    unsigned char data[2];
    data[0] = 0x10;
    err = ubit_i2c.write(BH1750_ADDR, (char *)&data, 1, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }
    nrf_delay_ms(120);
    err = ubit_i2c.read(BH1750_ADDR, (char *)&data, 2, false); 
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
    nrf_delay_ms(10);
   return mp_obj_new_int(data[0]*256+data[1]);
}
MP_DEFINE_CONST_FUN_OBJ_0(get_bh1750_light_obj, get_bh1750_light);
  
STATIC mp_obj_t pca9685_init(void) {
    int err;
    unsigned char data[2];
    data[0] = MODE1;
    data[1] = 0x00;
    err = ubit_i2c.write(PCA9685_ADDR, (char *)&data, 2, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }
    nrf_delay_ms(2); 
  
    _pca9685_set_freq(50);
    
    for(mp_uint_t i = 0; i < 16; i++)
      _pca9685_set_pwm(i, 0, 0);
  
   return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(pca9685_init_obj, pca9685_init);
  
STATIC mp_obj_t pca9685_dc_motor(mp_obj_t motor, mp_obj_t speed) {
    mp_int_t  _motor = mp_obj_get_int(motor);
    mp_int_t  _speed = mp_obj_get_int(speed);
    _speed *= 16; //-255-255 map tp -4095-4095
    _speed = (_speed > 4095) ? 4095 : _speed;
    _speed = (_speed < -4095) ? -4095 : _speed;

    if((_motor <= 4) && (_motor > 0)){
      if(_motor == 1)
        _motor = 2;
      else if(_motor == 2)
        _motor = 1;
      else if(_motor == 3)
        _motor = 4;
      else if(_motor == 4)
        _motor = 3;
      _motor = (_motor - 1)*2;
      
      if(_speed >= 0){
        _pca9685_set_pwm(_motor, 0, _speed);
        _pca9685_set_pwm(_motor+1, 0, 0);
      }
      else{
        _pca9685_set_pwm(_motor, 0, 0);
        _pca9685_set_pwm(_motor+1, 0, -_speed); 
      }
    } 
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(pca9685_dc_motor_obj, pca9685_dc_motor);
       
STATIC mp_obj_t pca9685_servo(mp_obj_t servoNum, mp_obj_t degree) {
    unsigned char  _servo = (unsigned char)(mp_obj_get_int(servoNum) + 7);
    mp_uint_t  _degree = mp_uint_t(mp_obj_get_int(degree));
    mp_uint_t value = _degree*10 + 600; //0.6-2.4ms
    value = mp_uint_t(value*4096/20000);
    _pca9685_set_pwm(_servo, 0, value);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(pca9685_servo_obj, pca9685_servo);
       
STATIC mp_obj_t pca9685_step_motor(mp_obj_t stepMT, mp_obj_t dir, mp_obj_t speed) {
      mp_int_t  _stepMT = mp_obj_get_int(stepMT);
      mp_int_t  _dir = mp_obj_get_int(dir);
      mp_uint_t  _speed = mp_uint_t(mp_obj_get_int(speed));
      _speed = (_speed < 25) ? 25 : _speed;
      _speed = (_speed > 200) ? 200 : _speed;

      _pca9685_set_freq(_speed); 
    
      if(_stepMT == 1){
        if(_dir == 1){
            _pca9685_set_pwm(0, STP_CHA_L, STP_CHA_H);
            _pca9685_set_pwm(2, STP_CHB_L, STP_CHB_H);
            _pca9685_set_pwm(1, STP_CHC_L, STP_CHC_H);
            _pca9685_set_pwm(3, STP_CHD_L, STP_CHD_H);         
        }
        else if(_dir == 0){
            _pca9685_set_pwm(3, STP_CHA_L, STP_CHA_H);
            _pca9685_set_pwm(1, STP_CHB_L, STP_CHB_H);
            _pca9685_set_pwm(2, STP_CHC_L, STP_CHC_H);
            _pca9685_set_pwm(0, STP_CHD_L, STP_CHD_H);          
        }
      }
      else if(_stepMT == 2){
        if(_dir == 1){
            _pca9685_set_pwm(4, STP_CHA_L, STP_CHA_H);
            _pca9685_set_pwm(6, STP_CHB_L, STP_CHB_H);
            _pca9685_set_pwm(5, STP_CHC_L, STP_CHC_H);
            _pca9685_set_pwm(7, STP_CHD_L, STP_CHD_H);          
        }
        else if(_dir == 0){
            _pca9685_set_pwm(7, STP_CHA_L, STP_CHA_H);
            _pca9685_set_pwm(5, STP_CHB_L, STP_CHB_H);
            _pca9685_set_pwm(6, STP_CHC_L, STP_CHC_H);
            _pca9685_set_pwm(4, STP_CHD_L, STP_CHD_H);          
        }
      }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(pca9685_step_motor_obj, pca9685_step_motor);
  
STATIC mp_obj_t pca9685_set_pwm(mp_obj_t pwmPin, mp_obj_t pwmVal ) {
    unsigned char  _pwmPin = (unsigned char)(mp_obj_get_int(pwmPin) + 7);
    mp_uint_t  _pwmVal = mp_uint_t(mp_obj_get_int(pwmVal));
    _pca9685_set_pwm(_pwmPin, 0, _pwmVal);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(pca9685_set_pwm_obj, pca9685_set_pwm);
  
STATIC void _pca9685_set_freq(mp_uint_t freq){
    mp_uint_t prescaleval = mp_uint_t(25000000/(4096*freq))-1;
    int err;
    unsigned char data[2];
    unsigned char oldMode;
  
    data[0] = MODE1;
    err = ubit_i2c.write(PCA9685_ADDR, (char *)&data, 1, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    }
    nrf_delay_ms(2);  
  
    err = ubit_i2c.read(PCA9685_ADDR, (char *)&data, 1, false);  //read mode1 reg
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }  
    oldMode = data[0];
    nrf_delay_ms(2);
  
    data[0] = MODE1;
    data[1] = (oldMode & 0x7f) | 0x10; 
    err = ubit_i2c.write(PCA9685_ADDR, (char *)&data, 2, false);  //go to sleep
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    } 
    nrf_delay_ms(2);
  
    data[0] = PRESCALE;
    data[1] = (unsigned char)prescaleval; 
    err = ubit_i2c.write(PCA9685_ADDR, (char *)&data, 2, false); //set new freq
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    } 
    nrf_delay_ms(2);
  
    data[0] = MODE1;
    data[1] = oldMode; 
    err = ubit_i2c.write(PCA9685_ADDR, (char *)&data, 2, false); //exit sleep
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    } 
    nrf_delay_ms(10);
  
    data[0] = MODE1;
    data[1] = oldMode | 0xa1; 
    err = ubit_i2c.write(PCA9685_ADDR, (char *)&data, 2, false); 
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
    } 
    nrf_delay_ms(10);
}
  
STATIC void _pca9685_set_pwm(unsigned char channel,  mp_uint_t on, mp_uint_t off){
    int err;
    unsigned char data[5];
    unsigned char _channel = channel;
    mp_uint_t  _on = on;
    mp_uint_t _off = off;
  
    if((_channel >= 0) && (_channel <= 15)){
      data[0] = LED0_ON_L + 4 * _channel;
      data[1] = _on & 0xff;
      data[2] = (_on >> 8) & 0xff;
      data[3] = _off & 0xff;
      data[4] = (_off >> 8) & 0xff;
      err = ubit_i2c.write(PCA9685_ADDR, (char *)&data, 5, false); 
      if (err != MICROBIT_OK) {
         nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write failed with error code %d", err));
      } 
      nrf_delay_ms(10);
    }      
}

STATIC mp_uint_t bluebit_uart_read(void *buf_in, mp_uint_t size) {
    byte *buf = (byte*)buf_in;

    // make sure we want at least 1 char
    if (size == 0) {
        return 0;
    }

    // check there is at least 1 char available
    if (!mp_hal_stdin_rx_any()) {
        return MP_STREAM_ERROR;
    }

    // read the data
    byte *orig_buf = buf;
    for (;;) {
        *buf++ = mp_hal_stdin_rx_chr();
        if (--size == 0 || !mp_hal_stdin_rx_any()) {
            // return number of bytes read
            return buf - orig_buf;
        }
    }
}

STATIC mp_uint_t bluebit_uart_write(const void *buf_in, mp_uint_t size) {
    const char *buf = (const char*)buf_in;
    mp_hal_stdout_tx_strn(buf, size);
    return size;
}
    
STATIC void mp3WriteCmd(unsigned char *buf_in) {
	unsigned char cmdBuff[10];
	unsigned int sum = 0;
	cmdBuff[0] = 0x7E;
	for(int i = 0; i<6; i++){
		cmdBuff[i+1] = *(buf_in+i);
		sum += *(buf_in+i);
	}
	sum = ((0xFFFF - sum) + 1);
	cmdBuff[7] = (unsigned char)(sum>>8);
	cmdBuff[8] = (unsigned char)(sum & 0xff);
	cmdBuff[9] = 0xEF;
	bluebit_uart_write(cmdBuff, 10);
}

STATIC void _matrix_clear(void){
	int err;
	unsigned char data[17];
	
	for(mp_uint_t i = 0; i < 8; i++)
      matrix_buf[i] = 0;
	
    data[0] = 0;
    for(mp_uint_t i = 0; i < 8; i++)
      data[i*2+1] = matrix_buf[i];
    err = ubit_i2c.write(HT16K33_ADDR, (char *)&data, 17, false);
    if (err != MICROBIT_OK) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read failed with error code %d", err));
    }
    nrf_delay_ms(2);
}
	
STATIC const mp_map_elem_t bluebit_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_bluebit) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mp3_init), (mp_obj_t)&mp3_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mp3_play), (mp_obj_t)&mp3_play_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mp3_stop), (mp_obj_t)&mp3_stop_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mp3_volume), (mp_obj_t)&mp3_volume_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_lcd_init), (mp_obj_t)&lcd_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_lcd_set_cursor), (mp_obj_t)&lcd_set_cursor_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_lcd_print), (mp_obj_t)&lcd_print_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_lcd_clear), (mp_obj_t)&lcd_clear_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_lcd_cmd), (mp_obj_t)&lcd_cmd_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_distance), (mp_obj_t)&get_distance_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_tm1650_init), (mp_obj_t)&tm1650_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_tm1650_print), (mp_obj_t)&tm1650_print_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_tm1650_clear), (mp_obj_t)&tm1650_clear_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_color), (mp_obj_t)&get_color_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_extIO_mode), (mp_obj_t)&extIO_mode_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_extIO_write), (mp_obj_t)&extIO_write_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_extIO_read), (mp_obj_t)&extIO_read_obj },
  
    { MP_OBJ_NEW_QSTR(MP_QSTR_midi_init), (mp_obj_t)&midi_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_midi_set_volume), (mp_obj_t)&midi_set_volume_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_midi_set_instrument), (mp_obj_t)&midi_set_instrument_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_midi_note), (mp_obj_t)&midi_note_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_matrix_init), (mp_obj_t)&matrix_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_matrix_print), (mp_obj_t)&matrix_print_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_matrix_draw_bmp), (mp_obj_t)&matrix_draw_bmp_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_matrix_clear), (mp_obj_t)&matrix_clear_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_matrix_draw_pixel), (mp_obj_t)&matrix_draw_pixel_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_matrix_show), (mp_obj_t)&matrix_show_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_ntc_temp), (mp_obj_t)&get_ntc_temp_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_sht20_temp), (mp_obj_t)&get_sht20_temp_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_sht20_humi), (mp_obj_t)&get_sht20_humi_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_bh1750_light), (mp_obj_t)&get_bh1750_light_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pca9685_init), (mp_obj_t)&pca9685_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pca9685_dc_motor), (mp_obj_t)&pca9685_dc_motor_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pca9685_servo), (mp_obj_t)&pca9685_servo_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pca9685_step_motor), (mp_obj_t)&pca9685_step_motor_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pca9685_set_pwm), (mp_obj_t)&pca9685_set_pwm_obj }, 
};

STATIC MP_DEFINE_CONST_DICT(bluebit_module_globals, bluebit_globals_table);

const mp_obj_module_t bluebit_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&bluebit_module_globals,
};


}


