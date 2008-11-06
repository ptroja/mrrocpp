/* 
    bttv - Bt848 frame grabber driver

    Copyright (C) 1996,97 Ralph Metzler (rjkm@thp.uni-koeln.de)
	Copyright (C) 2002. QNX version Carlos Beltran (cbeltran@dist.unige.it)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef _BTTV_H_
#define _BTTV_H_

#define BTTV_VERSION_CODE 0x000604

#define ushort unsigned short
#define uint unsigned int
#define unchar unsigned char
#define ulong unsigned long

volatile uint8_t	 *regbase8;
volatile uint32_t    *regbase;    /* device has 32-bit registers */


//#include <linux/types.h>
//#include <linux/wait.h>

//#include "i2c.h"
//#include "audiochip.h"
#include "bt848.h"

//#ifdef V4L2
//#include "videodev2.h"
//#else
//#include "videodev.h"
//#endif

#ifndef O_NONCAP  
#define O_NONCAP	O_TRUNC
#endif

#define MAX_GBUFFERS	2
#define RISCMEM_LEN	(32744*2)
#define VBI_MAXLINES    16
//#define VBIBUF_SIZE     (2048*VBI_MAXLINES*2)
//#define VBIBUF_SIZE     600000
#define VBIBUF_SIZE     1310720
typedef unsigned long  u32;
typedef unsigned short u16;


/* maximum needed buffer size for extended VBI frame mode capturing */
/*#define BTTV_MAX_FBUF	0x190000*/
/* maximum buffer size 922x576 with 32bpp for grabbing display mode */
#define BTTV_MAX_FBUF	0x208000

struct bttv_window 
{
	int x, y;
	ushort width, height;
	ushort bpp, bpl;
	ushort swidth, sheight;
	short cropx, cropy;
	ushort cropwidth, cropheight;
	unsigned long vidadr;
	ushort freq;
	int norm;
	int interlace;
	int color_fmt;
	ushort depth;
};

struct bttv_pll_info {
	unsigned int pll_ifreq;	   /* PLL input frequency 	 */
	unsigned int pll_ofreq;	   /* PLL output frequency       */
	unsigned int pll_crystal;  /* Crystal used for input     */
	unsigned int pll_current;  /* Currently programmed ofreq */
};

/*  Per-open data for handling multiple opens on one device */
struct device_open
{
	int	     isopen;
	int	     noncapturing;
	struct bttv  *dev;
};
#define MAX_OPENS 3

struct bttv
{
/*
#ifdef V4L2
	struct v4l2_device video_dev;
	struct v4l2_device radio_dev;
	struct v4l2_device vbi_dev;

	struct v4l2_format	clientfmt;
	struct v4l2_captureparm	capture;

#else
	struct video_device video_dev;
	struct video_device radio_dev;
	struct video_device vbi_dev;
	struct video_picture picture;		
	struct video_audio audio_dev;	
#endif
*/
	int dbx;
  int tuneradr;

	int user;
	int capuser;
	int open_count;
	struct device_open open_data[MAX_OPENS];
	int capturing_opens;

	//struct i2c_bus i2c;
	int have_audiochip;         /* set if there is an I2C chip
	                               connected (like MSP3400) -- does not
	                               conflict with audio_chip */
	int field_count;

	int have_tuner;
        int tuner_type;
        int channel;
        
        unsigned int nr;
	unsigned short id;
#if LINUX_VERSION_CODE < 0x020100
	unsigned char bus;          /* PCI bus the Bt848 is on */
	unsigned char devfn;
#else
	struct pci_dev *dev;
#endif
	unsigned int irq;          /* IRQ used by Bt848 card */
	unsigned char revision;
	unsigned long bt848_adr;      /* bus address of IO mem returned by PCI BIOS */
	unsigned char *bt848_mem;   /* pointer to mapped IO memory */
	unsigned long busriscmem; 
	u32 *riscmem;
  
	unsigned char *vbibuf;
	unsigned char *imagebuf;
	struct bttv_window win;
	int type;            /* card type  */
	int audio;           /* audio mode */
	int audio_chip;      /* set to one of the chips supported by bttv.c */
	int radio;

	u32 *risc_jmp;
	u32 *vbi_odd;
	u32 *vbi_even;
	u32 bus_vbi_even;
	u32 bus_vbi_odd;
    //WAIT_QUEUE vbiq;
	//WAIT_QUEUE capq;
	//WAIT_QUEUE capqo;
	//WAIT_QUEUE capqe;
	int vbip;

	u32 *risc_odd;
	u32 *risc_even;
	int cap;
	struct video_clip *cliprecs;

	int tuner;

	struct gbuffer *ogbuffers;
	struct gbuffer *egbuffers;
	u16 gwidth, gheight, gfmt;
	u16 gwidth_next, gheight_next, gfmt_next;
	u32 *grisc;
	
	unsigned long gro;
	unsigned long gre;
	unsigned long gro_next;
	unsigned long gre_next;

        int grf,grf_next;  /* frame numbers in grab queue */
        int frame_stat[MAX_GBUFFERS];
#define GBUFFER_UNUSED       0
#define GBUFFER_GRABBING     1
#define GBUFFER_DONE         2

        char *fbuffer;
	int gmode;
	int grabbing;
	int lastgrab;
	int grab;
	int grabcount;

	struct bttv_pll_info pll;
	unsigned int Fsc;
	unsigned int field;
	unsigned int last_field; /* number of last grabbed field */
	int i2c_command;
	int triton1;
};
#endif

/*The following should be done in more portable way. It depends on define
  of _ALPHA_BTTV in the Makefile.*/

#if defined(__powerpc__) /* big-endian */
extern __inline__ void io_st_le32(volatile unsigned *addr, unsigned val)
{
        __asm__ __volatile__ ("stwbrx %1,0,%2" : \
                            "=m" (*addr) : "r" (val), "r" (addr));
      __asm__ __volatile__ ("eieio" : : : "memory");
}

/*
#define btwrite(dat,adr)  io_st_le32((unsigned *)(btv->bt848_mem+(adr)),(dat))
#define btread(adr)       ld_le32((unsigned *)(btv->bt848_mem+(adr)))
#else
#ifdef _ALPHA_BTTV
#define btwrite(dat,adr)    writel((dat),(char *) (btv->bt848_adr+(adr)))
#define btread(adr)         readl(btv->bt848_adr+(adr))
#else
#define btwrite(dat,adr)    writel((dat), (char *) (btv->bt848_mem+(adr)))
#define btread(adr)         readl(btv->bt848_mem+(adr))
#endif*/
#endif


#define btwrite(dat,adr)  regbase8[adr] = dat
#define btread(adr)       regbase8[adr]

/*
#define btand(dat,adr)      btwrite((dat) & btread(adr), adr)
#define btor(dat,adr)       btwrite((dat) | btread(adr), adr)
#define btaor(dat,mask,adr) btwrite((dat) | ((mask) & btread(adr)), adr)
*/

#define btand(dat,adr)      regbase8[adr] = dat & regbase8[adr]
#define btor(dat,adr)       regbase8[adr] = dat | regbase8[adr]
#define btaor(dat,mask,adr) regbase8[adr] = dat | ((mask) & regbase8[adr])


/* bttv ioctls */

#define BTTV_READEE		_IOW('v',  BASE_VIDIOCPRIVATE+0, char [256])
#define BTTV_WRITEE		_IOR('v',  BASE_VIDIOCPRIVATE+1, char [256])
#define BTTV_FIELDNR		_IOR('v' , BASE_VIDIOCPRIVATE+2, unsigned int)
#define BTTV_PLLSET		_IOW('v' , BASE_VIDIOCPRIVATE+3, struct bttv_pll_info)
#define BTTV_BURST_ON      	_IOR('v' , BASE_VIDIOCPRIVATE+4, int)
#define BTTV_BURST_OFF     	_IOR('v' , BASE_VIDIOCPRIVATE+5, int)
#define BTTV_VERSION  	        _IOR('v' , BASE_VIDIOCPRIVATE+6, int)
#define BTTV_PICNR		_IOR('v' , BASE_VIDIOCPRIVATE+7, int)


#define BTTV_UNKNOWN       0x00
#define BTTV_MIRO          0x01
#define BTTV_HAUPPAUGE     0x02
#define BTTV_STB           0x03
#define BTTV_INTEL         0x04
#define BTTV_DIAMOND       0x05 
#define BTTV_AVERMEDIA     0x06 
#define BTTV_MATRIX_VISION 0x07 
#define BTTV_FLYVIDEO      0x08
#define BTTV_TURBOTV       0x09
#define BTTV_HAUPPAUGE878  0x0a
#define BTTV_MIROPRO       0x0b
#define BTTV_TVBOOSTAR     0x0c
#define BTTV_WINCAM        0x0d
#define BTTV_MAXI          0x0e
#define BTTV_TERRATV       0x0f
#define BTTV_VHX           0x10
#define BTTV_PXC200        0x11
#define BTTV_AVERMEDIA98   0x12
#define BTTV_FLYVIDEO98    0x13

#define AUDIO_TUNER        0x00
#define AUDIO_RADIO        0x01
#define AUDIO_EXTERN       0x02
#define AUDIO_INTERN       0x03
#define AUDIO_OFF          0x04 
#define AUDIO_ON           0x05
#define AUDIO_MUTE         0x80
#define AUDIO_UNMUTE       0x81

#define TDA9850            0x01
#define TDA9840            0x02
#define TDA8425            0x03
#define TEA6300            0x04

#define I2C_TSA5522        0xc2
#define I2C_TDA9840        0x84
#define I2C_TDA9850        0xb6
#define I2C_TDA8425        0x82
#define I2C_HAUPEE         0xa0
#define I2C_STBEE          0xae
#define I2C_VHX            0xc0
#define I2C_TEA6300        0x80
#define I2C_DPL3518	   0x84

#define TDA9840_SW         0x00
#define TDA9840_LVADJ      0x02
#define TDA9840_STADJ      0x03
#define TDA9840_TEST       0x04

#define TDA9850_CON1       0x04
#define TDA9850_CON2       0x05
#define TDA9850_CON3       0x06
#define TDA9850_CON4       0x07
#define TDA9850_ALI1       0x08
#define TDA9850_ALI2       0x09
#define TDA9850_ALI3       0x0a

#define TDA8425_VL         0x00
#define TDA8425_VR         0x01
#define TDA8425_BA         0x02
#define TDA8425_TR         0x03
#define TDA8425_S1         0x08
 
#define TEA6300_VL         0x00		/* volume control left */
#define TEA6300_VR         0x01		/* volume control right */
#define TEA6300_BA         0x02		/* bass control */
#define TEA6300_TR         0x03		/* treble control */
#define TEA6300_FA         0x04		/* fader control */
#define TEA6300_SW         0x05		/* mute and source switch */
#define BPL_ROMAN 768 //SIODMY YOYEK zmienienienie dlugosci linii z oryginalnej 1024 na dla kostki 768

