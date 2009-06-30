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

#include <stdio.h>
#include <stdlib.h>
#include <hw/pci.h>
#include <errno.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include <string.h>

#include "bttv.h"
#define BTTV_MAX 4

static int bttv_num;
static struct bttv bttvs[BTTV_MAX];
static resmgr_connect_funcs_t    connect_funcs;
static resmgr_io_funcs_t         io_funcs;
static iofunc_attr_t             attr;


int irq_debug = 1;

/********************************************************************************
	The read function.
*********************************************************************************/
int
io_read (resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb)
{
    int         nleft;
    int         nbytes;
    int         nparts;
    int         status;
	int i;

	struct bttv * btv = &bttvs[0];

    if ((status = iofunc_read_verify (ctp, msg, ocb, NULL)) != EOK)
        return (status);
        
    if (msg->i.xtype & _IO_XTYPE_MASK != _IO_XTYPE_NONE)
        return (ENOSYS);

    /*
     *  on all reads (first and subsequent) calculate
     *  how many bytes we can return to the client,
     *  based upon the number of bytes available (nleft)
     *  and the client's buffer size
     */

    nleft = ocb->attr->nbytes - ocb->offset;
    nbytes = min (msg->i.nbytes, nleft);

    if (nbytes > 0) {

		SETIOV(ctp->iov,btv->imagebuf+ ocb->offset,nbytes);

        /* set up the number of bytes (returned by client's read()) */
        _IO_SET_READ_NBYTES (ctp, nbytes);

        /*
         * advance the offset by the number of bytes
         * returned to the client.
         */

        ocb->offset += nbytes;
        
        nparts = 1;
    } else {
        /*
         * they've asked for zero bytes or they've already previously
         * read everything
         */
        
        _IO_SET_READ_NBYTES (ctp, 0);
        
        nparts = 0;
    }

    /* mark the access time as invalid (we just accessed it) */

    if (msg->i.nbytes > 0)
        ocb->attr->flags |= IOFUNC_ATTR_ATIME;

    return (_RESMGR_NPARTS (nparts));
}

/********************************************************************************
	Tvnorms
*********************************************************************************/

struct tvnorm 
{
    unsigned long Fsc; //u32
    unsigned int swidth, sheight; /* scaled standard width, height */
	unsigned int totalwidth; //u16
	unsigned char adelay, bdelay, iform; //u8
	unsigned long scaledtwidth; //u32
	unsigned int hdelayx1, hactivex1; //16
	unsigned int vdelay; //u16
    unsigned char vbipack; //u8
};

/********************************************************************************
	Tvnorms array
*********************************************************************************/

static struct tvnorm tvnorms[] = {
	/* PAL-BDGHI */
        /* max. active video is actually 922, but 924 is divisible by 4 and 3! */
 	/* actually, max active PAL with HSCALE=0 is 948, NTSC is 768 - nil */
#ifdef VIDEODAT
        { 35468950,
          924, 576, 1135, 0x7f, 0x72, (BT848_IFORM_PAL_BDGHI|BT848_IFORM_XT1),
          1135, 186, 924, 0x20, 255},
#else
        { 35468950,
          924, 576, 1135, 0x7f, 0x72, (BT848_IFORM_PAL_BDGHI|BT848_IFORM_XT1),
          1135, 186, 924, 0x20, 255},
#endif
/*
        { 35468950, 
          768, 576, 1135, 0x7f, 0x72, (BT848_IFORM_PAL_BDGHI|BT848_IFORM_XT1),
	  944, 186, 922, 0x20, 255},
*/
	/* NTSC */
	{ 28636363,
          768, 480,  910, 0x68, 0x5d, (BT848_IFORM_NTSC|BT848_IFORM_XT0),
          910, 128, 910, 0x1a, 144},
/*
	{ 28636363,
          640, 480,  910, 0x68, 0x5d, (BT848_IFORM_NTSC|BT848_IFORM_XT0),
          780, 122, 754, 0x1a, 144},
*/
#if 0
	/* SECAM EAST */
	{ 35468950, 
          768, 576, 1135, 0x7f, 0xb0, (BT848_IFORM_SECAM|BT848_IFORM_XT1),
	  944, 186, 922, 0x20, 255},
#else
	/* SECAM L */
        { 35468950,
          924, 576, 1135, 0x7f, 0xb0, (BT848_IFORM_SECAM|BT848_IFORM_XT1),
          1135, 186, 922, 0x20, 255},
#endif
        /* PAL-NC */
        { 28636363,
          640, 576,  910, 0x68, 0x5d, (BT848_IFORM_PAL_NC|BT848_IFORM_XT0),
          780, 130, 734, 0x1a, 144},
	/* PAL-M */
	{ 28636363, 
          640, 480, 910, 0x68, 0x5d, (BT848_IFORM_PAL_M|BT848_IFORM_XT0),
	  780, 135, 754, 0x1a, 144},
	/* PAL-N */
	{ 35468950, 
          768, 576, 1135, 0x7f, 0x72, (BT848_IFORM_PAL_N|BT848_IFORM_XT1),
	  944, 186, 922, 0x20, 144},
	/* NTSC-Japan */
	{ 28636363,
          640, 480,  910, 0x68, 0x5d, (BT848_IFORM_NTSC_J|BT848_IFORM_XT0),
	  780, 135, 754, 0x16, 144},
};
#define TVNORMS (sizeof(tvnorms)/sizeof(tvnorm))

/********************************************************************************
	Figure out card and tuner type 
*********************************************************************************/
static void
idcard(struct bttv *btv)
{
  int i;

  btwrite(0, BT848_GPIO_OUT_EN);
  printf("bttv: GPIO: 0x%08x\n", btread(BT848_GPIO_DATA));

  btv->type=BTTV_MIRO;
  //btv->tuner=TUNER_DEFAULT;
  btv->tuner = 5;
  

  /* How do I detect the tuner type for other cards but Miro ??? */
  printf("bttvx: model: ");
  switch (btv->type) {
  case BTTV_MIRO:
    btv->tuner=((btread(BT848_GPIO_DATA)>>10)-1)&7;
    printf("MIRO");
    break;
  case BTTV_HAUPPAUGE:
    printf("HAUPPAUGE");
    break;
  case BTTV_STB: 
    printf("STB");
    break;
  case BTTV_INTEL: 
    printf("Intel");
    break;
  case BTTV_DIAMOND: 
    printf("Diamond");
    break;
  }
  //printf(" (%s @ 0x%02x)\n", tuners[btv->tuner].name, btv->tuneradr);
  //audio(btv, AUDIO_MUTE);
}


/********************************************************************************
	Bright
*********************************************************************************/
static inline void
bt848_bright(struct bttv *btv, uint bright)
{
  btwrite(bright&0xff, BT848_BRIGHT);
}

/********************************************************************************
	Adapting the "virt_to_bus" Linux function into QNX
*********************************************************************************/
u32 virt_to_bus ( u32 * addr )
{
	off_t offset;
	mem_offset(addr, NOFD, 1, &offset, 0);
	return((u32)offset);
	
}

/********************************************************************************
	One of the RISC tabs, This should be done better
*********************************************************************************/
static int 
make_iov_risctab(struct bttv *btv)
{
  ulong i;
  ulong bpsl;
  dword iovp;
  ulong iovl, bl;
  dword **rp;
  dword *ro = btv->risc_odd;
  dword *re = btv->risc_even;

  bpsl=btv->win.width*btv->win.bpp;
  //bpsl = 1;

  //*(ro++)=BT848_RISC_SYNC|BT848_FIFO_STATUS_FM1|(14<<20); *(ro++)=0;
  *(ro++)=BT848_RISC_SYNC|BT848_FIFO_STATUS_FM1; *(ro++)=0;

  *(re++)=BT848_RISC_SYNC|BT848_FIFO_STATUS_FM1; *(re++)=0;
  iovl=600000;
  iovp=(dword)virt_to_bus(btv->imagebuf);
  
  for (i=0; i < btv->win.height; i++) {
    if (i&1)
      rp=&re;
    else
      rp=&ro;

    if (bpsl<=iovl) 
	{
      //*((*rp)++)=BT848_RISC_WRITE|BT848_RISC_SOL|BT848_RISC_EOL|bpsl; 
	  *((*rp)++)=BT848_RISC_WRITE|BT848_RISC_EOL|bpsl; 
      *((*rp)++)=iovp;
      iovp+=bpsl; iovl-=bpsl;
      if (!iovl) 
	  {
	//iov++;
	//iovl=iov->iov_len;
	//iovp=(dword) iov->iov_base;
	  }
    } else {
      *((*rp)++)=BT848_RISC_WRITE|BT848_RISC_SOL|iovl;
      *((*rp)++)=iovp;
      bl=bpsl-iovl;
      //iov++;
      while (bl>600000) {
	*((*rp)++)=BT848_RISC_WRITE|400000;
	//*((*rp)++)=(dword) btv->vbibuf;
	*((*rp)++)=(dword) iovp;
	bl-=600000;
	//iov++;
      }
      *((*rp)++)=BT848_RISC_WRITE|BT848_RISC_EOL|bl;
      //*((*rp)++)=(dword) btv->vbibuf;
	  *((*rp)++)=(dword) iovp;
      iovl=600000-bl; iovp=bl+(dword)btv->vbibuf;    
    }
  }

  *(ro++)=BT848_RISC_JUMP;
  *(ro++)=btv->bus_vbi_even;
  //*(re++)=BT848_RISC_JUMP|BT848_RISC_IRQ|(2<<16);
  *(re++)=BT848_RISC_JUMP;
  *(re++)=btv->bus_vbi_odd;

  return 0;
}

/********************************************************************************
	The other RISC tab.
*********************************************************************************/
static int 
make_rawrisctab(struct bttv *btv, unsigned int *ro,
                            unsigned int *re, unsigned int *vbuf)
{
    unsigned long line;
	//unsigned long bpl=1024;		/* bytes per line */
	unsigned long bpl = BPL_ROMAN*btv->win.bpp; //1024
	unsigned long vadr=(unsigned long) vbuf;

	*(ro++)=BT848_RISC_SYNC|BT848_FIFO_STATUS_FM1|BT848_RISC_RESYNC; 
        *(ro++)=0;
	*(re++)=BT848_RISC_SYNC|BT848_FIFO_STATUS_FM1|BT848_RISC_RESYNC;
        *(re++)=0;
  
        /* In PAL 650 blocks of 256 DWORDs are sampled, but only if VDELAY
           is 2 and without separate VBI grabbing.
           We'll have to handle this inside the IRQ handler ... */

	for (line=0; line < 625; line++)
	{
                
                *(re++)=BT848_RISC_WRITE|bpl|BT848_RISC_SOL|BT848_RISC_EOL;
                *(re++)=virt_to_bus(vadr);
				*(ro++)=BT848_RISC_WRITE|bpl|BT848_RISC_SOL|BT848_RISC_EOL;
                *(ro++)=virt_to_bus(vadr)+bpl;
                vadr+= (bpl*2);
	}
	
	*(ro++)=BT848_RISC_JUMP;
	//*(ro++)=btv->bus_vbi_even;
	*(ro++)=virt_to_bus(btv->risc_even);

	//*(re++)=BT848_RISC_JUMP|BT848_RISC_IRQ|(2<<16);
	*(re++)=BT848_RISC_JUMP;
	//*(re++)=btv->bus_vbi_odd;
	*(re++)=virt_to_bus(btv->risc_odd);
	
	return 0;
}

/********************************************************************************
	Hue
*********************************************************************************/
static inline void
bt848_hue(struct bttv *btv, uint hue)
{
  btwrite(hue&0xff, BT848_HUE);
}

/********************************************************************************
	Contrast
*********************************************************************************/
static inline void
bt848_contrast(struct bttv *btv, uint cont)
{
  uint conthi;

  conthi=(cont>>6)&4;
  btwrite(cont&0xff, BT848_CONTRAST_LO);
  btaor(conthi, ~4, BT848_E_CONTROL);
  btaor(conthi, ~4, BT848_O_CONTROL);
}

/********************************************************************************
	Sat u
*********************************************************************************/
static inline void
bt848_sat_u(struct bttv *btv, ulong data)
{
  ulong datahi;

  datahi=(data>>7)&2;
  btwrite(data&0xff, BT848_SAT_U_LO);
  btaor(datahi, ~2, BT848_E_CONTROL);
  btaor(datahi, ~2, BT848_O_CONTROL);
}

/********************************************************************************
	Sat_v
*********************************************************************************/
static inline void
bt848_sat_v(struct bttv *btv, ulong data)
{
  ulong datahi;

  datahi=(data>>8)&1;
  btwrite(data&0xff, BT848_SAT_V_LO);
  btaor(datahi, ~1, BT848_E_CONTROL);
  btaor(datahi, ~1, BT848_O_CONTROL);
}


/* If Bt848a or Bt849, use PLL for PAL/SECAM and crystal for NTSC*/

/* Frequency = (F_input / PLL_X) * PLL_I.PLL_F/PLL_C 
   PLL_X = Reference pre-divider (0=1, 1=2) 
   PLL_C = Post divider (0=6, 1=4)
   PLL_I = Integer input 
   PLL_F = Fractional input 
   
   F_input = 28.636363 MHz: 
   PAL (CLKx2 = 35.46895 MHz): PLL_X = 1, PLL_I = 0x0E, PLL_F = 0xDCF9, PLL_C = 0
*/

static void set_pll_freq(struct bttv *btv, unsigned int fin, unsigned int fout)
{
        unsigned char fl, fh, fi;
        
        /* prevent overflows */
        fin/=4;
        fout/=4;

        fout*=12;
        fi=fout/fin;

        fout=(fout%fin)*256;
        fh=fout/fin;

        fout=(fout%fin)*256;
        fl=fout/fin;

        /*printk("0x%02x 0x%02x 0x%02x\n", fi, fh, fl);*/
        btwrite(fl, BT848_PLL_F_LO);
        btwrite(fh, BT848_PLL_F_HI);
        btwrite(fi|BT848_PLL_X, BT848_PLL_XCI);
}

/********************************************************************************
	Set_pll
*********************************************************************************/
static int set_pll(struct bttv *btv)
{
    int i;
	unsigned long tv;

        if (!btv->pll.pll_crystal)
                return 0;

        if (btv->pll.pll_ifreq == btv->pll.pll_ofreq) {
                /* no PLL needed */
                if (btv->pll.pll_current == 0) {
                        /* printk ("bttv%d: PLL: is off\n",btv->nr); */
                        return 0;
                }
                printf ("bttv%d: PLL: switching off\n",btv->nr);
                btwrite(0x00,BT848_TGCTRL);
                btwrite(0x00,BT848_PLL_XCI);
                btv->pll.pll_current = 0;
                return 0;
        }

        if (btv->pll.pll_ofreq == btv->pll.pll_current) {
                /* printk("bttv%d: PLL: no change required\n",btv->nr); */
                return 1;
        }
        
        printf("bttv%d: PLL: %d => %d ... ",btv->nr,
               btv->pll.pll_ifreq, btv->pll.pll_ofreq);

	set_pll_freq(btv, btv->pll.pll_ifreq, btv->pll.pll_ofreq);

    for (i=0; i<100; i++) 
    {
         if ((regbase[0]&BT848_DSTATUS_PLOCK))
            regbase[0] = 0;
         else
         {
            btwrite(0x08,BT848_TGCTRL);
            btv->pll.pll_current = btv->pll.pll_ofreq;
            printf("ok\n");
            return 1;
         }
         delay(10);
    }
    btv->pll.pll_current = 0;
    printf("oops\n");
    return -1;
}


/********************************************************************************
Lets try to put the size
*********************************************************************************/

static void 
bt848_set_size(struct bttv *btv)
{
  ushort vscale, hscale;
  ulong xsf, sr;
  ushort hdelay, vdelay;
  ushort hactive, vactive;
  ushort inter;
  unchar crop;
  struct tvnorm *tvn;

  if (!btv->win.width)
    return;
  if (!btv->win.height)
    return;

  tvn=&tvnorms[btv->win.norm];

  btv->pll.pll_ofreq = tvn->Fsc;
  set_pll(btv);
    
  inter=(btv->win.interlace&1)^1;

  switch (btv->win.bpp) {
  case 1: 
    //btwrite(BT848_COLOR_FMT_RGB8, BT848_COLOR_FMT);
	btwrite(BT848_COLOR_FMT_Y8, BT848_COLOR_FMT);
	
    break;
  case 2: 
    btwrite(BT848_COLOR_FMT_RGB16, BT848_COLOR_FMT);
    break;
  case 3: 
    btwrite(BT848_COLOR_FMT_RGB24, BT848_COLOR_FMT);
    break;
  case 4: 
    btwrite(BT848_COLOR_FMT_RGB32, BT848_COLOR_FMT);
    break;
  }

  hactive=btv->win.width;
  if (hactive < 193) {
    btwrite (2, BT848_E_VTC);
    btwrite (2, BT848_O_VTC);
  } else if (hactive < 385) {
    btwrite (1, BT848_E_VTC);
    btwrite (1, BT848_O_VTC);
  } else {
    btwrite (0, BT848_E_VTC);
    btwrite (0, BT848_O_VTC);
  }

  if (btv->win.norm==1) { 
    btv->win.cropwidth=640;
    btv->win.cropheight=480;
    btwrite(0x68, BT848_ADELAY);
    btwrite(0x5d, BT848_BDELAY);
    btaor(BT848_IFORM_NTSC, ~7, BT848_IFORM);
    btaor(BT848_IFORM_XT0, ~0x18, BT848_IFORM);
    xsf = (btv->win.width*365625UL)/300000UL;
    hscale = ((910UL*4096UL)/xsf-4096);
    vdelay=btv->win.cropy+0x16;
    hdelay=((hactive*135)/754+btv->win.cropx)&0x3fe;
  } else {
    btv->win.cropwidth=768;
    btv->win.cropheight=576;
    if (btv->win.norm==0) { 
      btwrite(0x7f, BT848_ADELAY);
      btwrite(0x72, BT848_BDELAY);
      btaor(BT848_IFORM_PAL_BDGHI, ~BT848_IFORM_NORM, BT848_IFORM);
    } else {
      btwrite(0x7f, BT848_ADELAY);
      btwrite(0x00, BT848_BDELAY);
      btaor(BT848_IFORM_SECAM, ~BT848_IFORM_NORM, BT848_IFORM);
    }
    btaor(BT848_IFORM_XT1, ~0x18, BT848_IFORM);
    xsf = (btv->win.width*36875UL)/30000UL;
    hscale = ((1135UL*4096UL)/xsf-4096);
    vdelay=btv->win.cropy+0x20;
    hdelay=((hactive*186)/922+btv->win.cropx)&0x3fe;
  }
    

  sr=((btv->win.cropheight>>inter)*512)/btv->win.height-512;
  vscale=(0x10000UL-sr)&0x1fff;
  vactive=btv->win.cropheight;

  if (btv->win.interlace) 
  { 
    btor(BT848_VSCALE_INT, BT848_E_VSCALE_HI);
    btor(BT848_VSCALE_INT, BT848_O_VSCALE_HI);
  } else 
  {
    btand(~BT848_VSCALE_INT, BT848_E_VSCALE_HI);
    btand(~BT848_VSCALE_INT, BT848_O_VSCALE_HI);
  }

  btwrite(hscale>>8, BT848_E_HSCALE_HI);
  btwrite(hscale>>8, BT848_O_HSCALE_HI);
  btwrite(hscale&0xff, BT848_E_HSCALE_LO);
  btwrite(hscale&0xff, BT848_O_HSCALE_LO);

  btwrite((vscale>>8)|(btread(BT848_E_VSCALE_HI)&0xe0), BT848_E_VSCALE_HI);
  btwrite((vscale>>8)|(btread(BT848_O_VSCALE_HI)&0xe0), BT848_O_VSCALE_HI);
  btwrite(vscale&0xff, BT848_E_VSCALE_LO);
  btwrite(vscale&0xff, BT848_O_VSCALE_LO);

  btwrite(hactive&0xff, BT848_E_HACTIVE_LO);
  btwrite(hactive&0xff, BT848_O_HACTIVE_LO);
  btwrite(hdelay&0xff, BT848_E_HDELAY_LO);
  btwrite(hdelay&0xff, BT848_O_HDELAY_LO);

  btwrite(vactive&0xff, BT848_E_VACTIVE_LO);
  btwrite(vactive&0xff, BT848_O_VACTIVE_LO);
  btwrite(vdelay&0xff, BT848_E_VDELAY_LO);
  btwrite(vdelay&0xff, BT848_O_VDELAY_LO);

  crop=((hactive>>8)&0x03)|((hdelay>>6)&0x0c)|
    ((vactive>>4)&0x30)|((vdelay>>2)&0xc0);
  btwrite(crop, BT848_E_CROP);
  btwrite(crop, BT848_O_CROP);
}

/**************************************************************************
	Interrupt handler.
	This is not working. When I program the interrupt I block QNX totally.
***************************************************************************/

const struct sigevent *
isr_handler (void *arg, int id)
{
    // look at the hardware to see if it caused the interrupt
    // if not, simply return (NULL);
    
    // in a level-sensitive environment, clear the cause of
    // the interrupt, or at least issue InterruptMask to
    // disable the PIC from reinterrupting the kernel
    
    // return a pointer to an event structure (preinitialized
    // by main) that contains SIGEV_INTR as its notification type.
    // This causes the InterruptWait in "int_thread" to unblock.


	u32 stat,astat;
	u32 dstat;
	int count;
	//struct bttv *btv;
	//btv=(struct bttv *)dev_id;
	struct bttv * btv = &bttvs[0];
	count=0;
	while (1) 
	{
		/* get/clear interrupt status bits */
		stat=btread(BT848_INT_STAT);
		astat=stat&btread(BT848_INT_MASK);
		if (!astat)
			return;
		btwrite(stat,BT848_INT_STAT);

		/* get device status bits */
		dstat=btread(BT848_DSTATUS);

		if (irq_debug) {
			printf("bttv%d: irq loop=%d fc=%d "
			       "riscs=%x, riscc=%08x, ",
			       btv->nr, count, btv->field_count,
			       stat>>28, btread(BT848_RISC_COUNT));
			//bttv_print_irqbits(stat,astat);
			if (stat & BT848_INT_HLOCK)
				printf("   HLOC => %s", (dstat & BT848_DSTATUS_HLOC)
				       ? "yes" : "no");
			if (stat & BT848_INT_VPRES)
				printf("   PRES => %s", (dstat & BT848_DSTATUS_PRES)
				       ? "yes" : "no");
			if (stat & BT848_INT_FMTCHG)
				printf("   NUML => %s", (dstat & BT848_DSTATUS_PRES)
				       ? "625" : "525");
			printf("\n");
		}

		if (astat&BT848_INT_VSYNC) 
                        btv->field_count++;

		/*
		if (astat & BT848_INT_GPINT)
			wake_up(&btv->gpioq);
		
                if ((astat & BT848_INT_RISCI)  &&  (stat & (1<<28)))
			bttv_irq_switch_fields(btv);

		if ((astat & BT848_INT_HLOCK)  &&  btv->opt_automute) {
			if ((dstat & BT848_DSTATUS_HLOC) || btv->radio_user)
				audio_mux(btv, AUDIO_UNMUTE);
			else
				audio_mux(btv, AUDIO_MUTE);
		}

		if (astat & (BT848_INT_SCERR|BT848_INT_OCERR)) {
			printk(KERN_INFO "bttv%d: %s%s @ %08x,",btv->nr,
			       (astat & BT848_INT_SCERR) ? "SCERR" : "",
			       (astat & BT848_INT_OCERR) ? "OCERR" : "",
			       btread(BT848_RISC_COUNT));
			bttv_print_irqbits(stat,astat);
			printk("\n");
			if (bttv_debug)
				bttv_print_riscaddr(btv);
		}
		if (fdsr && astat & BT848_INT_FDSR) {
			printk(KERN_INFO "bttv%d: FDSR @ %08x\n",
			       btv->nr,btread(BT848_RISC_COUNT));
			if (bttv_debug)
				bttv_print_riscaddr(btv);
		}*/

		count++;
		if (count > 20) {
			btwrite(0, BT848_INT_MASK);
			printf("bttv%d: IRQ lockup, cleared int mask\n", btv->nr);
		}
	}

	printf("Interrupcion\n");
	fflush(NULL);

    return (0);

}

/********************************************************************************
	Find the bt848.
	Does it exist in the machine?
*********************************************************************************/
int 
find_bt848()
{
	short pci_index;
    struct pci_dev_info info;
    void            *hdl;
    int         i;
	struct bttv * btv;
	uint32_t command, latency;
	uint32_t command2;
	uint32_t reg_adress;
	int id;

	btv = &bttvs[0];

	btv->bt848_mem=NULL;
	btv->riscmem=NULL;

    memset(&info, 0, sizeof (info));

    if (pci_attach(0) < 0) {
        perror("pci_attach");
        exit(EXIT_FAILURE);
    }

    /*
     * Fill in the Vendor and Device ID for a 3dfx VooDoo3
     * graphics adapter.
     */
    info.VendorId = 0x109e;
    info.DeviceId = 0x36e;

	hdl = pci_attach_device(0,
							PCI_SHARE|PCI_INIT_ALL, 
							0, 
							&info); 

	if (hdl == NULL)
		return 0;

	/* Enable bus mastering*/
	pci_read_config32(info.BusNumber,
					 info.DevFunc,
					 PCI_COMMAND,
					 1,
					 &command);

	command |= PCI_COMMAND_MASTER;
	command |= 0x00000002;

	pci_write_config32(info.BusNumber,
					  info.DevFunc,
					  PCI_COMMAND,
					  1,
					  &command);

	pci_read_config32(info.BusNumber,
					  info.DevFunc,
					  0x04,
					  1,
					  &command2);

	pci_read_config32(info.BusNumber,
					  info.DevFunc,
					  0x10,
					  1,
					  &reg_adress);

	reg_adress &= PCI_BASE_ADDRESS_MEM_MASK;

	pci_read_config32(info.BusNumber,
					  info.DevFunc,
					  PCI_LATENCY_TIMER,
					  1,
					  &latency);

	if (!latency) {
      latency=32;
	  pci_write_config32(info.BusNumber,
						info.DevFunc,
						PCI_LATENCY_TIMER,
						1,
						&latency);
    }

	printf("bttvx: Vendor and device ID  0x%llx\n",command2);
	printf("bttvx: REG ADRESS = 0x%llx\n",reg_adress);

    if (hdl == 0) 
	{
        perror("pci_attach_device");
        exit(EXIT_FAILURE);
    }

    for (i = 0; i < 4; i++) {
        if (info.BaseAddressSize[i] > 0)
            printf("bttvx: Aperture %d: "
                "Base 0x%llx Length %d bytes Type %s\n", i,
                PCI_IS_MEM(info.CpuBaseAddress[i]) ?
                PCI_MEM_ADDR(info.CpuBaseAddress[i]) :
                PCI_IO_ADDR(info.CpuBaseAddress[i]),
                info.BaseAddressSize[i],
                PCI_IS_MEM(info.CpuBaseAddress[i]) ? "MEM" : "IO");
    }

    printf("bttvx: IRQ 0x%x\n", info.Irq);

	/* Try to map registers */
	btv->vbip = VBIBUF_SIZE;
	btv->bt848_adr = info.CpuBaseAddress[0];
	btv->irq = info.Irq;

    regbase = (uint32_t *) mmap_device_memory(NULL, 
								 info.BaseAddressSize[0],
								 PROT_READ|PROT_WRITE|PROT_NOCACHE, 
								 MAP_TYPE,
								 PCI_MEM_ADDR(info.CpuBaseAddress[0]));
								 //reg_adress);

	regbase8 = (uint8_t *) mmap_device_memory(NULL, 
								 info.BaseAddressSize[0],
								 PROT_READ|PROT_WRITE|PROT_NOCACHE, 
								 MAP_TYPE,
								 PCI_MEM_ADDR(info.CpuBaseAddress[0]));
								 //reg_adress);

	//Interrupt mask register = 0 (INT_MASK)
	regbase[65]=0;

	ThreadCtl( _NTO_TCTL_IO, 0 );
	id = InterruptAttach (info.Irq, isr_handler, NULL, 0, 0);

	//did the map work?
	if(regbase == MAP_FAILED)
		printf("bttvx: The map failed: %s\n", strerror(errno));
	else
		printf("bttvx: The map Suceeded: %s\n", strerror(errno));

	btv->bt848_mem = (unchar *) regbase8;

	//PLL programming. For the moment only PAL.
	btv->pll.pll_crystal = 0;
    btv->pll.pll_ifreq   = 0;
    btv->pll.pll_ofreq   = 0;
    btv->pll.pll_current = 0;
    
	if (btv->win.norm==1) 
	{ 
       /* 35 MHz crystal installed */
      btv->pll.pll_ifreq=35468950;
      btv->pll.pll_crystal=BT848_IFORM_XT1;
    }
	else
	{
	  /* 28 MHz crystal installed */
      btv->pll.pll_ifreq=28636363;
	  //btv->pll.pll_ifreq=28618000;
      btv->pll.pll_crystal=BT848_IFORM_XT0;       
    }
	
    //pci_detach_device(hdl);
}

/********************************************************************************
	Activa DMA register
*********************************************************************************/
inline static void
bt848_dma(struct bttv *btv, uint state)
{
  if (state)
    btor(3, BT848_GPIO_DMA_CTL);
  else
    btand(~3, BT848_GPIO_DMA_CTL);
}

/***************************************************************************
  Program all the RISC jumping instructions.
****************************************************************************/
static void
bt848_set_risc_jmps(struct bttv *btv)
{
  int flags=btv->cap;
  
  btv->risc_jmp[0]=BT848_RISC_SYNC|BT848_RISC_RESYNC|BT848_FIFO_STATUS_VRE;
  btv->risc_jmp[1]=0;

  btv->risc_jmp[2]=BT848_RISC_JUMP;
  if (flags&8)
    btv->risc_jmp[3]=virt_to_bus(btv->vbi_odd);
  else
    btv->risc_jmp[3]=virt_to_bus(btv->risc_jmp+4);

  btv->risc_jmp[4]=BT848_RISC_JUMP;
  if (flags&2)
    btv->risc_jmp[5]=virt_to_bus(btv->risc_odd);
  else
    btv->risc_jmp[5]=virt_to_bus(btv->risc_jmp+6);

  btv->risc_jmp[6]=BT848_RISC_SYNC|BT848_RISC_RESYNC|BT848_FIFO_STATUS_VRO;
  btv->risc_jmp[7]=0;

  btv->risc_jmp[8]=BT848_RISC_JUMP;
  if (flags&4)
    btv->risc_jmp[9]=virt_to_bus(btv->vbi_even);
  else
    btv->risc_jmp[9]=virt_to_bus(btv->risc_jmp+10);

  btv->risc_jmp[10]=BT848_RISC_JUMP;
  if (flags&1)
    btv->risc_jmp[11]=virt_to_bus(btv->risc_even);
  else
    btv->risc_jmp[11]=virt_to_bus(btv->risc_jmp);

  btaor(flags, ~0x0f, BT848_CAP_CTL);
  if (flags&0x0f)
    bt848_dma(btv, 3);
  else
    bt848_dma(btv, 0);
}

/*************************************************************************
	Activate capture
**************************************************************************/
static void 
bt848_cap(struct bttv * btv,int state)
{

if (state) 
	{
		btv->cap|=3;
		bt848_set_risc_jmps(btv);
	}
	else
	{
		btv->cap&=~3;
		bt848_set_risc_jmps(btv);
	}
}

/********************************************************************************
  Open
*********************************************************************************/
int
io_open(resmgr_context_t *ctp, io_open_t *msg,RESMGR_HANDLE_T *handle, void *extra)
{
	struct bttv *btv;
	btv = &bttvs[0];

	btv->vbip = VBIBUF_SIZE;
	btv->cap |= 0x0c;
	bt848_set_risc_jmps(btv);
	bt848_cap(btv, 1); //activa capture
	return (iofunc_open_default (ctp, msg, handle, extra));
}

/***************************************************************************
  Vbitab programming. I am not using it really.
****************************************************************************/
#define VBI_SPL 2044
/* RISC command to write one VBI data line */
#define VBI_RISC BT848_RISC_WRITE|VBI_SPL|BT848_RISC_EOL|BT848_RISC_SOL

static void
make_vbitab(struct bttv *btv)
{

  int i;
  off_t offset;
  u32 *po;
  u32 *pe;

  //mem_offset(btv->vbi_odd, NOFD, 1, &offset, 0);
  //po=(u32 *)PCI_MEM_ADDR(offset);
  po = btv->vbi_odd;
  //mem_offset(btv->vbi_even, NOFD, 1, &offset, 0);
  //pe=(u32 *)PCI_MEM_ADDR(offset);
  pe = btv->vbi_even;

  
  *(po++)=BT848_RISC_SYNC|BT848_FIFO_STATUS_FM1;
  
  *(po++)=0;

  for (i=0; i<16; i++) 
  {
    *(po++)=BT848_RISC_WRITE|2044|BT848_RISC_EOL|BT848_RISC_SOL|(13<<20);
	mem_offset(btv->vbibuf, NOFD, 1, &offset, 0);
	offset += i*2048;
    *(po++)= ((u32)offset);
	///*(po++) = btv->vbibuf+i*2048;
  }
  *(po++)=BT848_RISC_JUMP;
  mem_offset(btv->risc_jmp+4, NOFD, 1, &offset, 0);
  //*(po++)=virt_to_bus(btv->risc_jmp+4);
  *(po++) = ((u32)offset);
  ////*(po++) = btv->risc_jmp+4;

  *(pe++)=BT848_RISC_SYNC|BT848_FIFO_STATUS_FM1; *(pe++)=0;
  for (i=16; i<32; i++) {
    *(pe++)=BT848_RISC_WRITE|2044|BT848_RISC_EOL|BT848_RISC_SOL;
    //*(pe++)=virt_to_bus(btv->vbibuf)+i*2048;
	mem_offset(btv->vbibuf, NOFD, 1, &offset, 0);
	offset += i*2048;
	*(pe++) = ((u32) offset);
	////*(pe++) = btv->vbibuf)+i*2048;
  }
  *(pe++)=BT848_RISC_JUMP|BT848_RISC_IRQ|(0x01<<16);
  //*(pe++)=virt_to_bus(btv->risc_jmp+10);

  mem_offset(btv->risc_jmp+10, NOFD, 1, &offset, 0);
  *(pe++) = ((u32)offset);
  ////*(pe++) = btv->risc_jmp+10;

}


/***********************************************************************************
	Initialization
************************************************************************************/
static int
init_bt848(struct bttv * btv, int video_format)
{
  int i;

  //Reset the framegrabber
  btwrite(0,BT848_SRESET);

  /* default setup for max. PAL size in a 1024xXXX hicolor framebuffer */

  btv->win.norm = video_format; /* change this to 1 for NTSC */

  btv->win.interlace = 0;
  btv->win.x=0;
  btv->win.y=0;

  switch(btv->win.norm)
  {
  case 0:
	  btv->win.width=768; /* 640 */
	  btv->win.height=576; /* 480 */
	  btv->win.cropwidth=768; /* 640 */
	  btv->win.cropheight=576; /* 480 */
	  break;

  case 1:
	  btv->win.width = 640; 
	  btv->win.height = 480; 
	  btv->win.cropwidth = 640; 
	  btv->win.cropheight = 480;
	  break;
  }

  btv->win.cropx=0;
  btv->win.cropy=0;
  btv->win.bpp=2;
  btv->win.bpl=1024*btv->win.bpp;
  //btv->win.bpl=2048*btv->win.bpp;
  btv->win.swidth=1024;
  btv->win.sheight=768;
  btv->cap=0;

  //wtv.riscmem=(ulong *) malloc(RISCMEM_LEN);
  btv->risc_odd = (u32 *) mmap(0,
					RISCMEM_LEN/2,
					PROT_READ|PROT_WRITE|PROT_NOCACHE,
					MAP_PHYS|MAP_ANON,
					NOFD,
					0);

  btv->risc_even = (u32 *) mmap(0,
					RISCMEM_LEN/2,
					PROT_READ|PROT_WRITE|PROT_NOCACHE,
					MAP_PHYS|MAP_ANON,
					NOFD,
					0);

  btv->risc_jmp = (u32 *) mmap(0,
					2048,
					PROT_READ|PROT_WRITE|PROT_NOCACHE,
					MAP_PHYS|MAP_ANON,
					NOFD,
					0);

  btv->vbi_odd  = btv->risc_jmp + 12;
  btv->vbi_even = btv->vbi_odd + 256;

  btv->bus_vbi_odd=virt_to_bus(btv->risc_jmp);
  btv->bus_vbi_even=virt_to_bus(btv->risc_jmp+6);
  //btwrite(virt_to_bus(btv->risc_jmp+2), BT848_RISC_STRT_ADD);

  //Important one: program the RISC address in the RISC_STRT_ADD register
  regbase[69] = virt_to_bus(btv->risc_jmp+2);
  //regbase[69] = virt_to_bus( btv->risc_even);

  btv->vbibuf = (unchar *) mmap(0,
					VBIBUF_SIZE,
					PROT_READ|PROT_WRITE|PROT_NOCACHE,
					MAP_PHYS|MAP_ANON | MAP_BELOW16M ,
					NOFD,
					0);
  memset(btv->vbibuf, 1,VBIBUF_SIZE);

  btv->imagebuf =   (unchar *) mmap(0,
					VBIBUF_SIZE,
					PROT_READ|PROT_WRITE|PROT_NOCACHE,
					MAP_PHYS|MAP_ANON | MAP_BELOW16M ,
					NOFD,
					0);
  memset(btv->imagebuf, 1,VBIBUF_SIZE);

  //bt848_muxsel(btv, 1);
  

  btwrite(0x10, BT848_COLOR_CTL);

  btwrite(0x00, BT848_CAP_CTL);

  btwrite(0x0ff, BT848_VBI_PACK_SIZE);
  btwrite(1, BT848_VBI_PACK_DEL);

  btwrite(0xfc, BT848_GPIO_DMA_CTL);

  switch(btv->win.norm)
  {
  case 0:
	  btwrite(BT848_IFORM_MUX1 | BT848_IFORM_XTAUTO | BT848_IFORM_PAL_BDGHI,
  	  BT848_IFORM);

	  break;

  case 1:
	  btwrite(BT848_IFORM_MUX1 | BT848_IFORM_XTAUTO | BT848_IFORM_NTSC,
  	  BT848_IFORM);

	  break;
  }
  

  bt848_set_size(btv);

  bt848_bright(btv, 0x10);
  btwrite(0xd8, BT848_CONTRAST_LO);

  btwrite(0x60, BT848_E_VSCALE_HI);
  btwrite(0x60, BT848_O_VSCALE_HI);
  btwrite(/*BT848_ADC_SYNC_T|*/
	  BT848_ADC_RESERVED|BT848_ADC_CRUSH, BT848_ADC);

  btwrite(BT848_CONTROL_LDEC, BT848_E_CONTROL);
  btwrite(BT848_CONTROL_LDEC, BT848_O_CONTROL);
  btwrite(0x00, BT848_E_SCLOOP);
  btwrite(0x00, BT848_O_SCLOOP);

  //This is the interrupt part, It blocks the QNX system. ??

  //btwrite(0xffffffUL,BT848_INT_STAT);
  regbase[64] = 0xffffffUL;
/*	  BT848_INT_PABORT|BT848_INT_RIPERR|BT848_INT_PPERR|BT848_INT_FDSR|
	  BT848_INT_FTRGT|BT848_INT_FBUS|*/
 /*btwrite(
	  BT848_INT_SCERR|(1<<23)|
	  BT848_INT_RISCI|BT848_INT_OCERR|BT848_INT_VPRES|
	  BT848_INT_I2CDONE|BT848_INT_FMTCHG|BT848_INT_HLOCK,
	  BT848_INT_MASK);
  */

  //Program the tab stuff
  make_vbitab(btv);
  //make_iov_risctab(btv);
  make_rawrisctab(btv, 
				  btv->risc_odd,
                  btv->risc_even, 
				  btv->imagebuf);
  
  bt848_set_risc_jmps(btv);

  idcard(btv);

  return 0;
}

/*************************************************************************
 *	Main function. Activate the resorce manager	
 *
 *************************************************************************/

main(int argc, char **argv)
{
	int i; 
	/* declare variables we'll be using */
	resmgr_attr_t		resmgr_attr;
	dispatch_t			*dpp;
	dispatch_context_t  *ctp;
	int                 id;
	int video_format = 0;

	if ( argc == 1 || argc > 2)
	{
		printf("Use: ./bttvx video_format\n");
		printf("0 ----> PAL\n");
		printf("1 ----> NTSC\n"); 
		exit(0);
	}

	video_format = atoi(argv[1]);

	if (video_format > 1 || video_format < 0)
	{
		printf("bttvx: Sorry!, only PAL and NTSC supported\n");
		exit(0);
	}

	i = find_bt848();

	if ( i == 0)
	{
		printf("bttvx: sorry I didn't find the card\n");
		return(0);
	}

	i = init_bt848(&bttvs[0],video_format); 

	/* initialize dispatch interface */
	if((dpp = dispatch_create()) == NULL) 
	{
		fprintf(stderr, 
			    "%s: Unable to allocate dispatch handle.\n",
				argv[0]);
		return EXIT_FAILURE;
	}

	/* initialize resource manager attributes */
	memset(&resmgr_attr, 0, sizeof resmgr_attr);
	
	resmgr_attr.nparts_max = 1;
	//resmgr_attr.msg_max_size = 2048;
	resmgr_attr.msg_max_size=VBIBUF_SIZE;

	/* initialize functions for handling messages */
	iofunc_func_init(_RESMGR_CONNECT_NFUNCS, 
					 &connect_funcs,
					 _RESMGR_IO_NFUNCS, 
					 &io_funcs);

	io_funcs.read = io_read;
	connect_funcs.open = io_open;

	/* initialize attribute structure used by the device */
	iofunc_attr_init(&attr, 
					 S_IFNAM | 0666, 
					 0, 0);
	//attr.nbytes = strlen(buffer)+1;
	attr.nbytes = VBIBUF_SIZE + 1;

    /* attach our device name */
    id = resmgr_attach(dpp,            /* dispatch handle        */
                       &resmgr_attr,   /* resource manager attrs */
                       "/dev/bttvx",  /* device name            */
                       _FTYPE_ANY,     /* open type              */
                       0,              /* flags                  */
                       &connect_funcs, /* connect routines       */
                       &io_funcs,      /* I/O routines           */
                       &attr);         /* handle                 */
    if(id == -1) {
        fprintf(stderr, "%s: Unable to attach name.\n", argv[0]);
        return EXIT_FAILURE;
    }

	/* allocate a context structure */
	ctp = dispatch_context_alloc(dpp);

	/* start the resource manager message loop */
	while(1) 
	{
		if((ctp = dispatch_block(ctp)) == NULL) 
		{
			fprintf(stderr, "block error\n");
			return EXIT_FAILURE;
		}
		dispatch_handler(ctp);
	}
}

