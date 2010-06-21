#ifndef __EDP_SPEAKER_SOUND_H
#define __EDP_SPEAKER_SOUND_H

#include <stdio.h>

typedef struct
{
    char    tag[4];
    long    length;
}
RiffTag;

typedef struct
{
    char    Riff[4];
    long    Size;
    char    Wave[4];
}
RiffHdr;


typedef struct
{
    short   FormatTag;
    short   Channels;
    long    SamplesPerSec;
    long    AvgBytesPerSec;
    short   BlockAlign;
    short   BitsPerSample;
}
WaveHdr;


int err (char *msg);
int FindTag (FILE * fp, const char *tag);
int CheckHdr (FILE * fp);
int dev_raw (int fd);
int dev_unraw (int fd);


// *****************************************************************************
/* *INDENT-OFF* */
#ifdef __USAGE
%C[Options] *

Options:
    -a[card#:]<dev#>  the card & device number to play out on
#endif
/* *INDENT-ON* */
// *****************************************************************************

#endif
