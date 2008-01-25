#ifndef _EDP_SPEAKER_SOUND_H
#define _EDP_SPEAKER_SOUND_H

#include <errno.h>
//#include <fcntl.h>
#include <gulliver.h>
//#include <stdio.h>
//#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
//#include <sys/stat.h>
#include <sys/termio.h>
//#include <sys/types.h>
//#include <unistd.h>

#include <sys/asoundlib.h>
#include<math.h>

const char *kRiffId = "RIFF";
const char *kWaveId = "WAVE";

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


int
err (char *msg)
{
    perror (msg);
    return -1;
}


int
FindTag (FILE * fp, const char *tag)
{
    int     retVal;
    RiffTag tagBfr =
    {"", 0};

    retVal = 0;

    // Keep reading until we find the tag or hit the EOF.
    while (fread ((unsigned char *) &tagBfr, sizeof (tagBfr), 1, fp))
    {

        // If this is our tag, set the length and break.
        if (strncmp (tag, tagBfr.tag, sizeof tagBfr.tag) == 0)
        {
            retVal = ENDIAN_LE32(tagBfr.length);
            break;
        }

        // Skip ahead the specified number of bytes in the stream
        fseek (fp, tagBfr.length, SEEK_CUR);
    }

    // Return the result of our operation
    return (retVal);
}


int
CheckHdr (FILE * fp)
{
    RiffHdr riffHdr =
    {"", 0};

    // Read the header and, if successful, play the file
    // file or WAVE file.
    if (fread ((unsigned char *) &riffHdr, sizeof (RiffHdr), 1, fp) == 0)
        return 0;

    if (strncmp (riffHdr.Riff, kRiffId, strlen (kRiffId)) ||
        strncmp (riffHdr.Wave, kWaveId, strlen (kWaveId)))
        return -1;

    return 0;
}


int
dev_raw (int fd)
{
    struct termios termios_p;

    if (tcgetattr (fd, &termios_p))
        return (-1);

    termios_p.c_cc[VMIN] = 1;
    termios_p.c_cc[VTIME] = 0;
    termios_p.c_lflag &= ~(ECHO | ICANON | ISIG |
        ECHOE | ECHOK | ECHONL);
    termios_p.c_oflag &= ~(OPOST);
    return (tcsetattr (fd, TCSANOW, &termios_p));
}

int
dev_unraw (int fd)
{
    struct termios termios_p;

    if (tcgetattr (fd, &termios_p))
        return (-1);

    termios_p.c_lflag |= (ECHO | ICANON | ISIG |
        ECHOE | ECHOK | ECHONL);
    termios_p.c_oflag |= (OPOST);
    return (tcsetattr (fd, TCSAFLUSH, &termios_p));
}

//badziewie
struct
{
    char    riff_id[4];
    char    wave_len[4];
    struct
    {
        char    fmt_id[8];
        char    fmt_len[4];
        struct
        {
            char    format_tag[2];
            char    voices[2];
            char    rate[4];
            char    char_per_sec[4];
            char    block_align[2];
            char    bits_per_sample[2];
        }
        fmt;
        struct
        {
            char    data_id[4];
            char    data_len[4];
        }
        data;
    }
    wave;
}
riff_hdr =
{
    {'R', 'I', 'F', 'F' },
    {sizeof (riff_hdr.wave), 0, 0, 0 },
    {
        {'W', 'A', 'V', 'E', 'f', 'm', 't', ' ' },
        {sizeof (riff_hdr.wave.fmt), 0, 0, 0 },
        {
            {1, 0 }, 
            {0, 0 },
            {0, 0, 0, 0 },
            {0, 0, 0, 0 },
            {0, 0 },
            {0, 0 }
        },
        {
            {'d', 'a', 't', 'a' },
            {0, 0, 0, 0 }
        }
    }
};
 //niewiadomo czy jest potrzebne to badziewie

int		 i;
	int 	state;

	int     card;
    int     dev;

    snd_pcm_t *pcm_handle;
    //FILE   *file1;
    //FILE   *file2;
    int     mSamples;
    int     mSampleRate;
    int     mSampleChannels;
    int     mSampleBits;
    int     mSampleTime;
	short   *mSampleBfr1;  //char  
    
 	int     rtn;   

  

    snd_pcm_channel_info_t pi;
    snd_mixer_t *mixer_handle;
    snd_mixer_group_t group;
    snd_pcm_channel_params_t pp;
    snd_pcm_channel_setup_t setup;
    int     bsize, n, N = 0, c;

    fd_set  rfds, wfds;


 FILE   *file1;
    WaveHdr wavHdr1;

// *****************************************************************************
/* *INDENT-OFF* */
#ifdef __USAGE
%C[Options] *

Options:
    -a[card#:]<dev#>  the card & device number to play out on
#endif
/* *INDENT-ON* */
// *****************************************************************************

//int i=0;
//int max=0,min=1000;

#endif /* _EDP_SPEAKER_SOUND_H */
