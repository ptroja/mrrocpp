#include <linux/sockios.h>

#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef FAILURE
#define FAILURE -1
#endif

#define SIOCDEVSOCKETCAN     SIOCDEVPRIVATE

struct socketcanconf
{
        int icmd;
        unsigned char key[32];
	unsigned int ven_dev_id;
	unsigned int sub_ven_dev_id;
};

#define ICPDAS_SET_AUTHKEY      0x1
#define ICPDAS_GET_AUTHKEY      0x2
#define ICPDAS_ENABLE_UDELAY    0x3
#define ICPDAS_DISABLE_UDELAY   0x4
#define ICPDAS_GET_CARDID 	0x5
