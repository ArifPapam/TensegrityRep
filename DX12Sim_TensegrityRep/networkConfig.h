#ifndef _NETWORK_CONFIG

#define _NETWORK_CONFIG

#define CAM_HEIGHT_HACK 3000
#define PERF_AVG_CNT 100

//Old Client
const unsigned short clientPort = 27016;
const unsigned short clientPortB = 27015;

//NetworkData
const unsigned short dataPort = 27015;

//NetworkStat
const unsigned short statPort = 27016;

//All IP
/*const char * IP_B = "169.254.39.78";
#if 0
const char * IP_A = "127.0.0.1";
#else
const char * IP_A = "192.168.100.3";
#endif*/

#define _IP_B "169.254.39.78"
#if 1
#define _IP_A "127.0.0.1"
#else
//#define _IP_A "192.168.100.8"
#define _IP_A "192.168.100.10"
//#define _IP_A "169.254.110.61"
#endif

#endif