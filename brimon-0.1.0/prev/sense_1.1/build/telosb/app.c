#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/usr/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 38 "/usr/msp430/include/sys/inttypes.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 385 "/usr/lib/ncc/nesc_nx.h"
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/usr/msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 42 "/usr/msp430/include/string.h" 3
extern void *memset(void *arg_0x2ba081031980, int arg_0x2ba081031be8, size_t arg_0x2ba08102f020);
#line 63
extern void *memset(void *arg_0x2ba08104ab10, int arg_0x2ba08104ad78, size_t arg_0x2ba081049060);
# 59 "/usr/msp430/include/stdlib.h" 3
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/usr/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x2ba0810840c8);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x2ba081087180);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 23 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 39 "/usr/msp430/include/msp430/iostructures.h" 3
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
#line 108
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};









struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 116 "/usr/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");





volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");





volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");










volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 92 "/usr/msp430/include/msp430/usart.h" 3
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");
#line 277
volatile unsigned char U1TCTL __asm ("0x0079");
# 27 "/usr/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 70
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 127
#line 118
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 26 "/usr/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
#line 76
#line 64
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/usr/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/usr/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 83 "/usr/msp430/include/msp430x16x.h" 3
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 158 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 193
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 248
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
# 8 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4258 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4259 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 18 "flashsampler.h"
enum __nesc_unnamed4260 {
  SAMPLE_INTERVAL = 1024L * 60, 

  SAMPLE_PERIOD = 1000, 


  BUFFER_SIZE = 512, 

  TOTAL_SAMPLES = 32768L, 

  TOTAL_BUFFERS = TOTAL_SAMPLES / BUFFER_SIZE, 


  SUMMARY_SAMPLES = 256, 

  DFACTOR = 32768L / SUMMARY_SAMPLES
};
# 29 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4261 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4262 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4263 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 32 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/types/Leds.h"
enum __nesc_unnamed4264 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/types/Storage.h"
typedef uint8_t volume_id_t;
typedef uint32_t storage_addr_t;
typedef uint32_t storage_len_t;
typedef uint32_t storage_cookie_t;

enum __nesc_unnamed4265 {
  SEEK_BEGINNING = 0
};
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4266 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4267 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 76
#line 66
typedef struct __nesc_unnamed4268 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4269 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25p.h"
typedef storage_addr_t stm25p_addr_t;
typedef storage_len_t stm25p_len_t;

enum __nesc_unnamed4270 {
  STM25P_NUM_SECTORS = 16, 
  STM25P_SECTOR_SIZE_LOG2 = 16, 
  STM25P_SECTOR_SIZE = 1L << STM25P_SECTOR_SIZE_LOG2, 
  STM25P_SECTOR_MASK = 0xffff, 
  STM25P_PAGE_SIZE_LOG2 = 8, 
  STM25P_PAGE_SIZE = 1 << STM25P_PAGE_SIZE_LOG2, 
  STM25P_PAGE_MASK = STM25P_PAGE_SIZE - 1, 
  STM25P_INVALID_ADDRESS = 0xffffffff
};




#line 54
typedef struct stm25p_volume_info_t {
  uint8_t base;
  uint8_t size;
} stm25p_volume_info_t;
# 11 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/types/StorageVolumes.h"
static const stm25p_volume_info_t STM25P_VMAP[4] = { 
{ .base = 15, .size = 1 }, 
{ .base = 0, .size = 1 }, 
{ .base = 1, .size = 1 }, 
{ .base = 2, .size = 1 } };
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 80 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/crc.h"
static inline uint16_t crcByte(uint16_t crc, uint8_t b);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4271 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4272 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4273 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4274 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4275 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4276 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4277 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 169
#line 150
typedef enum __nesc_unnamed4278 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4279 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4280 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4281 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 248
#line 240
typedef struct __nesc_unnamed4282 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4283 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4284 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4285 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4286 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4287 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4288 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12.h"
#line 48
typedef struct __nesc_unnamed4289 {

  unsigned int inch : 4;
  unsigned int sref : 3;
  unsigned int ref2_5v : 1;
  unsigned int adc12ssel : 2;
  unsigned int adc12div : 3;
  unsigned int sht : 4;
  unsigned int sampcon_ssel : 2;
  unsigned int sampcon_id : 2;
  unsigned int  : 0;
} msp430adc12_channel_config_t;








#line 61
typedef struct __nesc_unnamed4290 {


  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute((packed))  adc12memctl_t;

enum inch_enum {


  INPUT_CHANNEL_A0 = 0, 
  INPUT_CHANNEL_A1 = 1, 
  INPUT_CHANNEL_A2 = 2, 
  INPUT_CHANNEL_A3 = 3, 
  INPUT_CHANNEL_A4 = 4, 
  INPUT_CHANNEL_A5 = 5, 
  INPUT_CHANNEL_A6 = 6, 
  INPUT_CHANNEL_A7 = 7, 
  EXTERNAL_REF_VOLTAGE_CHANNEL = 8, 
  REF_VOLTAGE_NEG_TERMINAL_CHANNEL = 9, 
  TEMPERATURE_DIODE_CHANNEL = 10, 
  SUPPLY_VOLTAGE_HALF_CHANNEL = 11, 
  INPUT_CHANNEL_NONE = 12
};

enum sref_enum {

  REFERENCE_AVcc_AVss = 0, 
  REFERENCE_VREFplus_AVss = 1, 
  REFERENCE_VeREFplus_AVss = 2, 
  REFERENCE_AVcc_VREFnegterm = 4, 
  REFERENCE_VREFplus_VREFnegterm = 5, 
  REFERENCE_VeREFplus_VREFnegterm = 6
};

enum ref2_5v_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1, 
  REFVOLT_LEVEL_NONE = 0
};

enum adc12ssel_enum {

  SHT_SOURCE_ADC12OSC = 0, 
  SHT_SOURCE_ACLK = 1, 
  SHT_SOURCE_MCLK = 2, 
  SHT_SOURCE_SMCLK = 3
};

enum adc12div_enum {

  SHT_CLOCK_DIV_1 = 0, 
  SHT_CLOCK_DIV_2 = 1, 
  SHT_CLOCK_DIV_3 = 2, 
  SHT_CLOCK_DIV_4 = 3, 
  SHT_CLOCK_DIV_5 = 4, 
  SHT_CLOCK_DIV_6 = 5, 
  SHT_CLOCK_DIV_7 = 6, 
  SHT_CLOCK_DIV_8 = 7
};

enum sht_enum {

  SAMPLE_HOLD_4_CYCLES = 0, 
  SAMPLE_HOLD_8_CYCLES = 1, 
  SAMPLE_HOLD_16_CYCLES = 2, 
  SAMPLE_HOLD_32_CYCLES = 3, 
  SAMPLE_HOLD_64_CYCLES = 4, 
  SAMPLE_HOLD_96_CYCLES = 5, 
  SAMPLE_HOLD_123_CYCLES = 6, 
  SAMPLE_HOLD_192_CYCLES = 7, 
  SAMPLE_HOLD_256_CYCLES = 8, 
  SAMPLE_HOLD_384_CYCLES = 9, 
  SAMPLE_HOLD_512_CYCLES = 10, 
  SAMPLE_HOLD_768_CYCLES = 11, 
  SAMPLE_HOLD_1024_CYCLES = 12
};

enum sampcon_ssel_enum {

  SAMPCON_SOURCE_TACLK = 0, 
  SAMPCON_SOURCE_ACLK = 1, 
  SAMPCON_SOURCE_SMCLK = 2, 
  SAMPCON_SOURCE_INCLK = 3
};

enum sampcon_id_enum {

  SAMPCON_CLOCK_DIV_1 = 0, 
  SAMPCON_CLOCK_DIV_2 = 1, 
  SAMPCON_CLOCK_DIV_3 = 2, 
  SAMPCON_CLOCK_DIV_4 = 3
};
typedef TMilli FlashSamplerC__Timer__precision_tag;
typedef uint16_t AccelSamplerC__Accel__val_t;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4291 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
enum /*FlashSamplerAppC.LogStorageC*/LogStorageC__0____nesc_unnamed4292 {
  LogStorageC__0__LOG_ID = 0U, LogStorageC__0__VOLUME_ID = 0U
};
typedef bool Stm25pLogP__Circular__val_t;
typedef TMilli /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__precision_tag;
enum /*HplStm25pSpiC.SpiC*/Msp430Spi0C__0____nesc_unnamed4293 {
  Msp430Spi0C__0__CLIENT_ID = 0U
};
enum /*HplStm25pSpiC.SpiC.UsartC*/Msp430Usart0C__0____nesc_unnamed4294 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
typedef bool /*FlashSamplerAppC.LogStorageC.ConfigP*/Stm25pLogConfigP__0__Circular__val_t;
enum /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0____nesc_unnamed4295 {
  BlockStorageC__0__BLOCK_ID = 0U, BlockStorageC__0__VOLUME_ID = 1U
};
typedef uint16_t AdcP__Read__val_t;
typedef uint16_t AdcP__ReadNow__val_t;
typedef const msp430adc12_channel_config_t *AdcP__Config__adc_config_t;
typedef TMilli Msp430RefVoltGeneratorP__SwitchOffTimer__precision_tag;
typedef TMilli Msp430RefVoltGeneratorP__SwitchOnTimer__precision_tag;
typedef const msp430adc12_channel_config_t *Msp430RefVoltArbiterImplP__Config__adc_config_t;
enum /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0____nesc_unnamed4296 {
  Msp430Adc12ClientAutoRVGC__0__ID = 0U
};
typedef const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__adc_config_t;
typedef const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__adc_config_t;
enum /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC*/AdcReadClientC__0____nesc_unnamed4297 {
  AdcReadClientC__0__CLIENT = 0U
};
typedef TMilli AdcStreamP__Alarm__precision_tag;
typedef uint32_t AdcStreamP__Alarm__size_type;
typedef const msp430adc12_channel_config_t *AdcStreamP__AdcConfigure__adc_config_t;
typedef uint16_t AdcStreamP__ReadStream__val_t;
enum /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4298 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef TMilli /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__size_type;
typedef uint16_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t;
enum /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1____nesc_unnamed4299 {
  Msp430Adc12ClientAutoRVGC__1__ID = 1U
};
typedef const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__adc_config_t;
typedef const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__adc_config_t;
enum /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0____nesc_unnamed4300 {
  AdcReadStreamClientC__0__RSCLIENT = 0U
};
typedef const msp430adc12_channel_config_t *Msp430AxisXP__AdcConfigure__adc_config_t;
enum /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2____nesc_unnamed4301 {
  Msp430Adc12ClientAutoRVGC__2__ID = 2U
};
typedef const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__adc_config_t;
typedef const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__adc_config_t;
enum /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC*/AdcReadNowClientC__0____nesc_unnamed4302 {
  AdcReadNowClientC__0__CLIENT = 1U
};
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Boot.nc"
static void FlashSamplerC__Boot__booted(void );
# 13 "Summary.nc"
static void FlashSamplerC__Summary__summarized(error_t ok);
# 13 "Sample.nc"
static void FlashSamplerC__Sample__sampled(error_t error);
# 72 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void FlashSamplerC__Timer__fired(void );
# 12 "Summary.nc"
static void SummarizerC__Summary__summarize(void );
# 95 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
static void SummarizerC__BlockRead__computeCrcDone(storage_addr_t addr, storage_len_t len, 
uint16_t crc, error_t error);
#line 67
static void SummarizerC__BlockRead__readDone(storage_addr_t addr, 
#line 62
void * buf, 




storage_len_t len, 
error_t error);
# 118 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
static void SummarizerC__LogWrite__syncDone(error_t error);
#line 100
static void SummarizerC__LogWrite__eraseDone(error_t error);
#line 68
static void SummarizerC__LogWrite__appendDone(
#line 61
void * buf, 






storage_len_t len, bool recordsLost, 
error_t error);
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
static void AccelSamplerC__BlockWrite__syncDone(error_t error);
#line 71
static void AccelSamplerC__BlockWrite__writeDone(storage_addr_t addr, 
#line 66
void * buf, 




storage_len_t len, 
error_t error);
#line 91
static void AccelSamplerC__BlockWrite__eraseDone(error_t error);
# 12 "Sample.nc"
static void AccelSamplerC__Sample__sample(void );
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
static void AccelSamplerC__Accel__bufferDone(error_t result, 
#line 86
AccelSamplerC__Accel__val_t * buf, 



uint16_t count);
#line 102
static void AccelSamplerC__Accel__readDone(error_t result, uint32_t usActualPeriod);
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 51
static error_t MotePlatformC__Init__init(void );
# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 32
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 34
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2ba0816dac98);
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear(void );


static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(uint16_t clockSource);
#line 43
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(int mode);





static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(uint16_t inputDivider);
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2ba0816dac98);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );



static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(msp430_compare_control_t control);
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t time);
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );



static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(msp430_compare_control_t control);
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(uint16_t time);
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2ba081571d50);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2ba081571d50);
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Off(void );










static void LedsP__Leds__led1On(void );
#line 89
static void LedsP__Leds__led2Toggle(void );
#line 66
static void LedsP__Leds__led1Off(void );
#line 83
static void LedsP__Leds__led2Off(void );
#line 45
static void LedsP__Leds__led0On(void );
#line 78
static void LedsP__Leds__led2On(void );
# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 71
static void /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__set(void );




static void /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P47*/HplMsp430GeneralIOP__31__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P47*/HplMsp430GeneralIOP__31__IO__set(void );
#line 71
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );




static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
#line 64
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput(void );
#line 85
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc(void );
# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 98 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 125 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 72 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2ba081b2b5d8);
# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2ba081b2b5d8, 
# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2ba081b2b5d8);
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
static error_t Stm25pLogP__Sector__default__read(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 101
static void Stm25pLogP__Sector__writeDone(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 101 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, 
error_t error);









static error_t Stm25pLogP__Sector__default__erase(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint8_t sector, uint8_t num_sectors);








static void Stm25pLogP__Sector__eraseDone(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 121 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint8_t sector, uint8_t num_sectors, error_t error);
#line 144
static void Stm25pLogP__Sector__computeCrcDone(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, stm25p_len_t len, 
uint16_t crc, error_t error);
#line 91
static error_t Stm25pLogP__Sector__default__write(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 56
static uint8_t Stm25pLogP__Sector__default__getNumSectors(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
static void Stm25pLogP__Sector__readDone(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, 
error_t error);
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogRead.nc"
static void Stm25pLogP__Read__default__readDone(
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb91a0, 
# 70 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogRead.nc"
void * buf, 




storage_len_t len, error_t error);
#line 115
static void Stm25pLogP__Read__default__seekDone(
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb91a0, 
# 115 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogRead.nc"
error_t error);
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Get.nc"
static Stm25pLogP__Circular__val_t Stm25pLogP__Circular__default__get(
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bd3858);
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t Stm25pLogP__Init__init(void );
# 118 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
static void Stm25pLogP__Write__default__syncDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb8538, 
# 118 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
error_t error);
#line 100
static void Stm25pLogP__Write__default__eraseDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb8538, 
# 100 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
error_t error);
#line 68
static void Stm25pLogP__Write__default__appendDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb8538, 
# 61 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
void * buf, 






storage_len_t len, bool recordsLost, 
error_t error);
#line 54
static error_t Stm25pLogP__Write__append(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb8538, 
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
void * buf, 






storage_len_t len);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pLogP__ClientResource__default__release(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bdb220);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pLogP__ClientResource__default__request(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bdb220);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Stm25pLogP__ClientResource__granted(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bdb220);
# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t Stm25pSectorP__SplitControl__start(void );
#line 109
static error_t Stm25pSectorP__SplitControl__stop(void );
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
static error_t Stm25pSectorP__Sector__read(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 101
static void Stm25pSectorP__Sector__default__writeDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 101 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, 
error_t error);









static error_t Stm25pSectorP__Sector__erase(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint8_t sector, uint8_t num_sectors);








static void Stm25pSectorP__Sector__default__eraseDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 121 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint8_t sector, uint8_t num_sectors, error_t error);
#line 144
static void Stm25pSectorP__Sector__default__computeCrcDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, stm25p_len_t len, 
uint16_t crc, error_t error);
#line 133
static error_t Stm25pSectorP__Sector__computeCrc(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 133 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len);
#line 91
static error_t Stm25pSectorP__Sector__write(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 56
static uint8_t Stm25pSectorP__Sector__getNumSectors(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
static void Stm25pSectorP__Sector__default__readDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, 
error_t error);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Stm25pSectorP__Stm25pResource__granted(
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c8ca68);
# 48 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pVolume.nc"
static volume_id_t Stm25pSectorP__Volume__default__getVolumeId(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c8ed10);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Stm25pSectorP__SpiResource__granted(void );
# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
static void Stm25pSectorP__Spi__sectorEraseDone(uint8_t sector, error_t error);
#line 77
static void Stm25pSectorP__Spi__readDone(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error);
#line 125
static void Stm25pSectorP__Spi__pageProgramDone(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error);
#line 101
static void Stm25pSectorP__Spi__computeCrcDone(uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len, error_t error);
#line 159
static void Stm25pSectorP__Spi__bulkEraseDone(error_t error);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pSectorP__ClientResource__release(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c94108);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pSectorP__ClientResource__request(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c94108);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Stm25pSectorP__ClientResource__default__granted(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c94108);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void Stm25pSectorP__signalDone_task__runTask(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d41020);
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d3f340);
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d3f340);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d44d40);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d44d40);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__startDone(error_t error);
#line 117
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__stopDone(error_t error);
# 72 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__fired(void );
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/PowerDownCleanup.nc"
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask__runTask(void );
# 73 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__granted(void );
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask__runTask(void );
# 74 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__default__start(void );









static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__default__stop(void );
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
static void Stm25pSpiP__SpiPacket__sendDone(
#line 64
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t Stm25pSpiP__Init__init(void );
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
static error_t Stm25pSpiP__Spi__powerDown(void );
#line 66
static error_t Stm25pSpiP__Spi__read(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len);
#line 136
static error_t Stm25pSpiP__Spi__sectorErase(uint8_t sector);
#line 55
static error_t Stm25pSpiP__Spi__powerUp(void );
#line 90
static error_t Stm25pSpiP__Spi__computeCrc(uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len);
#line 114
static error_t Stm25pSpiP__Spi__pageProgram(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Stm25pSpiP__SpiResource__granted(void );
#line 110
static error_t Stm25pSpiP__ClientResource__release(void );
#line 78
static error_t Stm25pSpiP__ClientResource__request(void );
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e949b0);
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e949b0);
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e92df8, 
# 48 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 71
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e92df8, 
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e90e18);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e91be8);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e91be8);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e91be8);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e956f8);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e956f8);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e956f8);
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );
# 180 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 197
static void HplMsp430Usart0P__Usart__clrRxIntr(void );
#line 97
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 177
static void HplMsp430Usart0P__Usart__disableRxIntr(void );
#line 207
static void HplMsp430Usart0P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart0P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart0P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart0P__Usart__disableUart(void );
#line 153
static void HplMsp430Usart0P__Usart__enableSpi(void );
#line 168
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t HplMsp430Usart0P__Usart__rx(void );
#line 192
static bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 158
static void HplMsp430Usart0P__Usart__disableSpi(void );
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2ba081fe8108, 
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2ba081fe8108);
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2ba081fe7020);
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d41020);
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d3f340);
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d3f340);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d44d40);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d44d40);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d44d40);
# 80 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
# 7 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__makeOutput(void );
#line 29
static void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__set(void );
static void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__clr(void );




static void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 29
static void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__GeneralIO__set(void );
# 48 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pVolume.nc"
static volume_id_t /*FlashSamplerAppC.LogStorageC.BinderP*/Stm25pBinderP__0__Volume__getVolumeId(void );
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Get.nc"
static /*FlashSamplerAppC.LogStorageC.ConfigP*/Stm25pLogConfigP__0__Circular__val_t /*FlashSamplerAppC.LogStorageC.ConfigP*/Stm25pLogConfigP__0__Circular__get(void );
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
static error_t Stm25pBlockP__Sector__default__read(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 101
static void Stm25pBlockP__Sector__writeDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 101 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, 
error_t error);









static error_t Stm25pBlockP__Sector__default__erase(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint8_t sector, uint8_t num_sectors);








static void Stm25pBlockP__Sector__eraseDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 121 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint8_t sector, uint8_t num_sectors, error_t error);
#line 144
static void Stm25pBlockP__Sector__computeCrcDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, stm25p_len_t len, 
uint16_t crc, error_t error);
#line 133
static error_t Stm25pBlockP__Sector__default__computeCrc(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 133 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len);
#line 91
static error_t Stm25pBlockP__Sector__default__write(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 56
static uint8_t Stm25pBlockP__Sector__default__getNumSectors(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
static void Stm25pBlockP__Sector__readDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, 
error_t error);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
static error_t Stm25pBlockP__Read__read(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082125970, 
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
storage_addr_t addr, 
#line 49
void * buf, 






storage_len_t len);
#line 95
static void Stm25pBlockP__Read__default__computeCrcDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082125970, 
# 95 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
storage_addr_t addr, storage_len_t len, 
uint16_t crc, error_t error);
#line 67
static void Stm25pBlockP__Read__default__readDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082125970, 
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
storage_addr_t addr, 
#line 62
void * buf, 




storage_len_t len, 
error_t error);
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
static void Stm25pBlockP__Write__default__syncDone(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082120b68, 
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
error_t error);
#line 71
static void Stm25pBlockP__Write__default__writeDone(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082120b68, 
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
storage_addr_t addr, 
#line 66
void * buf, 




storage_len_t len, 
error_t error);










static error_t Stm25pBlockP__Write__erase(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082120b68);
# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
static void Stm25pBlockP__Write__default__eraseDone(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082120b68, 
# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
error_t error);
#line 58
static error_t Stm25pBlockP__Write__write(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082120b68, 
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
storage_addr_t addr, 
#line 51
void * buf, 






storage_len_t len);
#line 103
static error_t Stm25pBlockP__Write__sync(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082120b68);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pBlockP__ClientResource__default__release(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211b530);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pBlockP__ClientResource__default__request(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211b530);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Stm25pBlockP__ClientResource__granted(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211b530);
# 48 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pVolume.nc"
static volume_id_t /*FlashSamplerAppC.BlockStorageC.BinderP*/Stm25pBinderP__1__Volume__getVolumeId(void );
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void AdcP__SubResourceReadNow__granted(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821aa580);
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Read.nc"
static void AdcP__Read__default__readDone(
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821b24e8, 
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Read.nc"
error_t result, AdcP__Read__val_t val);
# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadNow.nc"
static void AdcP__ReadNow__default__readDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821af318, 
# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadNow.nc"
error_t result, AdcP__ReadNow__val_t val);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void AdcP__ResourceReadNow__default__granted(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821ad020);
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static AdcP__Config__adc_config_t AdcP__Config__default__getConfiguration(
# 48 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821a9b18);
# 189 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__default__getData(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821e5910);
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__default__configureSingle(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821e5910, 
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * AdcP__SingleChannel__multipleDataReady(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821e5910, 
# 227 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 206
static error_t AdcP__SingleChannel__singleDataReady(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821e5910, 
# 206 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__default__release(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821ab318);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void AdcP__ResourceRead__granted(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821ab318);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void AdcP__readDone__runTask(void );
# 107 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static void Msp430Adc12ImplP__MultiChannel__default__dataReady(
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba0822210c8, 
# 107 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *buffer, uint16_t numSamples);
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
static void Msp430Adc12ImplP__HplAdc12__conversionDone(uint16_t iv);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA1__fired(void );
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__default__memOverflow(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba08221f020);
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba08221f020);
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t Msp430Adc12ImplP__Init__init(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void Msp430Adc12ImplP__TimerA__overflow(void );
# 189 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t Msp430Adc12ImplP__SingleChannel__getData(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba082223ac0);
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t Msp430Adc12ImplP__SingleChannel__configureSingle(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba082223ac0, 
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba082223ac0, 
# 227 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 138
static error_t Msp430Adc12ImplP__SingleChannel__configureMultiple(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba082223ac0, 
# 138 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
#line 206
static error_t Msp430Adc12ImplP__SingleChannel__default__singleDataReady(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba082223ac0, 
# 206 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA0__fired(void );
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t HplAdc12P__HplAdc12__getCtl0(void );
#line 82
static adc12memctl_t HplAdc12P__HplAdc12__getMCtl(uint8_t idx);
#line 106
static void HplAdc12P__HplAdc12__resetIFGs(void );
#line 118
static bool HplAdc12P__HplAdc12__isBusy(void );
#line 75
static void HplAdc12P__HplAdc12__setMCtl(uint8_t idx, adc12memctl_t memControl);
#line 128
static void HplAdc12P__HplAdc12__startConversion(void );
#line 51
static void HplAdc12P__HplAdc12__setCtl0(adc12ctl0_t control0);
#line 89
static uint16_t HplAdc12P__HplAdc12__getMem(uint8_t idx);





static void HplAdc12P__HplAdc12__setIEFlags(uint16_t mask);
#line 123
static void HplAdc12P__HplAdc12__stopConversion(void );
#line 57
static void HplAdc12P__HplAdc12__setCtl1(adc12ctl1_t control1);
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init(void );
# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue(void );
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237dc80);
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237a220);
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237a220);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237e9b0);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237e9b0);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237e9b0);
# 88 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId(void );
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
static void Msp430RefVoltGeneratorP__HplAdc12__conversionDone(uint16_t iv);
# 72 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOffTimer__fired(void );
# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__start(void );
#line 83
static error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__start(void );
#line 109
static error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__stop(void );
# 72 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOnTimer__fired(void );
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static Msp430RefVoltArbiterImplP__Config__adc_config_t Msp430RefVoltArbiterImplP__Config__default__getConfiguration(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba08242d248);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error_t error);
#line 117
static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error_t error);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__default__release(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba0823db538);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__default__request(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba0823db538);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP__AdcResource__granted(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba0823db538);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__ClientResource__release(
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba0823dc298);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__ClientResource__request(
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba0823dc298);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP__ClientResource__default__granted(
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba0823dc298);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void Msp430RefVoltArbiterImplP__switchOff__runTask(void );
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error_t error);
#line 117
static void Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error_t error);
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__adc_config_t /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration(void );
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void AdcStreamP__bufferDone__runTask(void );
#line 64
static void AdcStreamP__readStreamDone__runTask(void );
#line 64
static void AdcStreamP__readStreamFail__runTask(void );
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void AdcStreamP__Alarm__fired(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t AdcStreamP__Init__init(void );
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static AdcStreamP__AdcConfigure__adc_config_t AdcStreamP__AdcConfigure__default__getConfiguration(
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082462318);
# 189 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__default__getData(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082465b90);
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__default__configureSingle(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082465b90, 
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * AdcStreamP__SingleChannel__multipleDataReady(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082465b90, 
# 227 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 138
static error_t AdcStreamP__SingleChannel__default__configureMultiple(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082465b90, 
# 138 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
#line 206
static error_t AdcStreamP__SingleChannel__singleDataReady(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082465b90, 
# 206 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
static error_t AdcStreamP__ReadStream__postBuffer(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082468b08, 
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
AdcStreamP__ReadStream__val_t * buf, 




uint16_t count);









static error_t AdcStreamP__ReadStream__read(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082468b08, 
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
uint32_t usPeriod);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);





static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 92
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 67
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(
# 26 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824fb148, 
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(
# 26 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824fb148, 
# 102 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
#line 89
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(
# 24 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824ff020, 
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, 



uint16_t count);
#line 68
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__postBuffer(
# 24 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824ff020, 
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, 




uint16_t count);









static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__read(
# 24 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824ff020, 
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
uint32_t usPeriod);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(
# 24 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824ff020, 
# 102 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(
# 27 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824f7020);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__request(
# 27 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824f7020);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(
# 27 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824f7020);
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__adc_config_t /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration(void );
#line 58
static Msp430AxisXP__AdcConfigure__adc_config_t Msp430AxisXP__AdcConfigure__getConfiguration(void );
#line 58
static /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__adc_config_t /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__getConfiguration(void );
# 12 "Summary.nc"
static void FlashSamplerC__Summary__summarize(void );
# 12 "Sample.nc"
static void FlashSamplerC__Sample__sample(void );
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
static void FlashSamplerC__Leds__led0Off(void );










static void FlashSamplerC__Leds__led1On(void );




static void FlashSamplerC__Leds__led1Off(void );
#line 45
static void FlashSamplerC__Leds__led0On(void );
# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void FlashSamplerC__Timer__startOneShot(uint32_t dt);
# 24 "FlashSamplerC.nc"
static inline void FlashSamplerC__Boot__booted(void );



static inline void FlashSamplerC__Timer__fired(void );




static inline void FlashSamplerC__Sample__sampled(error_t error);





static inline void FlashSamplerC__Summary__summarized(error_t ok);
# 13 "Summary.nc"
static void SummarizerC__Summary__summarized(error_t ok);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
static error_t SummarizerC__BlockRead__read(storage_addr_t addr, 
#line 49
void * buf, 






storage_len_t len);
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
static error_t SummarizerC__LogWrite__append(
#line 47
void * buf, 






storage_len_t len);
# 19 "SummarizerC.nc"
uint16_t SummarizerC__summary[SUMMARY_SAMPLES];
#line 19
uint16_t SummarizerC__samples[DFACTOR];
uint16_t SummarizerC__index;

static void SummarizerC__nextSummarySample(void );

static inline void SummarizerC__Summary__summarize(void );





static void SummarizerC__nextSummarySample(void );




static inline void SummarizerC__BlockRead__readDone(storage_addr_t addr, void *buf, storage_len_t len, 
error_t error);
#line 51
static inline void SummarizerC__LogWrite__appendDone(void *buf, storage_len_t len, bool recordsLost, 
error_t error);




static inline void SummarizerC__BlockRead__computeCrcDone(storage_addr_t addr, storage_len_t len, 
uint16_t crc, error_t error);

static inline void SummarizerC__LogWrite__eraseDone(error_t error);

static inline void SummarizerC__LogWrite__syncDone(error_t error);
# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
static error_t AccelSamplerC__BlockWrite__erase(void );
#line 58
static error_t AccelSamplerC__BlockWrite__write(storage_addr_t addr, 
#line 51
void * buf, 






storage_len_t len);
#line 103
static error_t AccelSamplerC__BlockWrite__sync(void );
# 13 "Sample.nc"
static void AccelSamplerC__Sample__sampled(error_t error);
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
static void AccelSamplerC__Leds__led2Toggle(void );
#line 83
static void AccelSamplerC__Leds__led2Off(void );
#line 78
static void AccelSamplerC__Leds__led2On(void );
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
static error_t AccelSamplerC__Accel__postBuffer(
#line 63
AccelSamplerC__Accel__val_t * buf, 




uint16_t count);









static error_t AccelSamplerC__Accel__read(uint32_t usPeriod);
# 22 "AccelSamplerC.nc"
uint16_t AccelSamplerC__buffer1[BUFFER_SIZE];
#line 22
uint16_t AccelSamplerC__buffer2[BUFFER_SIZE];
int8_t AccelSamplerC__nbuffers;

static inline void AccelSamplerC__Sample__sample(void );




static inline void AccelSamplerC__BlockWrite__eraseDone(error_t ok);








static inline void AccelSamplerC__Accel__bufferDone(error_t ok, uint16_t *buf, uint16_t count);





static inline void AccelSamplerC__BlockWrite__writeDone(storage_addr_t addr, void *buf, storage_len_t len, 
error_t error);










static inline void AccelSamplerC__BlockWrite__syncDone(error_t error);




static inline void AccelSamplerC__Accel__readDone(error_t ok, uint32_t usActualPeriod);
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 51
static error_t PlatformP__MoteClockInit__init(void );
#line 51
static error_t PlatformP__LedsInit__init(void );
# 10 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 32 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4303 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 68
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 89
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 104
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 119
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 152
static inline void Msp430ClockP__startTimerB(void );
#line 164
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 193
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 218
static inline error_t Msp430ClockP__Init__init(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2ba0816dac98);
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 80 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(int mode);









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear(void );









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(uint16_t clockSource);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(uint16_t inputDivider);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2ba0816dac98);
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 89
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x);
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x);
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 89
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x);
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(uint16_t x);
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(12)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(10)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(26)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(24)))  ;
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 104
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 61
static void RealMainP__Scheduler__taskLoop(void );
#line 54
static bool RealMainP__Scheduler__runNextTask(void );
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2ba081571d50);
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4304 {

  SchedulerBasicP__NUM_TASKS = 14U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 86
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 159
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 29
static void LedsP__Led0__set(void );
static void LedsP__Led0__clr(void );




static void LedsP__Led1__makeOutput(void );
#line 29
static void LedsP__Led1__set(void );
static void LedsP__Led1__clr(void );
static void LedsP__Led2__toggle(void );



static void LedsP__Led2__makeOutput(void );
#line 29
static void LedsP__Led2__set(void );
static void LedsP__Led2__clr(void );
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 63
static inline void LedsP__Leds__led0On(void );




static inline void LedsP__Leds__led0Off(void );









static inline void LedsP__Leds__led1On(void );




static inline void LedsP__Leds__led1Off(void );









static inline void LedsP__Leds__led2On(void );




static inline void LedsP__Leds__led2Off(void );




static inline void LedsP__Leds__led2Toggle(void );
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 45
static void /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__set(void );
static void /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P47*/HplMsp430GeneralIOP__31__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P47*/HplMsp430GeneralIOP__31__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc(void );
#line 50
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc(void );
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4305 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4306 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 98 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 72 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4307 {
#line 63
  AlarmToTimerC__0__fired = 0U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 125 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2ba081b2b5d8);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4308 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 1U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4309 {

  VirtualizeTimerC__0__NUM_TIMERS = 4U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4310 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 89
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);
#line 148
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
static error_t Stm25pLogP__Sector__read(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 112
static error_t Stm25pLogP__Sector__erase(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint8_t sector, uint8_t num_sectors);
#line 91
static error_t Stm25pLogP__Sector__write(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468, 
# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 56
static uint8_t Stm25pLogP__Sector__getNumSectors(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb6468);
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogRead.nc"
static void Stm25pLogP__Read__readDone(
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb91a0, 
# 70 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogRead.nc"
void * buf, 




storage_len_t len, error_t error);
#line 115
static void Stm25pLogP__Read__seekDone(
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb91a0, 
# 115 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogRead.nc"
error_t error);
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Get.nc"
static Stm25pLogP__Circular__val_t Stm25pLogP__Circular__get(
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bd3858);
# 118 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
static void Stm25pLogP__Write__syncDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb8538, 
# 118 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
error_t error);
#line 100
static void Stm25pLogP__Write__eraseDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb8538, 
# 100 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
error_t error);
#line 68
static void Stm25pLogP__Write__appendDone(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bb8538, 
# 61 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
void * buf, 






storage_len_t len, bool recordsLost, 
error_t error);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pLogP__ClientResource__release(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bdb220);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pLogP__ClientResource__request(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
uint8_t arg_0x2ba081bdb220);







enum Stm25pLogP____nesc_unnamed4311 {
  Stm25pLogP__NUM_LOGS = 1U, 
  Stm25pLogP__BLOCK_SIZE = 4096, 
  Stm25pLogP__BLOCK_SIZE_LOG2 = 12, 
  Stm25pLogP__BLOCK_MASK = Stm25pLogP__BLOCK_SIZE - 1, 
  Stm25pLogP__BLOCKS_PER_SECTOR = STM25P_SECTOR_SIZE / Stm25pLogP__BLOCK_SIZE, 
  Stm25pLogP__MAX_RECORD_SIZE = 254, 
  Stm25pLogP__INVALID_HEADER = 0xff
};








#line 64
typedef enum Stm25pLogP____nesc_unnamed4312 {
  Stm25pLogP__S_IDLE, 
  Stm25pLogP__S_READ, 
  Stm25pLogP__S_SEEK, 
  Stm25pLogP__S_ERASE, 
  Stm25pLogP__S_APPEND, 
  Stm25pLogP__S_SYNC
} Stm25pLogP__stm25p_log_req_t;






#line 73
typedef struct Stm25pLogP__stm25p_log_state_t {
  storage_cookie_t cookie;
  void *buf;
  uint8_t len;
  Stm25pLogP__stm25p_log_req_t req;
} Stm25pLogP__stm25p_log_state_t;





#line 80
typedef struct Stm25pLogP__stm25p_log_info_t {
  stm25p_addr_t read_addr;
  stm25p_addr_t remaining;
  stm25p_addr_t write_addr;
} Stm25pLogP__stm25p_log_info_t;

Stm25pLogP__stm25p_log_state_t Stm25pLogP__m_log_state[Stm25pLogP__NUM_LOGS];
Stm25pLogP__stm25p_log_state_t Stm25pLogP__m_req;
Stm25pLogP__stm25p_log_info_t Stm25pLogP__m_log_info[Stm25pLogP__NUM_LOGS];
stm25p_addr_t Stm25pLogP__m_addr;
bool Stm25pLogP__m_records_lost;
uint8_t Stm25pLogP__m_header;
uint8_t Stm25pLogP__m_len;







#line 94
typedef enum Stm25pLogP____nesc_unnamed4313 {
  Stm25pLogP__S_SEARCH_BLOCKS, 
  Stm25pLogP__S_SEARCH_RECORDS, 
  Stm25pLogP__S_SEARCH_SEEK, 
  Stm25pLogP__S_HEADER, 
  Stm25pLogP__S_DATA
} Stm25pLogP__stm25p_log_rw_state_t;

Stm25pLogP__stm25p_log_rw_state_t Stm25pLogP__m_rw_state;

static inline error_t Stm25pLogP__newRequest(uint8_t client);
static void Stm25pLogP__continueReadOp(uint8_t client);
static void Stm25pLogP__continueAppendOp(uint8_t client);
static void Stm25pLogP__signalDone(uint8_t id, error_t error);

static inline error_t Stm25pLogP__Init__init(void );
#line 157
static inline error_t Stm25pLogP__Write__append(uint8_t id, void *buf, storage_len_t len);
#line 190
static inline error_t Stm25pLogP__newRequest(uint8_t client);
#line 202
static inline uint8_t Stm25pLogP__calcSector(uint8_t client, stm25p_addr_t addr);




static stm25p_addr_t Stm25pLogP__calcAddr(uint8_t client, stm25p_addr_t addr);






static void Stm25pLogP__ClientResource__granted(uint8_t id);
#line 272
static void Stm25pLogP__continueReadOp(uint8_t client);
#line 313
static inline void Stm25pLogP__Sector__readDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error);
#line 418
static void Stm25pLogP__continueAppendOp(uint8_t client);
#line 446
static inline void Stm25pLogP__Sector__eraseDone(uint8_t id, uint8_t sector, 
uint8_t num_sectors, 
error_t error);
#line 470
static inline void Stm25pLogP__Sector__writeDone(uint8_t id, storage_addr_t addr, 
uint8_t *buf, 
storage_len_t len, 
error_t error);
#line 485
static void Stm25pLogP__signalDone(uint8_t id, error_t error);
#line 514
static inline void Stm25pLogP__Sector__computeCrcDone(uint8_t id, stm25p_addr_t addr, stm25p_len_t len, uint16_t crc, error_t error);

static inline void Stm25pLogP__Read__default__readDone(uint8_t id, void *data, storage_len_t len, error_t error);
static inline void Stm25pLogP__Read__default__seekDone(uint8_t id, error_t error);
static inline void Stm25pLogP__Write__default__eraseDone(uint8_t id, error_t error);
static inline void Stm25pLogP__Write__default__appendDone(uint8_t id, void *data, storage_len_t len, bool recordsLost, error_t error);
static inline void Stm25pLogP__Write__default__syncDone(uint8_t id, error_t error);


static inline uint8_t Stm25pLogP__Sector__default__getNumSectors(uint8_t id);
static inline error_t Stm25pLogP__Sector__default__read(uint8_t id, storage_addr_t addr, uint8_t *buf, storage_len_t len);
static inline error_t Stm25pLogP__Sector__default__write(uint8_t id, storage_addr_t addr, uint8_t *buf, storage_len_t len);
static inline error_t Stm25pLogP__Sector__default__erase(uint8_t id, uint8_t sector, uint8_t num_sectors);

static inline error_t Stm25pLogP__ClientResource__default__request(uint8_t id);
static inline error_t Stm25pLogP__ClientResource__default__release(uint8_t id);
static inline bool Stm25pLogP__Circular__default__get(uint8_t id);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void Stm25pSectorP__SplitControl__startDone(error_t error);
#line 117
static void Stm25pSectorP__SplitControl__stopDone(error_t error);
# 101 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
static void Stm25pSectorP__Sector__writeDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 101 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, 
error_t error);
#line 121
static void Stm25pSectorP__Sector__eraseDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 121 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint8_t sector, uint8_t num_sectors, error_t error);
#line 144
static void Stm25pSectorP__Sector__computeCrcDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, stm25p_len_t len, 
uint16_t crc, error_t error);
#line 78
static void Stm25pSectorP__Sector__readDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c93328, 
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, 
error_t error);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pSectorP__Stm25pResource__release(
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c8ca68);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pSectorP__Stm25pResource__request(
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c8ca68);
# 48 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pVolume.nc"
static volume_id_t Stm25pSectorP__Volume__getVolumeId(
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c8ed10);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pSectorP__SpiResource__release(void );
#line 78
static error_t Stm25pSectorP__SpiResource__request(void );
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
static error_t Stm25pSectorP__Spi__powerDown(void );
#line 66
static error_t Stm25pSectorP__Spi__read(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len);
#line 136
static error_t Stm25pSectorP__Spi__sectorErase(uint8_t sector);
#line 55
static error_t Stm25pSectorP__Spi__powerUp(void );
#line 90
static error_t Stm25pSectorP__Spi__computeCrc(uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len);
#line 114
static error_t Stm25pSectorP__Spi__pageProgram(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Stm25pSectorP__ClientResource__granted(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
uint8_t arg_0x2ba081c94108);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t Stm25pSectorP__signalDone_task__postTask(void );
# 86 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
enum Stm25pSectorP____nesc_unnamed4314 {
#line 86
  Stm25pSectorP__signalDone_task = 2U
};
#line 86
typedef int Stm25pSectorP____nesc_sillytask_signalDone_task[Stm25pSectorP__signalDone_task];
#line 56
enum Stm25pSectorP____nesc_unnamed4315 {
  Stm25pSectorP__NO_CLIENT = 0xff
};







#line 60
typedef enum Stm25pSectorP____nesc_unnamed4316 {
  Stm25pSectorP__S_IDLE, 
  Stm25pSectorP__S_READ, 
  Stm25pSectorP__S_WRITE, 
  Stm25pSectorP__S_ERASE, 
  Stm25pSectorP__S_CRC
} Stm25pSectorP__stm25p_sector_state_t;
Stm25pSectorP__stm25p_sector_state_t Stm25pSectorP__m_state;





#line 69
typedef enum Stm25pSectorP____nesc_unnamed4317 {
  Stm25pSectorP__S_NONE, 
  Stm25pSectorP__S_START, 
  Stm25pSectorP__S_STOP
} Stm25pSectorP__stm25p_power_state_t;
Stm25pSectorP__stm25p_power_state_t Stm25pSectorP__m_power_state;

uint8_t Stm25pSectorP__m_client;
stm25p_addr_t Stm25pSectorP__m_addr;
stm25p_len_t Stm25pSectorP__m_len;
stm25p_len_t Stm25pSectorP__m_cur_len;
uint8_t *Stm25pSectorP__m_buf;
error_t Stm25pSectorP__m_error;
uint16_t Stm25pSectorP__m_crc;


static inline void Stm25pSectorP__signalDone(error_t error);


static error_t Stm25pSectorP__SplitControl__start(void );






static inline error_t Stm25pSectorP__SplitControl__stop(void );






static inline error_t Stm25pSectorP__ClientResource__request(uint8_t id);







static error_t Stm25pSectorP__ClientResource__release(uint8_t id);










static inline void Stm25pSectorP__Stm25pResource__granted(uint8_t id);




static inline uint8_t Stm25pSectorP__getVolumeId(uint8_t client);



static inline void Stm25pSectorP__SpiResource__granted(void );
#line 153
static inline stm25p_addr_t Stm25pSectorP__physicalAddr(uint8_t id, stm25p_addr_t addr);




static stm25p_len_t Stm25pSectorP__calcWriteLen(stm25p_addr_t addr);








static uint8_t Stm25pSectorP__Sector__getNumSectors(uint8_t id);



static error_t Stm25pSectorP__Sector__read(uint8_t id, stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len);










static inline void Stm25pSectorP__Spi__readDone(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error);



static error_t Stm25pSectorP__Sector__write(uint8_t id, stm25p_addr_t addr, 
uint8_t *buf, 
stm25p_len_t len);
#line 202
static inline void Stm25pSectorP__Spi__pageProgramDone(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error);









static error_t Stm25pSectorP__Sector__erase(uint8_t id, uint8_t sector, 
uint8_t num_sectors);
#line 226
static inline void Stm25pSectorP__Spi__sectorEraseDone(uint8_t sector, error_t error);







static inline error_t Stm25pSectorP__Sector__computeCrc(uint8_t id, uint16_t crc, 
stm25p_addr_t addr, 
stm25p_len_t len);









static inline void Stm25pSectorP__Spi__computeCrcDone(uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len, error_t error);




static inline void Stm25pSectorP__Spi__bulkEraseDone(error_t error);



static inline void Stm25pSectorP__signalDone(error_t error);




static inline void Stm25pSectorP__signalDone_task__runTask(void );
#line 284
static inline void Stm25pSectorP__ClientResource__default__granted(uint8_t id);
static inline void Stm25pSectorP__Sector__default__readDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, error_t error);
static inline void Stm25pSectorP__Sector__default__writeDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, error_t error);
static inline void Stm25pSectorP__Sector__default__eraseDone(uint8_t id, uint8_t sector, uint8_t num_sectors, error_t error);
static inline void Stm25pSectorP__Sector__default__computeCrcDone(uint8_t id, stm25p_addr_t addr, stm25p_len_t len, uint16_t crc, error_t error);
static inline volume_id_t Stm25pSectorP__Volume__default__getVolumeId(uint8_t id);
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
enum /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4318 {
#line 39
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[2U];
uint8_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d41020);
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d3f340);
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d3f340);
# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 73 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d44d40);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
enum /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4319 {
#line 75
  ArbiterP__0__grantedTask = 3U
};
#line 75
typedef int /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4320 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4321 {
#line 68
  ArbiterP__0__default_owner_id = 2U
};
#line 69
enum /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4322 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__reqResId;



static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id);
#line 108
static inline error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 130
static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 187
static inline void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 201
static inline void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id);
#line 213
static inline void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__start(void );
#line 109
static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__stop(void );
# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__startOneShot(uint32_t dt);




static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__stop(void );
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/PowerDownCleanup.nc"
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__PowerDownCleanup__cleanup(void );
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask__postTask(void );
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__release(void );
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask__postTask(void );
# 74 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__start(void );









static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__stop(void );
# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
enum /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0____nesc_unnamed4323 {
#line 69
  DeferredPowerManagerP__0__startTask = 4U
};
#line 69
typedef int /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0____nesc_sillytask_startTask[/*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask];







enum /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0____nesc_unnamed4324 {
#line 77
  DeferredPowerManagerP__0__timerTask = 5U
};
#line 77
typedef int /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0____nesc_sillytask_timerTask[/*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask];
#line 65
bool /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__stopping = FALSE;
bool /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__requested = FALSE;
bool /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__stopTimer = FALSE;

static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask__runTask(void );







static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask__runTask(void );



static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__requested(void );










static inline error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__default__start(void );







static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__startDone(error_t error);



static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__granted(void );



static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__fired(void );
#line 120
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__stopDone(error_t error);










static inline error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__default__stop(void );







static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
static error_t Stm25pSpiP__SpiPacket__send(
#line 48
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiByte.nc"
static uint8_t Stm25pSpiP__SpiByte__write(uint8_t tx);
# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void Stm25pSpiP__CSN__makeOutput(void );
#line 29
static void Stm25pSpiP__CSN__set(void );
static void Stm25pSpiP__CSN__clr(void );
# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
static void Stm25pSpiP__Spi__sectorEraseDone(uint8_t sector, error_t error);
#line 77
static void Stm25pSpiP__Spi__readDone(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error);
#line 125
static void Stm25pSpiP__Spi__pageProgramDone(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error);
#line 101
static void Stm25pSpiP__Spi__computeCrcDone(uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len, error_t error);
#line 159
static void Stm25pSpiP__Spi__bulkEraseDone(error_t error);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pSpiP__SpiResource__release(void );
#line 78
static error_t Stm25pSpiP__SpiResource__request(void );
#line 92
static void Stm25pSpiP__ClientResource__granted(void );
# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void Stm25pSpiP__Hold__makeOutput(void );
#line 29
static void Stm25pSpiP__Hold__set(void );
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
enum Stm25pSpiP____nesc_unnamed4325 {
  Stm25pSpiP__CRC_BUF_SIZE = 16
};









#line 60
typedef enum Stm25pSpiP____nesc_unnamed4326 {
  Stm25pSpiP__S_READ = 0x3, 
  Stm25pSpiP__S_PAGE_PROGRAM = 0x2, 
  Stm25pSpiP__S_SECTOR_ERASE = 0xd8, 
  Stm25pSpiP__S_BULK_ERASE = 0xc7, 
  Stm25pSpiP__S_WRITE_ENABLE = 0x6, 
  Stm25pSpiP__S_POWER_ON = 0xab, 
  Stm25pSpiP__S_DEEP_SLEEP = 0xb9
} Stm25pSpiP__stm25p_cmd_t;

uint8_t Stm25pSpiP__m_cmd[4];

bool Stm25pSpiP__m_is_writing = FALSE;
bool Stm25pSpiP__m_computing_crc = FALSE;

stm25p_addr_t Stm25pSpiP__m_addr;
uint8_t *Stm25pSpiP__m_buf;
stm25p_len_t Stm25pSpiP__m_len;
stm25p_addr_t Stm25pSpiP__m_cur_addr;
stm25p_len_t Stm25pSpiP__m_cur_len;
uint8_t Stm25pSpiP__m_crc_buf[Stm25pSpiP__CRC_BUF_SIZE];
uint16_t Stm25pSpiP__m_crc;

static error_t Stm25pSpiP__newRequest(bool write, stm25p_len_t cmd_len);
static void Stm25pSpiP__signalDone(error_t error);

static uint8_t Stm25pSpiP__sendCmd(uint8_t cmd, uint8_t len);
#line 100
static inline error_t Stm25pSpiP__Init__init(void );







static inline error_t Stm25pSpiP__ClientResource__request(void );







static inline error_t Stm25pSpiP__ClientResource__release(void );







static inline stm25p_len_t Stm25pSpiP__calcReadLen(void );



static inline error_t Stm25pSpiP__Spi__powerDown(void );




static inline error_t Stm25pSpiP__Spi__powerUp(void );




static error_t Stm25pSpiP__Spi__read(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len);







static inline error_t Stm25pSpiP__Spi__computeCrc(uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len);







static error_t Stm25pSpiP__Spi__pageProgram(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len);







static error_t Stm25pSpiP__Spi__sectorErase(uint8_t sector);










static error_t Stm25pSpiP__newRequest(bool write, stm25p_len_t cmd_len);










static void Stm25pSpiP__releaseAndRequest(void );




static void Stm25pSpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);
#line 238
static inline void Stm25pSpiP__SpiResource__granted(void );










static void Stm25pSpiP__signalDone(error_t error);
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e92df8, 
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e90e18);
# 180 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void );
#line 197
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void );
#line 97
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset);
#line 177
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void );
#line 224
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data);
#line 168
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void );
#line 192
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void );
#line 158
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void );
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e91be8);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e91be8);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x2ba081e956f8);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void );
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4327 {
#line 67
  Msp430SpiNoDmaP__0__signalDone_task = 6U
};
#line 67
typedef int /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task];
#line 56
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4328 {
  Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos;
uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client;

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );






static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id);







static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id);



static inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
#line 112
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id);

static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id);

static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void );
#line 144
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 166
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 183
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );




static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 85
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static volatile uint8_t HplMsp430Usart0P__IE1 __asm ("0x0000");
static volatile uint8_t HplMsp430Usart0P__ME1 __asm ("0x0004");
static volatile uint8_t HplMsp430Usart0P__IFG1 __asm ("0x0002");
static volatile uint8_t HplMsp430Usart0P__U0TCTL __asm ("0x0071");

static volatile uint8_t HplMsp430Usart0P__U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(18)))  ;




void sig_UART0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(16)))  ;
#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 207
static inline void HplMsp430Usart0P__Usart__disableUart(void );
#line 238
static inline void HplMsp430Usart0P__Usart__enableSpi(void );








static void HplMsp430Usart0P__Usart__disableSpi(void );








static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config);








static inline void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 330
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void );










static inline void HplMsp430Usart0P__Usart__clrRxIntr(void );



static inline void HplMsp430Usart0P__Usart__clrIntr(void );



static inline void HplMsp430Usart0P__Usart__disableRxIntr(void );







static inline void HplMsp430Usart0P__Usart__disableIntr(void );



static inline void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 382
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static uint8_t HplMsp430Usart0P__Usart__rx(void );
# 80 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2ba081fe8108, 
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2ba081fe8108);
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x2ba081fe7020);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id);
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4329 {
#line 39
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d41020);
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d3f340);
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d3f340);
# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void );
# 73 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void );
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ba081d44d40);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void );
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4330 {
#line 75
  ArbiterP__1__grantedTask = 7U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4331 {
#line 67
  ArbiterP__1__RES_CONTROLLED, ArbiterP__1__RES_GRANTING, ArbiterP__1__RES_IMM_GRANTING, ArbiterP__1__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4332 {
#line 68
  ArbiterP__1__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4333 {
#line 69
  ArbiterP__1__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;



static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id);
#line 108
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id);
#line 130
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 150
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );
#line 163
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );
#line 187
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
#line 199
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id);



static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );





static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 97 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__HplGeneralIO__set(void );




static void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__HplGeneralIO__clr(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__set(void );
static inline void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__clr(void );




static inline void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__makeOutput(void );
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__HplGeneralIO__set(void );
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__GeneralIO__set(void );





static inline void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBinderP.nc"
static inline volume_id_t /*FlashSamplerAppC.LogStorageC.BinderP*/Stm25pBinderP__0__Volume__getVolumeId(void );
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogConfigP.nc"
static inline bool /*FlashSamplerAppC.LogStorageC.ConfigP*/Stm25pLogConfigP__0__Circular__get(void );
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
static error_t Stm25pBlockP__Sector__read(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 112
static error_t Stm25pBlockP__Sector__erase(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint8_t sector, uint8_t num_sectors);
#line 133
static error_t Stm25pBlockP__Sector__computeCrc(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 133 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len);
#line 91
static error_t Stm25pBlockP__Sector__write(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70, 
# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
#line 56
static uint8_t Stm25pBlockP__Sector__getNumSectors(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211da70);
# 95 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
static void Stm25pBlockP__Read__computeCrcDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082125970, 
# 95 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
storage_addr_t addr, storage_len_t len, 
uint16_t crc, error_t error);
#line 67
static void Stm25pBlockP__Read__readDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082125970, 
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
storage_addr_t addr, 
#line 62
void * buf, 




storage_len_t len, 
error_t error);
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
static void Stm25pBlockP__Write__syncDone(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082120b68, 
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
error_t error);
#line 71
static void Stm25pBlockP__Write__writeDone(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082120b68, 
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
storage_addr_t addr, 
#line 66
void * buf, 




storage_len_t len, 
error_t error);
#line 91
static void Stm25pBlockP__Write__eraseDone(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba082120b68, 
# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
error_t error);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pBlockP__ClientResource__release(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211b530);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Stm25pBlockP__ClientResource__request(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
uint8_t arg_0x2ba08211b530);






enum Stm25pBlockP____nesc_unnamed4334 {
  Stm25pBlockP__NUM_BLOCKS = 1U
};








#line 55
typedef enum Stm25pBlockP____nesc_unnamed4335 {
  Stm25pBlockP__S_IDLE, 
  Stm25pBlockP__S_READ, 
  Stm25pBlockP__S_CRC, 
  Stm25pBlockP__S_WRITE, 
  Stm25pBlockP__S_SYNC, 
  Stm25pBlockP__S_ERASE
} Stm25pBlockP__stm25p_block_req_t;






#line 64
typedef struct Stm25pBlockP__stm25p_block_state_t {
  storage_addr_t addr;
  void *buf;
  storage_len_t len;
  Stm25pBlockP__stm25p_block_req_t req;
} Stm25pBlockP__stm25p_block_state_t;

Stm25pBlockP__stm25p_block_state_t Stm25pBlockP__m_block_state[Stm25pBlockP__NUM_BLOCKS];
Stm25pBlockP__stm25p_block_state_t Stm25pBlockP__m_req;

static error_t Stm25pBlockP__newRequest(uint8_t client);
static void Stm25pBlockP__signalDone(uint8_t id, uint16_t crc, error_t error);










static inline error_t Stm25pBlockP__Read__read(uint8_t id, storage_addr_t addr, void *buf, 
storage_len_t len);
#line 105
static error_t Stm25pBlockP__Write__write(uint8_t id, storage_addr_t addr, void *buf, 
storage_len_t len);







static inline error_t Stm25pBlockP__Write__sync(uint8_t id);




static inline error_t Stm25pBlockP__Write__erase(uint8_t id);




static error_t Stm25pBlockP__newRequest(uint8_t client);
#line 136
static void Stm25pBlockP__ClientResource__granted(uint8_t id);
#line 166
static inline void Stm25pBlockP__Sector__readDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error);



static inline void Stm25pBlockP__Sector__writeDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error);



static inline void Stm25pBlockP__Sector__eraseDone(uint8_t id, uint8_t sector, 
uint8_t num_sectors, 
error_t error);



static inline void Stm25pBlockP__Sector__computeCrcDone(uint8_t id, stm25p_addr_t addr, 
stm25p_len_t len, 
uint16_t crc, 
error_t error);



static void Stm25pBlockP__signalDone(uint8_t id, uint16_t crc, error_t error);
#line 222
static inline void Stm25pBlockP__Read__default__readDone(uint8_t id, storage_addr_t addr, void *buf, storage_len_t len, error_t error);
static inline void Stm25pBlockP__Read__default__computeCrcDone(uint8_t id, storage_addr_t addr, storage_len_t len, uint16_t crc, error_t error);
static inline void Stm25pBlockP__Write__default__writeDone(uint8_t id, storage_addr_t addr, void *buf, storage_len_t len, error_t error);
static inline void Stm25pBlockP__Write__default__eraseDone(uint8_t id, error_t error);
static inline void Stm25pBlockP__Write__default__syncDone(uint8_t id, error_t error);


static inline uint8_t Stm25pBlockP__Sector__default__getNumSectors(uint8_t id);
static inline error_t Stm25pBlockP__Sector__default__read(uint8_t id, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
static inline error_t Stm25pBlockP__Sector__default__write(uint8_t id, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len);
static inline error_t Stm25pBlockP__Sector__default__erase(uint8_t id, uint8_t sector, uint8_t num_sectors);
static inline error_t Stm25pBlockP__Sector__default__computeCrc(uint8_t id, uint16_t crc, storage_addr_t addr, storage_len_t len);
static inline error_t Stm25pBlockP__ClientResource__default__request(uint8_t id);
static inline error_t Stm25pBlockP__ClientResource__default__release(uint8_t id);
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBinderP.nc"
static inline volume_id_t /*FlashSamplerAppC.BlockStorageC.BinderP*/Stm25pBinderP__1__Volume__getVolumeId(void );
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Read.nc"
static void AdcP__Read__readDone(
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821b24e8, 
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Read.nc"
error_t result, AdcP__Read__val_t val);
# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadNow.nc"
static void AdcP__ReadNow__readDone(
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821af318, 
# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadNow.nc"
error_t result, AdcP__ReadNow__val_t val);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void AdcP__ResourceReadNow__granted(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821ad020);
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static AdcP__Config__adc_config_t AdcP__Config__getConfiguration(
# 48 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821a9b18);
# 189 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__getData(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821e5910);
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__configureSingle(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821e5910, 
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__release(
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x2ba0821ab318);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t AdcP__readDone__postTask(void );
# 136 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
enum AdcP____nesc_unnamed4336 {
#line 136
  AdcP__readDone = 8U
};
#line 136
typedef int AdcP____nesc_sillytask_readDone[AdcP__readDone];
#line 54
enum AdcP____nesc_unnamed4337 {
  AdcP__STATE_READ, 
  AdcP__STATE_READNOW, 
  AdcP__STATE_READNOW_INVALID_CONFIG
};


uint8_t AdcP__state;
uint8_t AdcP__owner;
uint16_t AdcP__value;

static error_t AdcP__configure(uint8_t client);
#line 80
static void AdcP__ResourceRead__granted(uint8_t client);
#line 98
static void AdcP__SubResourceReadNow__granted(uint8_t nowClient);
#line 136
static inline void AdcP__readDone__runTask(void );





static error_t AdcP__SingleChannel__singleDataReady(uint8_t client, uint16_t data);
#line 161
static inline uint16_t *AdcP__SingleChannel__multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples);







static inline error_t AdcP__ResourceRead__default__release(uint8_t client);

static inline void AdcP__Read__default__readDone(uint8_t client, error_t result, uint16_t val);




static inline void AdcP__ResourceReadNow__default__granted(uint8_t nowClient);
static inline void AdcP__ReadNow__default__readDone(uint8_t client, error_t result, uint16_t val);

static inline error_t AdcP__SingleChannel__default__getData(uint8_t client);




const msp430adc12_channel_config_t AdcP__defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
AdcP__Config__default__getConfiguration(uint8_t client);



static inline error_t AdcP__SingleChannel__default__configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config);
# 107 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static void Msp430Adc12ImplP__MultiChannel__dataReady(
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba0822210c8, 
# 107 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *buffer, uint16_t numSamples);
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t Msp430Adc12ImplP__HplAdc12__getCtl0(void );
#line 82
static adc12memctl_t Msp430Adc12ImplP__HplAdc12__getMCtl(uint8_t idx);
#line 106
static void Msp430Adc12ImplP__HplAdc12__resetIFGs(void );
#line 75
static void Msp430Adc12ImplP__HplAdc12__setMCtl(uint8_t idx, adc12memctl_t memControl);
#line 128
static void Msp430Adc12ImplP__HplAdc12__startConversion(void );
#line 51
static void Msp430Adc12ImplP__HplAdc12__setCtl0(adc12ctl0_t control0);
#line 89
static uint16_t Msp430Adc12ImplP__HplAdc12__getMem(uint8_t idx);





static void Msp430Adc12ImplP__HplAdc12__setIEFlags(uint16_t mask);
#line 123
static void Msp430Adc12ImplP__HplAdc12__stopConversion(void );
#line 57
static void Msp430Adc12ImplP__HplAdc12__setCtl1(adc12ctl1_t control1);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port64__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port64__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port64__selectModuleFunc(void );
# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA1__setEvent(uint16_t time);
# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void Msp430Adc12ImplP__ControlA0__setControl(msp430_compare_control_t control);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port62__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port62__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port62__selectModuleFunc(void );
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__memOverflow(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba08221f020);
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__conversionTimeOverflow(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba08221f020);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port67__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port67__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port67__selectModuleFunc(void );
#line 64
static void Msp430Adc12ImplP__Port60__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port60__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port60__selectModuleFunc(void );
#line 64
static void Msp430Adc12ImplP__Port65__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port65__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port65__selectModuleFunc(void );
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static void Msp430Adc12ImplP__TimerA__clear(void );


static void Msp430Adc12ImplP__TimerA__setClockSource(uint16_t clockSource);
#line 43
static void Msp430Adc12ImplP__TimerA__disableEvents(void );
#line 39
static void Msp430Adc12ImplP__TimerA__setMode(int mode);





static void Msp430Adc12ImplP__TimerA__setInputDivider(uint16_t inputDivider);
# 88 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
static uint8_t Msp430Adc12ImplP__ADCArbiterInfo__userId(void );
# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void Msp430Adc12ImplP__ControlA1__setControl(msp430_compare_control_t control);
# 227 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static uint16_t * Msp430Adc12ImplP__SingleChannel__multipleDataReady(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba082223ac0, 
# 227 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 206
static error_t Msp430Adc12ImplP__SingleChannel__singleDataReady(
# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x2ba082223ac0, 
# 206 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port63__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port63__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port63__selectModuleFunc(void );
# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA0__setEvent(uint16_t time);
# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port61__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port61__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port61__selectModuleFunc(void );
#line 64
static void Msp430Adc12ImplP__Port66__makeInput(void );
#line 85
static void Msp430Adc12ImplP__Port66__selectIOFunc(void );
#line 78
static void Msp430Adc12ImplP__Port66__selectModuleFunc(void );
# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
enum Msp430Adc12ImplP____nesc_unnamed4338 {
  Msp430Adc12ImplP__SINGLE_DATA = 1, 
  Msp430Adc12ImplP__SINGLE_DATA_REPEAT = 2, 
  Msp430Adc12ImplP__MULTIPLE_DATA = 4, 
  Msp430Adc12ImplP__MULTIPLE_DATA_REPEAT = 8, 
  Msp430Adc12ImplP__MULTI_CHANNEL = 16, 
  Msp430Adc12ImplP__CONVERSION_MODE_MASK = 0x1F, 

  Msp430Adc12ImplP__ADC_BUSY = 32, 
  Msp430Adc12ImplP__USE_TIMERA = 64, 
  Msp430Adc12ImplP__ADC_OVERFLOW = 128
};

uint8_t Msp430Adc12ImplP__state;

uint16_t Msp430Adc12ImplP__resultBufferLength;
uint16_t * Msp430Adc12ImplP__resultBufferStart;
uint16_t Msp430Adc12ImplP__resultBufferIndex;
uint8_t Msp430Adc12ImplP__numChannels;
uint8_t Msp430Adc12ImplP__clientID;

static inline error_t Msp430Adc12ImplP__Init__init(void );










static inline void Msp430Adc12ImplP__prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
#line 121
static inline void Msp430Adc12ImplP__startTimerA(void );
#line 142
static inline void Msp430Adc12ImplP__configureAdcPin(uint8_t inch);
#line 159
static void Msp430Adc12ImplP__resetAdcPin(uint8_t inch);
#line 176
static error_t Msp430Adc12ImplP__SingleChannel__configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config);
#line 271
static inline error_t Msp430Adc12ImplP__SingleChannel__configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies);
#line 394
static error_t Msp430Adc12ImplP__SingleChannel__getData(uint8_t id);
#line 503
static void Msp430Adc12ImplP__stopConversion(void );
#line 540
static inline void Msp430Adc12ImplP__TimerA__overflow(void );
static inline void Msp430Adc12ImplP__CompareA0__fired(void );
static inline void Msp430Adc12ImplP__CompareA1__fired(void );

static inline void Msp430Adc12ImplP__HplAdc12__conversionDone(uint16_t iv);
#line 640
static inline error_t Msp430Adc12ImplP__SingleChannel__default__singleDataReady(uint8_t id, uint16_t data);




static inline uint16_t *Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples);




static inline void Msp430Adc12ImplP__MultiChannel__default__dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples);

static inline void Msp430Adc12ImplP__Overflow__default__memOverflow(uint8_t id);
static inline void Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(uint8_t id);
# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
static void HplAdc12P__HplAdc12__conversionDone(uint16_t iv);
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static volatile uint16_t HplAdc12P__ADC12CTL0 __asm ("0x01A0");
static volatile uint16_t HplAdc12P__ADC12CTL1 __asm ("0x01A2");
static volatile uint16_t HplAdc12P__ADC12IFG __asm ("0x01A4");
static volatile uint16_t HplAdc12P__ADC12IE __asm ("0x01A6");
static volatile uint16_t HplAdc12P__ADC12IV __asm ("0x01A8");

static inline adc12ctl0_t HplAdc12P__int2adc12ctl0(uint16_t x)  ;

static inline uint16_t HplAdc12P__adc12ctl0cast2int(adc12ctl0_t x)  ;
static inline uint16_t HplAdc12P__adc12ctl1cast2int(adc12ctl1_t x)  ;
static inline uint8_t HplAdc12P__adc12memctl2int(adc12memctl_t x)  ;
static inline adc12memctl_t HplAdc12P__int2adc12memctl(uint8_t x)  ;

static inline void HplAdc12P__HplAdc12__setCtl0(adc12ctl0_t control0);



static inline void HplAdc12P__HplAdc12__setCtl1(adc12ctl1_t control1);



static inline adc12ctl0_t HplAdc12P__HplAdc12__getCtl0(void );







static inline void HplAdc12P__HplAdc12__setMCtl(uint8_t i, adc12memctl_t memCtl);



static inline adc12memctl_t HplAdc12P__HplAdc12__getMCtl(uint8_t i);



static inline uint16_t HplAdc12P__HplAdc12__getMem(uint8_t i);



static inline void HplAdc12P__HplAdc12__setIEFlags(uint16_t mask);


static inline void HplAdc12P__HplAdc12__resetIFGs(void );




static inline void HplAdc12P__HplAdc12__startConversion(void );




static void HplAdc12P__HplAdc12__stopConversion(void );
#line 118
static inline bool HplAdc12P__HplAdc12__isBusy(void );

void sig_ADC_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(14)))  ;
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/RoundRobinResourceQueueC.nc"
enum /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0____nesc_unnamed4339 {
  RoundRobinResourceQueueC__0__NO_ENTRY = 0xFF, 
  RoundRobinResourceQueueC__0__SIZE = 3U ? (3U - 1) / 8 + 1 : 0
};

uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__SIZE];
uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last = 0;

static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__clearEntry(uint8_t id);



static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init(void );




static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue(void );
#line 87
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(resource_client_id_t id);
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237dc80);
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237a220);
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237a220);
# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void );
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__granted(
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2ba08237e9b0);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void );
# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_unnamed4340 {
#line 69
  SimpleArbiterP__0__grantedTask = 9U
};
#line 69
typedef int /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_sillytask_grantedTask[/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask];
#line 62
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_unnamed4341 {
#line 62
  SimpleArbiterP__0__RES_IDLE = 0, SimpleArbiterP__0__RES_GRANTING = 1, SimpleArbiterP__0__RES_BUSY = 2
};
#line 63
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_unnamed4342 {
#line 63
  SimpleArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_IDLE;
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId;



static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id);
#line 97
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id);
#line 137
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId(void );
#line 155
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );









static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id);



static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t Msp430RefVoltGeneratorP__HplAdc12__getCtl0(void );
#line 118
static bool Msp430RefVoltGeneratorP__HplAdc12__isBusy(void );
#line 51
static void Msp430RefVoltGeneratorP__HplAdc12__setCtl0(adc12ctl0_t control0);
# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP__SwitchOffTimer__stop(void );
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(error_t error);
#line 117
static void Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(error_t error);
#line 92
static void Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(error_t error);
#line 117
static void Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(error_t error);
# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP__SwitchOnTimer__stop(void );
# 65 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
#line 52
typedef enum Msp430RefVoltGeneratorP____nesc_unnamed4343 {

  Msp430RefVoltGeneratorP__GENERATOR_OFF = 0, 

  Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE = 1, 
  Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE = 2, 

  Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING = 3, 
  Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING = 4, 

  Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING = 5, 
  Msp430RefVoltGeneratorP__REFERENCE_2_5V_OFF_PENDING = 6
} 
Msp430RefVoltGeneratorP__state_t;

Msp430RefVoltGeneratorP__state_t Msp430RefVoltGeneratorP__m_state;


static error_t Msp430RefVoltGeneratorP__switchOn(uint8_t level);
static error_t Msp430RefVoltGeneratorP__switchOff(void );
static void Msp430RefVoltGeneratorP__signalStartDone(Msp430RefVoltGeneratorP__state_t state, error_t result);
static void Msp430RefVoltGeneratorP__signalStopDone(Msp430RefVoltGeneratorP__state_t state, error_t result);
static error_t Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__state_t targetState);
static inline error_t Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__state_t nextState);


static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__start(void );



static inline error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__start(void );



static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__stop(void );







static error_t Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__state_t targetState);
#line 126
static inline error_t Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__state_t nextState);
#line 153
static void Msp430RefVoltGeneratorP__signalStartDone(Msp430RefVoltGeneratorP__state_t state, error_t result);






static void Msp430RefVoltGeneratorP__signalStopDone(Msp430RefVoltGeneratorP__state_t state, error_t result);







static inline void Msp430RefVoltGeneratorP__SwitchOnTimer__fired(void );
#line 185
static inline void Msp430RefVoltGeneratorP__SwitchOffTimer__fired(void );
#line 213
static inline void Msp430RefVoltGeneratorP__HplAdc12__conversionDone(uint16_t iv);



static error_t Msp430RefVoltGeneratorP__switchOn(uint8_t level);
#line 236
static error_t Msp430RefVoltGeneratorP__switchOff(void );
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static Msp430RefVoltArbiterImplP__Config__adc_config_t Msp430RefVoltArbiterImplP__Config__getConfiguration(
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba08242d248);
# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltArbiterImplP__RefVolt_2_5V__start(void );
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__release(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba0823db538);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__request(
# 40 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba0823db538);
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP__ClientResource__granted(
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x2ba0823dc298);
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t Msp430RefVoltArbiterImplP__switchOff__postTask(void );
# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__start(void );
#line 109
static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__stop(void );
# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
enum Msp430RefVoltArbiterImplP____nesc_unnamed4344 {
#line 51
  Msp430RefVoltArbiterImplP__switchOff = 10U
};
#line 51
typedef int Msp430RefVoltArbiterImplP____nesc_sillytask_switchOff[Msp430RefVoltArbiterImplP__switchOff];
#line 46
enum Msp430RefVoltArbiterImplP____nesc_unnamed4345 {
  Msp430RefVoltArbiterImplP__NO_OWNER = 0xFF
};
uint8_t Msp430RefVoltArbiterImplP__syncOwner = Msp430RefVoltArbiterImplP__NO_OWNER;



static inline error_t Msp430RefVoltArbiterImplP__ClientResource__request(uint8_t client);
#line 70
static void Msp430RefVoltArbiterImplP__AdcResource__granted(uint8_t client);
#line 98
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error_t error);








static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error_t error);








static error_t Msp430RefVoltArbiterImplP__ClientResource__release(uint8_t client);
#line 136
static inline void Msp430RefVoltArbiterImplP__switchOff__runTask(void );










static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error_t error);



static inline void Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error_t error);








static inline void Msp430RefVoltArbiterImplP__ClientResource__default__granted(uint8_t client);
static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__request(uint8_t client);








static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__release(uint8_t client);
const msp430adc12_channel_config_t Msp430RefVoltArbiterImplP__defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP__Config__default__getConfiguration(uint8_t client);
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__adc_config_t /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__getConfiguration(void );
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration(void );
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t AdcStreamP__bufferDone__postTask(void );
#line 56
static error_t AdcStreamP__readStreamDone__postTask(void );
#line 56
static error_t AdcStreamP__readStreamFail__postTask(void );
# 98 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static AdcStreamP__Alarm__size_type AdcStreamP__Alarm__getNow(void );
#line 92
static void AdcStreamP__Alarm__startAt(AdcStreamP__Alarm__size_type t0, AdcStreamP__Alarm__size_type dt);
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static AdcStreamP__AdcConfigure__adc_config_t AdcStreamP__AdcConfigure__getConfiguration(
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082462318);
# 189 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__getData(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082465b90);
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__configureSingle(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082465b90, 
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 138
static error_t AdcStreamP__SingleChannel__configureMultiple(
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082465b90, 
# 138 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
static void AdcStreamP__ReadStream__bufferDone(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082468b08, 
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
AdcStreamP__ReadStream__val_t * buf, 



uint16_t count);
#line 102
static void AdcStreamP__ReadStream__readDone(
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x2ba082468b08, 
# 102 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 119 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
enum AdcStreamP____nesc_unnamed4346 {
#line 119
  AdcStreamP__readStreamDone = 11U
};
#line 119
typedef int AdcStreamP____nesc_sillytask_readStreamDone[AdcStreamP__readStreamDone];
#line 135
enum AdcStreamP____nesc_unnamed4347 {
#line 135
  AdcStreamP__readStreamFail = 12U
};
#line 135
typedef int AdcStreamP____nesc_sillytask_readStreamFail[AdcStreamP__readStreamFail];
#line 156
enum AdcStreamP____nesc_unnamed4348 {
#line 156
  AdcStreamP__bufferDone = 13U
};
#line 156
typedef int AdcStreamP____nesc_sillytask_bufferDone[AdcStreamP__bufferDone];
#line 58
enum AdcStreamP____nesc_unnamed4349 {
  AdcStreamP__NSTREAM = 1U
};




uint8_t AdcStreamP__client = AdcStreamP__NSTREAM;


struct AdcStreamP__list_entry_t {
  uint16_t count;
  struct AdcStreamP__list_entry_t * next;
};
struct AdcStreamP__list_entry_t *AdcStreamP__bufferQueue[AdcStreamP__NSTREAM];
struct AdcStreamP__list_entry_t * *AdcStreamP__bufferQueueEnd[AdcStreamP__NSTREAM];
uint16_t * AdcStreamP__lastBuffer;
#line 74
uint16_t AdcStreamP__lastCount;

uint16_t AdcStreamP__count;
uint16_t * AdcStreamP__buffer;
uint16_t * AdcStreamP__pos;
uint32_t AdcStreamP__now;
#line 79
uint32_t AdcStreamP__period;
bool AdcStreamP__periodModified;


static inline error_t AdcStreamP__Init__init(void );








static inline void AdcStreamP__sampleSingle(void );



static error_t AdcStreamP__postBuffer(uint8_t c, uint16_t *buf, uint16_t n);
#line 115
static inline error_t AdcStreamP__ReadStream__postBuffer(uint8_t c, uint16_t *buf, uint16_t n);



static inline void AdcStreamP__readStreamDone__runTask(void );
#line 135
static inline void AdcStreamP__readStreamFail__runTask(void );
#line 156
static inline void AdcStreamP__bufferDone__runTask(void );
#line 168
static inline void AdcStreamP__nextAlarm(void );




static inline void AdcStreamP__Alarm__fired(void );



static error_t AdcStreamP__nextBuffer(bool startNextAlarm);
#line 206
static void AdcStreamP__nextMultiple(uint8_t c);
#line 221
static error_t AdcStreamP__ReadStream__read(uint8_t c, uint32_t usPeriod);
#line 242
static error_t AdcStreamP__SingleChannel__singleDataReady(uint8_t streamClient, uint16_t data);
#line 281
static uint16_t *AdcStreamP__SingleChannel__multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length);
#line 304
const msp430adc12_channel_config_t AdcStreamP__defaultConfig = { 
.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };

static inline const msp430adc12_channel_config_t *AdcStreamP__AdcConfigure__default__getConfiguration(uint8_t c);



static inline error_t AdcStreamP__SingleChannel__default__configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies);



static inline error_t AdcStreamP__SingleChannel__default__getData(uint8_t c);



static inline error_t AdcStreamP__SingleChannel__default__configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config);
# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 33
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 92
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__get(void );
# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0;
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt;

enum /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1____nesc_unnamed4350 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 5, 
  TransformAlarmC__1__MAX_DELAY = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type )1 << /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 96
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__set_alarm(void );
#line 136
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type dt);
#line 151
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 166
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__postBuffer(
# 26 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824fb148, 
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__val_t * buf, 




uint16_t count);









static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__read(
# 26 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824fb148, 
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
uint32_t usPeriod);










static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__bufferDone(
# 24 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824ff020, 
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__readDone(
# 24 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824ff020, 
# 102 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__release(
# 27 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824f7020);
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__request(
# 27 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x2ba0824f7020);



uint32_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__period[1U];

static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__postBuffer(uint8_t client, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count);




static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__read(uint8_t client, uint32_t usPeriod);









static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count);




static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(uint8_t client, error_t result, uint32_t actualPeriod);





static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(uint8_t client);




static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__request(uint8_t client);


static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(uint8_t client);
#line 79
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count);



static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(uint8_t client, error_t result, uint32_t actualPeriod);
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__adc_config_t /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__getConfiguration(void );
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration(void );
# 39 "Msp430AxisXP.nc"
const msp430adc12_channel_config_t Msp430AxisXP__config = { 
.inch = INPUT_CHANNEL_A0, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_2_5, 
.adc12ssel = SHT_SOURCE_ADC12OSC, 
.adc12div = SHT_CLOCK_DIV_2, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };


static inline const msp430adc12_channel_config_t *Msp430AxisXP__AdcConfigure__getConfiguration(void );
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
static /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__adc_config_t /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__getConfiguration(void );
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__getConfiguration(void );
# 212 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 540 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__TimerA__overflow(void )
#line 540
{
}

# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 37
  Msp430Adc12ImplP__TimerA__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 37
}
#line 37
# 126 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x2ba0816dac98){
#line 28
  switch (arg_0x2ba0816dac98) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x2ba0816dac98);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 28
}
#line 28
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4351 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

# 541 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__CompareA0__fired(void )
#line 541
{
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 34
  Msp430Adc12ImplP__CompareA0__fired();
#line 34
}
#line 34
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4352 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

# 542 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__CompareA1__fired(void )
#line 542
{
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 34
  Msp430Adc12ImplP__CompareA1__fired();
#line 34
}
#line 34
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4353 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 34
}
#line 34
# 120 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 28
}
#line 28
# 115 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 28
}
#line 28
# 185 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 103 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 166 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

#line 166
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 71
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__overflow();
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 71
}
#line 71
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 37
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 37
}
#line 37
# 126 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 70 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 67
}
#line 67
# 151 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4354 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 86 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 70 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 119 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 32 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4355 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}

# 324 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__SingleChannel__default__getData(uint8_t c)
{
  return FAIL;
}

# 189 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP__SingleChannel__getData(uint8_t arg_0x2ba082465b90){
#line 189
  unsigned char __nesc_result;
#line 189

#line 189
  switch (arg_0x2ba082465b90) {
#line 189
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 189
      __nesc_result = Msp430Adc12ImplP__SingleChannel__getData(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 189
      break;
#line 189
    default:
#line 189
      __nesc_result = AdcStreamP__SingleChannel__default__getData(arg_0x2ba082465b90);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return __nesc_result;
#line 189
}
#line 189
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__sampleSingle(void )
#line 92
{
  AdcStreamP__SingleChannel__getData(AdcStreamP__client);
}

#line 173
static inline void AdcStreamP__Alarm__fired(void )
#line 173
{
  AdcStreamP__sampleSingle();
}

# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 67
  AdcStreamP__Alarm__fired();
#line 67
}
#line 67
# 151 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 67
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 34
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4356 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}

# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 7);
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port67__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput();
#line 64
}
#line 64
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 7;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port67__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 6);
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port66__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput();
#line 64
}
#line 64
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 6;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port66__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 5);
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port65__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput();
#line 64
}
#line 64
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 5;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port65__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 4);
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port64__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput();
#line 64
}
#line 64
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 4;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port64__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 3);
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port63__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput();
#line 64
}
#line 64
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 3;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port63__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 2);
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port62__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput();
#line 64
}
#line 64
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 2;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port62__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 1);
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port61__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput();
#line 64
}
#line 64
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 1;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port61__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc();
#line 78
}
#line 78
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )54U &= ~(0x01 << 0);
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port60__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput();
#line 64
}
#line 64
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )55U |= 0x01 << 0;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port60__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc();
#line 78
}
#line 78
# 142 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__configureAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP__Port60__selectModuleFunc();
#line 147
      Msp430Adc12ImplP__Port60__makeInput();
#line 147
      break;
      case 1: Msp430Adc12ImplP__Port61__selectModuleFunc();
#line 148
      Msp430Adc12ImplP__Port61__makeInput();
#line 148
      break;
      case 2: Msp430Adc12ImplP__Port62__selectModuleFunc();
#line 149
      Msp430Adc12ImplP__Port62__makeInput();
#line 149
      break;
      case 3: Msp430Adc12ImplP__Port63__selectModuleFunc();
#line 150
      Msp430Adc12ImplP__Port63__makeInput();
#line 150
      break;
      case 4: Msp430Adc12ImplP__Port64__selectModuleFunc();
#line 151
      Msp430Adc12ImplP__Port64__makeInput();
#line 151
      break;
      case 5: Msp430Adc12ImplP__Port65__selectModuleFunc();
#line 152
      Msp430Adc12ImplP__Port65__makeInput();
#line 152
      break;
      case 6: Msp430Adc12ImplP__Port66__selectModuleFunc();
#line 153
      Msp430Adc12ImplP__Port66__makeInput();
#line 153
      break;
      case 7: Msp430Adc12ImplP__Port67__selectModuleFunc();
#line 154
      Msp430Adc12ImplP__Port67__makeInput();
#line 154
      break;
    }
}

# 100 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P__HplAdc12__startConversion(void )
#line 100
{
  HplAdc12P__ADC12CTL0 |= 0x0010;
  HplAdc12P__ADC12CTL0 |= 0x0001 + 0x0002;
}

# 128 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__startConversion(void ){
#line 128
  HplAdc12P__HplAdc12__startConversion();
#line 128
}
#line 128
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__setMode(int mode){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(mode);
#line 39
}
#line 39
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4357 {
#line 46
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 89
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)
{
  * (volatile uint16_t * )356U = /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(x);
}

# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void Msp430Adc12ImplP__ControlA1__setControl(msp430_compare_control_t control){
#line 35
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(control);
#line 35
}
#line 35
# 121 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__startTimerA(void )
{

  msp430_compare_control_t ccSetSHI = { 
  .ccifg = 0, .cov = 0, .out = 1, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccRSOutmod = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 7, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP__ControlA1__setControl(ccResetSHI);
  Msp430Adc12ImplP__ControlA1__setControl(ccSetSHI);

  Msp430Adc12ImplP__ControlA1__setControl(ccRSOutmod);
  Msp430Adc12ImplP__TimerA__setMode(MSP430TIMER_UP_MODE);
}

# 119 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 32 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 92
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4358 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4359 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4360 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4361 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 120 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 28
}
#line 28
# 113 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 46
  SchedulerBasicP__Scheduler__init();
#line 46
}
#line 46
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 34
}
#line 34
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 29 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 29
}
#line 29
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 34
}
#line 34
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 29 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 29
}
#line 29
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 34
}
#line 34
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 29 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 29
}
#line 29
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 35
}
#line 35
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = LedsP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 36 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TA0R;

#line 8
  while (TA0R - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = MotePlatformC__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 152 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 140
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TA0CTL = 0x0020 | (Msp430ClockP__TA0CTL & ~(0x0020 | 0x0010));
}

#line 104
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 134
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 32 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 32
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 32
}
#line 32
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP__TA0CTL = 0x0200 | 0x0002;
}

#line 129
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 31
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 31
}
#line 31
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~(1 << 1);
}

#line 124
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 30
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 30
}
#line 30
# 170 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 56
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 119
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 29 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 29
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 29
}
#line 29
# 218 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TA0CTL = 0x0004;
  Msp430ClockP__TA0IV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = Msp430ClockP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 10 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = PlatformP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 36 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
inline static error_t AccelSamplerC__BlockWrite__write(storage_addr_t addr, void * buf, storage_len_t len){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = Stm25pBlockP__Write__write(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID, addr, buf, len);
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle();
#line 44
}
#line 44
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle();
}

# 31 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 31
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle();
#line 31
}
#line 31
# 103 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Toggle(void )
#line 103
{
  LedsP__Led2__toggle();
  ;
#line 105
  ;
}

# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
inline static void AccelSamplerC__Leds__led2Toggle(void ){
#line 89
  LedsP__Leds__led2Toggle();
#line 89
}
#line 89
# 39 "AccelSamplerC.nc"
static inline void AccelSamplerC__Accel__bufferDone(error_t ok, uint16_t *buf, uint16_t count)
#line 39
{

  AccelSamplerC__Leds__led2Toggle();
  AccelSamplerC__BlockWrite__write(AccelSamplerC__nbuffers * sizeof AccelSamplerC__buffer1, buf, sizeof AccelSamplerC__buffer1);
}

# 79 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count)
{
}

# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__bufferDone(uint8_t arg_0x2ba0824ff020, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, uint16_t count){
#line 89
  switch (arg_0x2ba0824ff020) {
#line 89
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 89
      AccelSamplerC__Accel__bufferDone(result, buf, count);
#line 89
      break;
#line 89
    default:
#line 89
      /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(arg_0x2ba0824ff020, result, buf, count);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 48 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__bufferDone(client, result, buf, count);
}

# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
inline static void AdcStreamP__ReadStream__bufferDone(uint8_t arg_0x2ba082468b08, error_t result, AdcStreamP__ReadStream__val_t * buf, uint16_t count){
#line 89
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(arg_0x2ba082468b08, result, buf, count);
#line 89
}
#line 89
# 156 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__bufferDone__runTask(void )
#line 156
{
  uint16_t *b;
#line 157
  uint16_t c;

#line 158
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      b = AdcStreamP__lastBuffer;
      c = AdcStreamP__lastCount;
      AdcStreamP__lastBuffer = (void *)0;
    }
#line 163
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP__ReadStream__bufferDone(AdcStreamP__client, SUCCESS, b, c);
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pSectorP__Stm25pResource__request(uint8_t arg_0x2ba081c8ca68){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(arg_0x2ba081c8ca68);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 102 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline error_t Stm25pSectorP__ClientResource__request(uint8_t id)
#line 102
{
  return Stm25pSectorP__Stm25pResource__request(id);
}

# 234 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline error_t Stm25pBlockP__ClientResource__default__request(uint8_t id)
#line 234
{
#line 234
  return FAIL;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pBlockP__ClientResource__request(uint8_t arg_0x2ba08211b530){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0x2ba08211b530) {
#line 78
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 78
      __nesc_result = Stm25pSectorP__ClientResource__request(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = Stm25pBlockP__ClientResource__default__request(arg_0x2ba08211b530);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 201 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 201
{
}

# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
inline static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(uint8_t arg_0x2ba081d41020){
#line 43
    /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(arg_0x2ba081d41020);
#line 43
}
#line 43
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] != /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY || /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
        if (/*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = id;
          }
        else {
#line 78
          /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail] = id;
          }
#line 79
        /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 81 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__requested(void )
#line 81
{
  if (/*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__stopping == FALSE) {
      /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__stopTimer = TRUE;
      /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask__postTask();
    }
  else {
#line 86
    /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__requested = TRUE;
    }
}

# 73 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void ){
#line 73
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__requested();
#line 73
}
#line 73
# 24 "SummarizerC.nc"
static inline void SummarizerC__Summary__summarize(void )
#line 24
{

  SummarizerC__index = 0;
  SummarizerC__nextSummarySample();
}

# 12 "Summary.nc"
inline static void FlashSamplerC__Summary__summarize(void ){
#line 12
  SummarizerC__Summary__summarize();
#line 12
}
#line 12
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr();
#line 39
}
#line 39
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr();
}

# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__clr(void ){
#line 30
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr();
#line 30
}
#line 30
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1On(void )
#line 78
{
  LedsP__Led1__clr();
  ;
#line 80
  ;
}

# 61 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
inline static void FlashSamplerC__Leds__led1On(void ){
#line 61
  LedsP__Leds__led1On();
#line 61
}
#line 61
# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Off(void )
#line 68
{
  LedsP__Led0__set();
  ;
#line 70
  ;
}

# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
inline static void FlashSamplerC__Leds__led0Off(void ){
#line 50
  LedsP__Leds__led0Off();
#line 50
}
#line 50
# 33 "FlashSamplerC.nc"
static inline void FlashSamplerC__Sample__sampled(error_t error)
#line 33
{
  FlashSamplerC__Leds__led0Off();
  FlashSamplerC__Leds__led1On();
  FlashSamplerC__Summary__summarize();
}

# 13 "Sample.nc"
inline static void AccelSamplerC__Sample__sampled(error_t error){
#line 13
  FlashSamplerC__Sample__sampled(error);
#line 13
}
#line 13
# 62 "AccelSamplerC.nc"
static inline void AccelSamplerC__Accel__readDone(error_t ok, uint32_t usActualPeriod)
#line 62
{


  AccelSamplerC__Sample__sampled(FAIL);
}

# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
}

# 102 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__readDone(uint8_t arg_0x2ba0824ff020, error_t result, uint32_t usActualPeriod){
#line 102
  switch (arg_0x2ba0824ff020) {
#line 102
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 102
      AccelSamplerC__Accel__readDone(result, usActualPeriod);
#line 102
      break;
#line 102
    default:
#line 102
      /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(arg_0x2ba0824ff020, result, usActualPeriod);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(uint8_t client)
#line 67
{
#line 67
  return FAIL;
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__release(uint8_t arg_0x2ba0824f7020){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x2ba0824f7020) {
#line 110
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 110
      __nesc_result = Msp430RefVoltArbiterImplP__ClientResource__release(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(arg_0x2ba0824f7020);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__release(client);
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__readDone(client, result, actualPeriod);
}

# 102 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
inline static void AdcStreamP__ReadStream__readDone(uint8_t arg_0x2ba082468b08, error_t result, uint32_t usActualPeriod){
#line 102
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(arg_0x2ba082468b08, result, usActualPeriod);
#line 102
}
#line 102
# 135 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__readStreamFail__runTask(void )
#line 135
{

  struct AdcStreamP__list_entry_t *entry;
  uint8_t c = AdcStreamP__client;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 140
    entry = AdcStreamP__bufferQueue[c];
#line 140
    __nesc_atomic_end(__nesc_atomic); }
  for (; entry; entry = entry->next) {
      uint16_t tmp_count __attribute((unused))  = entry->count;

#line 143
      AdcStreamP__ReadStream__bufferDone(c, FAIL, (uint16_t * )entry, entry->count);
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP__bufferQueue[c] = (void *)0;
      AdcStreamP__bufferQueueEnd[c] = &AdcStreamP__bufferQueue[c];
    }
#line 150
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP__client = AdcStreamP__NSTREAM;
  AdcStreamP__ReadStream__readDone(c, FAIL, 0);
}

# 170 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__release(uint8_t client)
#line 170
{
#line 170
  return FAIL;
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Msp430RefVoltArbiterImplP__AdcResource__release(uint8_t arg_0x2ba0823db538){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x2ba0823db538) {
#line 110
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 110
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(/*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 110
      break;
#line 110
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 110
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 110
      break;
#line 110
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 110
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(/*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = Msp430RefVoltArbiterImplP__AdcResource__default__release(arg_0x2ba0823db538);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/RoundRobinResourceQueueC.nc"
static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty(void )
#line 56
{
  int i;

  /* atomic removed: atomic calls only */
#line 58
  {
    for (i = 0; i < sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ; i++) 
      if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[i] > 0) {
          unsigned char __nesc_temp = 
#line 60
          FALSE;

#line 60
          return __nesc_temp;
        }
#line 61
    {
      unsigned char __nesc_temp = 
#line 61
      TRUE;

#line 61
      return __nesc_temp;
    }
  }
}

# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/RoundRobinResourceQueueC.nc"
static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__clearEntry(uint8_t id)
#line 47
{
  /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[id / 8] &= ~(1 << id % 8);
}

#line 69
static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue(void )
#line 69
{
  int i;

  /* atomic removed: atomic calls only */
#line 71
  {
    for (i = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last + 1; ; i++) {
        if (i == 3U) {
          i = 0;
          }
#line 75
        if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(i)) {
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__clearEntry(i);
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last = i;
            {
              unsigned char __nesc_temp = 
#line 78
              i;

#line 78
              return __nesc_temp;
            }
          }
#line 80
        if (i == /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last) {
          break;
          }
      }
#line 83
    {
      unsigned char __nesc_temp = 
#line 83
      /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__NO_ENTRY;

#line 83
      return __nesc_temp;
    }
  }
}

# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 173 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 173
{
}

# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x2ba08237a220){
#line 55
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x2ba08237a220);
#line 55
}
#line 55
# 86 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline error_t Stm25pBlockP__Read__read(uint8_t id, storage_addr_t addr, void *buf, 
storage_len_t len)
#line 87
{
  Stm25pBlockP__m_req.req = Stm25pBlockP__S_READ;
  Stm25pBlockP__m_req.addr = addr;
  Stm25pBlockP__m_req.buf = buf;
  Stm25pBlockP__m_req.len = len;
  return Stm25pBlockP__newRequest(id);
}

# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
inline static error_t SummarizerC__BlockRead__read(storage_addr_t addr, void * buf, storage_len_t len){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = Stm25pBlockP__Read__read(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID, addr, buf, len);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 119 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__readStreamDone__runTask(void )
#line 119
{
  uint8_t c = AdcStreamP__client;
  uint32_t actualPeriod = AdcStreamP__period;

#line 122
  if (AdcStreamP__periodModified) {
    actualPeriod = AdcStreamP__period - AdcStreamP__period % 1000;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP__bufferQueue[c] = (void *)0;
      AdcStreamP__bufferQueueEnd[c] = &AdcStreamP__bufferQueue[c];
    }
#line 129
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP__client = AdcStreamP__NSTREAM;
  AdcStreamP__ReadStream__readDone(c, SUCCESS, actualPeriod);
}

# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t Msp430RefVoltArbiterImplP__switchOff__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(Msp430RefVoltArbiterImplP__switchOff);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 153 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP__SwitchOnTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(2U);
#line 67
}
#line 67
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 98 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 125 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 125
  unsigned long __nesc_result;
#line 125

#line 125
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 148 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(3U, dt);
#line 62
}
#line 62
# 126 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__state_t nextState)
#line 126
{
  error_t result;

  if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__GENERATOR_OFF) {
    result = EALREADY;
    }
  else {
#line 131
    if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE) {
        if ((result = Msp430RefVoltGeneratorP__switchOff()) == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = nextState;
            Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(20);
          }
      }
    else {
#line 136
      if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING) {
          if ((result = Msp430RefVoltGeneratorP__switchOff()) == SUCCESS) {

              Msp430RefVoltGeneratorP__state_t oldState = Msp430RefVoltGeneratorP__m_state;

#line 140
              Msp430RefVoltGeneratorP__SwitchOnTimer__stop();
              Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__GENERATOR_OFF;
              Msp430RefVoltGeneratorP__signalStartDone(oldState, FAIL);
              Msp430RefVoltGeneratorP__signalStopDone(nextState, SUCCESS);
            }
        }
      else {
#line 145
        if (Msp430RefVoltGeneratorP__m_state == nextState) {
          result = SUCCESS;
          }
        else {
#line 148
          result = EBUSY;
          }
        }
      }
    }
#line 150
  return result;
}

#line 86
static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__stop(void )
#line 86
{
  return Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING);
}

# 109 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__stop(void ){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_1_5V__stop();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 136 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__switchOff__runTask(void )
{

  if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {
      if (Msp430RefVoltArbiterImplP__RefVolt_1_5V__stop() == SUCCESS) {
          Msp430RefVoltArbiterImplP__syncOwner = Msp430RefVoltArbiterImplP__NO_OWNER;
        }
      else {
#line 143
        Msp430RefVoltArbiterImplP__switchOff__postTask();
        }
    }
}

# 118 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline bool HplAdc12P__HplAdc12__isBusy(void )
#line 118
{
#line 118
  return HplAdc12P__ADC12CTL1 & 0x0001;
}

# 118 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static bool Msp430RefVoltGeneratorP__HplAdc12__isBusy(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = HplAdc12P__HplAdc12__isBusy();
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 57 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  adc12ctl0_t HplAdc12P__int2adc12ctl0(uint16_t x)
#line 57
{
#line 57
  union __nesc_unnamed4362 {
#line 57
    uint16_t f;
#line 57
    adc12ctl0_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

#line 72
static inline adc12ctl0_t HplAdc12P__HplAdc12__getCtl0(void )
#line 72
{
  return HplAdc12P__int2adc12ctl0(HplAdc12P__ADC12CTL0);
}

# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static adc12ctl0_t Msp430RefVoltGeneratorP__HplAdc12__getCtl0(void ){
#line 63
  struct __nesc_unnamed4254 __nesc_result;
#line 63

#line 63
  __nesc_result = HplAdc12P__HplAdc12__getCtl0();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  uint16_t HplAdc12P__adc12ctl0cast2int(adc12ctl0_t x)
#line 59
{
#line 59
  union __nesc_unnamed4363 {
#line 59
    adc12ctl0_t f;
#line 59
    uint16_t t;
  } 
#line 59
  c = { .f = x };

#line 59
  return c.t;
}



static inline void HplAdc12P__HplAdc12__setCtl0(adc12ctl0_t control0)
#line 64
{
  HplAdc12P__ADC12CTL0 = HplAdc12P__adc12ctl0cast2int(control0);
}

# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430RefVoltGeneratorP__HplAdc12__setCtl0(adc12ctl0_t control0){
#line 51
  HplAdc12P__HplAdc12__setCtl0(control0);
#line 51
}
#line 51
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 50 "Msp430AxisXP.nc"
static inline const msp430adc12_channel_config_t *Msp430AxisXP__AdcConfigure__getConfiguration(void )
{
  return &Msp430AxisXP__config;
}

# 186 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static inline const msp430adc12_channel_config_t *
AdcP__Config__default__getConfiguration(uint8_t client)
{
  return &AdcP__defaultConfig;
}

# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
inline static AdcP__Config__adc_config_t AdcP__Config__getConfiguration(uint8_t arg_0x2ba0821a9b18){
#line 58
  struct __nesc_unnamed4289 const *__nesc_result;
#line 58

#line 58
  switch (arg_0x2ba0821a9b18) {
#line 58
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 58
      __nesc_result = Msp430AxisXP__AdcConfigure__getConfiguration();
#line 58
      break;
#line 58
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT:
#line 58
      __nesc_result = Msp430AxisXP__AdcConfigure__getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      __nesc_result = AdcP__Config__default__getConfiguration(arg_0x2ba0821a9b18);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 191 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__SingleChannel__default__configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config)
#line 192
{
#line 192
  return FAIL;
}

# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcP__SingleChannel__configureSingle(uint8_t arg_0x2ba0821e5910, const msp430adc12_channel_config_t * config){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  switch (arg_0x2ba0821e5910) {
#line 84
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 84
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureSingle(/*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID, config);
#line 84
      break;
#line 84
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT:
#line 84
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureSingle(/*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID, config);
#line 84
      break;
#line 84
    default:
#line 84
      __nesc_result = AdcP__SingleChannel__default__configureSingle(arg_0x2ba0821e5910, config);
#line 84
      break;
#line 84
    }
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 177 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__ResourceReadNow__default__granted(uint8_t nowClient)
#line 177
{
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void AdcP__ResourceReadNow__granted(uint8_t arg_0x2ba0821ad020){
#line 92
    AdcP__ResourceReadNow__default__granted(arg_0x2ba0821ad020);
#line 92
}
#line 92
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__get();
}

# 98 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static AdcStreamP__Alarm__size_type AdcStreamP__Alarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 328 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__SingleChannel__default__configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config)
#line 329
{
#line 329
  return FAIL;
}

# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP__SingleChannel__configureSingle(uint8_t arg_0x2ba082465b90, const msp430adc12_channel_config_t * config){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  switch (arg_0x2ba082465b90) {
#line 84
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 84
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureSingle(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID, config);
#line 84
      break;
#line 84
    default:
#line 84
      __nesc_result = AdcStreamP__SingleChannel__default__configureSingle(arg_0x2ba082465b90, config);
#line 84
      break;
#line 84
    }
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 314 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline const msp430adc12_channel_config_t *AdcStreamP__AdcConfigure__default__getConfiguration(uint8_t c)
{
  return &AdcStreamP__defaultConfig;
}

# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
inline static AdcStreamP__AdcConfigure__adc_config_t AdcStreamP__AdcConfigure__getConfiguration(uint8_t arg_0x2ba082462318){
#line 58
  struct __nesc_unnamed4289 const *__nesc_result;
#line 58

#line 58
  switch (arg_0x2ba082462318) {
#line 58
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 58
      __nesc_result = Msp430AxisXP__AdcConfigure__getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      __nesc_result = AdcStreamP__AdcConfigure__default__getConfiguration(arg_0x2ba082462318);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP__readStreamDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcStreamP__readStreamDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 136 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type dt)
{
  /* atomic removed: atomic calls only */
  {
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 = t0;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt = dt;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__set_alarm();
  }
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void AdcStreamP__Alarm__startAt(AdcStreamP__Alarm__size_type t0, AdcStreamP__Alarm__size_type dt){
#line 92
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 168 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__nextAlarm(void )
#line 168
{
  AdcStreamP__Alarm__startAt(AdcStreamP__now, AdcStreamP__period);
  AdcStreamP__now += AdcStreamP__period;
}

# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )372U = x;
}

# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void Msp430Adc12ImplP__CompareA1__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(time);
#line 30
}
#line 30
# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )370U = x;
}

# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void Msp430Adc12ImplP__CompareA0__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(time);
#line 30
}
#line 30
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4364 {
#line 46
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 89
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)
{
  * (volatile uint16_t * )354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(x);
}

# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void Msp430Adc12ImplP__ControlA0__setControl(msp430_compare_control_t control){
#line 35
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(control);
#line 35
}
#line 35
# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(uint16_t inputDivider)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0040 | 0x0080)) | ((inputDivider << 6) & (0x0040 | 0x0080));
}

# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__setInputDivider(uint16_t inputDivider){
#line 45
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(inputDivider);
#line 45
}
#line 45
# 105 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(uint16_t clockSource)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(256U | 512U)) | ((clockSource << 8) & (256U | 512U));
}

# 44 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__setClockSource(uint16_t clockSource){
#line 44
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(clockSource);
#line 44
}
#line 44
# 100 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents(void )
{
  * (volatile uint16_t * )352U &= ~2U;
}

# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__disableEvents(void ){
#line 43
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents();
#line 43
}
#line 43
# 90 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear(void )
{
  * (volatile uint16_t * )352U |= 4U;
}

# 41 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__clear(void ){
#line 41
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear();
#line 41
}
#line 41
# 103 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON)
{

  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP__TimerA__setMode(MSP430TIMER_STOP_MODE);
  Msp430Adc12ImplP__TimerA__clear();
  Msp430Adc12ImplP__TimerA__disableEvents();
  Msp430Adc12ImplP__TimerA__setClockSource(csSAMPCON);
  Msp430Adc12ImplP__TimerA__setInputDivider(cdSAMPCON);
  Msp430Adc12ImplP__ControlA0__setControl(ccResetSHI);
  Msp430Adc12ImplP__CompareA0__setEvent(interval - 1);
  Msp430Adc12ImplP__CompareA1__setEvent((interval - 1) / 2);
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P__HplAdc12__setIEFlags(uint16_t mask)
#line 92
{
#line 92
  HplAdc12P__ADC12IE = mask;
}

# 95 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__setIEFlags(uint16_t mask){
#line 95
  HplAdc12P__HplAdc12__setIEFlags(mask);
#line 95
}
#line 95
# 61 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  uint8_t HplAdc12P__adc12memctl2int(adc12memctl_t x)
#line 61
{
#line 61
  union __nesc_unnamed4365 {
#line 61
    adc12memctl_t f;
#line 61
    uint8_t t;
  } 
#line 61
  c = { .f = x };

#line 61
  return c.t;
}

#line 80
static inline void HplAdc12P__HplAdc12__setMCtl(uint8_t i, adc12memctl_t memCtl)
#line 80
{
  ((char *)0x0080)[i] = HplAdc12P__adc12memctl2int(memCtl);
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__setMCtl(uint8_t idx, adc12memctl_t memControl){
#line 75
  HplAdc12P__HplAdc12__setMCtl(idx, memControl);
#line 75
}
#line 75
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  uint16_t HplAdc12P__adc12ctl1cast2int(adc12ctl1_t x)
#line 60
{
#line 60
  union __nesc_unnamed4366 {
#line 60
    adc12ctl1_t f;
#line 60
    uint16_t t;
  } 
#line 60
  c = { .f = x };

#line 60
  return c.t;
}






static inline void HplAdc12P__HplAdc12__setCtl1(adc12ctl1_t control1)
#line 68
{
  HplAdc12P__ADC12CTL1 = HplAdc12P__adc12ctl1cast2int(control1);
}

# 57 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__setCtl1(adc12ctl1_t control1){
#line 57
  HplAdc12P__HplAdc12__setCtl1(control1);
#line 57
}
#line 57
#line 51
inline static void Msp430Adc12ImplP__HplAdc12__setCtl0(adc12ctl0_t control0){
#line 51
  HplAdc12P__HplAdc12__setCtl0(control0);
#line 51
}
#line 51
#line 63
inline static adc12ctl0_t Msp430Adc12ImplP__HplAdc12__getCtl0(void ){
#line 63
  struct __nesc_unnamed4254 __nesc_result;
#line 63

#line 63
  __nesc_result = HplAdc12P__HplAdc12__getCtl0();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 88 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t Msp430Adc12ImplP__ADCArbiterInfo__userId(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 271 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP__SingleChannel__configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies)
{
  error_t result = ERESERVE;





  if ((((!config || !buf) || !length) || jiffies == 1) || jiffies == 2) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 284
    {
      if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 286
          EBUSY;

          {
#line 286
            __nesc_atomic_end(__nesc_atomic); 
#line 286
            return __nesc_temp;
          }
        }
#line 287
      if (Msp430Adc12ImplP__ADCArbiterInfo__userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = length > 16 ? 3 : 1, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = jiffies == 0 ? 0 : 1, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 0 };

          uint16_t i;
#line 303
          uint16_t mask = 1;
          adc12ctl0_t ctl0 = Msp430Adc12ImplP__HplAdc12__getCtl0();

#line 305
          ctl0.msc = jiffies == 0 ? 1 : 0;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP__state = Msp430Adc12ImplP__MULTIPLE_DATA;
          Msp430Adc12ImplP__resultBufferStart = (void *)0;
          Msp430Adc12ImplP__resultBufferLength = length;
          Msp430Adc12ImplP__resultBufferStart = buf;
          Msp430Adc12ImplP__resultBufferIndex = 0;
          Msp430Adc12ImplP__HplAdc12__setCtl0(ctl0);
          Msp430Adc12ImplP__HplAdc12__setCtl1(ctl1);
          for (i = 0; i < length - 1 && i < 15; i++) 
            Msp430Adc12ImplP__HplAdc12__setMCtl(i, memctl);
          memctl.eos = 1;
          Msp430Adc12ImplP__HplAdc12__setMCtl(i, memctl);
          Msp430Adc12ImplP__HplAdc12__setIEFlags(mask << i);

          if (jiffies) {
              Msp430Adc12ImplP__state |= Msp430Adc12ImplP__USE_TIMERA;
              Msp430Adc12ImplP__prepareTimerA(jiffies, config->sampcon_ssel, config->sampcon_id);
            }
          result = SUCCESS;
        }
    }
#line 328
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 318 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__SingleChannel__default__configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies)
{
  return FAIL;
}

# 138 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP__SingleChannel__configureMultiple(uint8_t arg_0x2ba082465b90, const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies){
#line 138
  unsigned char __nesc_result;
#line 138

#line 138
  switch (arg_0x2ba082465b90) {
#line 138
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 138
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureMultiple(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID, config, buffer, numSamples, jiffies);
#line 138
      break;
#line 138
    default:
#line 138
      __nesc_result = AdcStreamP__SingleChannel__default__configureMultiple(arg_0x2ba082465b90, config, buffer, numSamples, jiffies);
#line 138
      break;
#line 138
    }
#line 138

#line 138
  return __nesc_result;
#line 138
}
#line 138
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP__readStreamFail__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcStreamP__readStreamFail);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 180 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__SingleChannel__default__getData(uint8_t client)
{
  return EINVAL;
}

# 189 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcP__SingleChannel__getData(uint8_t arg_0x2ba0821e5910){
#line 189
  unsigned char __nesc_result;
#line 189

#line 189
  switch (arg_0x2ba0821e5910) {
#line 189
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 189
      __nesc_result = Msp430Adc12ImplP__SingleChannel__getData(/*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 189
      break;
#line 189
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT:
#line 189
      __nesc_result = Msp430Adc12ImplP__SingleChannel__getData(/*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID);
#line 189
      break;
#line 189
    default:
#line 189
      __nesc_result = AdcP__SingleChannel__default__getData(arg_0x2ba0821e5910);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return __nesc_result;
#line 189
}
#line 189
# 165 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id)
#line 165
{
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__granted(uint8_t arg_0x2ba08237e9b0){
#line 92
  switch (arg_0x2ba08237e9b0) {
#line 92
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 92
      Msp430RefVoltArbiterImplP__AdcResource__granted(/*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 92
      break;
#line 92
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 92
      Msp430RefVoltArbiterImplP__AdcResource__granted(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 92
      break;
#line 92
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 92
      Msp430RefVoltArbiterImplP__AdcResource__granted(/*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(arg_0x2ba08237e9b0);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 171 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 171
{
}

# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x2ba08237a220){
#line 49
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(arg_0x2ba08237a220);
#line 49
}
#line 49
# 155 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void )
#line 155
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId;
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_BUSY;
    }
#line 159
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId);
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__granted(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId);
}

# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
inline static /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__adc_config_t /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__getConfiguration(void ){
#line 58
  struct __nesc_unnamed4289 const *__nesc_result;
#line 58

#line 58
  __nesc_result = Msp430AxisXP__AdcConfigure__getConfiguration();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration(void )
{
  return /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__getConfiguration();
}

# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
inline static /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__adc_config_t /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__getConfiguration(void ){
#line 58
  struct __nesc_unnamed4289 const *__nesc_result;
#line 58

#line 58
  __nesc_result = Msp430AxisXP__AdcConfigure__getConfiguration();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration(void )
{
  return /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__getConfiguration();
}

# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
inline static /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__adc_config_t /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__getConfiguration(void ){
#line 58
  struct __nesc_unnamed4289 const *__nesc_result;
#line 58

#line 58
  __nesc_result = Msp430AxisXP__AdcConfigure__getConfiguration();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__getConfiguration(void )
{
  return /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__getConfiguration();
}

# 172 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP__Config__default__getConfiguration(uint8_t client)
{
  return &Msp430RefVoltArbiterImplP__defaultConfig;
}

# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/AdcConfigure.nc"
inline static Msp430RefVoltArbiterImplP__Config__adc_config_t Msp430RefVoltArbiterImplP__Config__getConfiguration(uint8_t arg_0x2ba08242d248){
#line 58
  struct __nesc_unnamed4289 const *__nesc_result;
#line 58

#line 58
  switch (arg_0x2ba08242d248) {
#line 58
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 58
      __nesc_result = /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration();
#line 58
      break;
#line 58
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 58
      __nesc_result = /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration();
#line 58
      break;
#line 58
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 58
      __nesc_result = /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      __nesc_result = Msp430RefVoltArbiterImplP__Config__default__getConfiguration(arg_0x2ba08242d248);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 167 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 167
{
}

# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(uint8_t arg_0x2ba08237dc80){
#line 43
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(arg_0x2ba08237dc80);
#line 43
}
#line 43
# 87 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/RoundRobinResourceQueueC.nc"
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(resource_client_id_t id)
#line 87
{
  /* atomic removed: atomic calls only */
#line 88
  {
    if (!/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(id)) {
        /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[id / 8] |= 1 << id % 8;
        {
          unsigned char __nesc_temp = 
#line 91
          SUCCESS;

#line 91
          return __nesc_temp;
        }
      }
#line 93
    {
      unsigned char __nesc_temp = 
#line 93
      EBUSY;

#line 93
      return __nesc_temp;
    }
  }
}

# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__start(void )
#line 78
{
  return Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE);
}

# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_1_5V__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(2U, dt);
#line 62
}
#line 62





inline static void Msp430RefVoltGeneratorP__SwitchOffTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(3U);
#line 67
}
#line 67
# 82 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__start(void )
#line 82
{
  return Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE);
}

# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_2_5V__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_2_5V__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 172 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__Read__default__readDone(uint8_t client, error_t result, uint16_t val)
#line 172
{
}

# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Read.nc"
inline static void AdcP__Read__readDone(uint8_t arg_0x2ba0821b24e8, error_t result, AdcP__Read__val_t val){
#line 63
    AdcP__Read__default__readDone(arg_0x2ba0821b24e8, result, val);
#line 63
}
#line 63
# 170 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__ResourceRead__default__release(uint8_t client)
#line 170
{
#line 170
  return FAIL;
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t AdcP__ResourceRead__release(uint8_t arg_0x2ba0821ab318){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x2ba0821ab318) {
#line 110
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 110
      __nesc_result = Msp430RefVoltArbiterImplP__ClientResource__release(/*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = AdcP__ResourceRead__default__release(arg_0x2ba0821ab318);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 136 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__readDone__runTask(void )
{
  AdcP__ResourceRead__release(AdcP__owner);
  AdcP__Read__readDone(AdcP__owner, SUCCESS, AdcP__value);
}

# 284 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__ClientResource__default__granted(uint8_t id)
#line 284
{
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void Stm25pSectorP__ClientResource__granted(uint8_t arg_0x2ba081c94108){
#line 92
  switch (arg_0x2ba081c94108) {
#line 92
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID:
#line 92
      Stm25pLogP__ClientResource__granted(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID);
#line 92
      break;
#line 92
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID:
#line 92
      Stm25pBlockP__ClientResource__granted(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID);
#line 92
      break;
#line 92
    default:
#line 92
      Stm25pSectorP__ClientResource__default__granted(arg_0x2ba081c94108);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 117 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void Stm25pSectorP__SplitControl__stopDone(error_t error){
#line 117
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__stopDone(error);
#line 117
}
#line 117
# 114 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id)
#line 114
{
#line 114
  return FAIL;
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(uint8_t arg_0x2ba081e91be8){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x2ba081e91be8) {
#line 110
    case /*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 110
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(/*HplStm25pSpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(arg_0x2ba081e91be8);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 81 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id)
#line 81
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(id);
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pSpiP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(/*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 116 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static inline error_t Stm25pSpiP__ClientResource__release(void )
#line 116
{
  return Stm25pSpiP__SpiResource__release();
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pSectorP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = Stm25pSpiP__ClientResource__release();
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 128 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static inline error_t Stm25pSpiP__Spi__powerDown(void )
#line 128
{
  Stm25pSpiP__sendCmd(Stm25pSpiP__S_DEEP_SLEEP, 1);
  return SUCCESS;
}

# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
inline static error_t Stm25pSectorP__Spi__powerDown(void ){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = Stm25pSpiP__Spi__powerDown();
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__release(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 100 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__startDone(error_t error)
#line 100
{
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__release();
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void Stm25pSectorP__SplitControl__startDone(error_t error){
#line 92
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__startDone(error);
#line 92
}
#line 92
# 133 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static inline error_t Stm25pSpiP__Spi__powerUp(void )
#line 133
{
  Stm25pSpiP__sendCmd(Stm25pSpiP__S_POWER_ON, 5);
  return SUCCESS;
}

# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
inline static error_t Stm25pSectorP__Spi__powerUp(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = Stm25pSpiP__Spi__powerUp();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 130 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__SpiResource__granted(void )
#line 130
{
  error_t error;
  Stm25pSectorP__stm25p_power_state_t power_state = Stm25pSectorP__m_power_state;

#line 133
  Stm25pSectorP__m_power_state = Stm25pSectorP__S_NONE;
  if (power_state == Stm25pSectorP__S_START) {
      error = Stm25pSectorP__Spi__powerUp();
      Stm25pSectorP__SpiResource__release();
      Stm25pSectorP__SplitControl__startDone(error);
      return;
    }
  else {
#line 140
    if (power_state == Stm25pSectorP__S_STOP) {
        error = Stm25pSectorP__Spi__powerDown();
        Stm25pSectorP__SpiResource__release();
        Stm25pSectorP__SplitControl__stopDone(error);
        return;
      }
    }
#line 146
  Stm25pSectorP__ClientResource__granted(Stm25pSectorP__m_client);
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void Stm25pSpiP__ClientResource__granted(void ){
#line 92
  Stm25pSectorP__SpiResource__granted();
#line 92
}
#line 92
# 238 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static inline void Stm25pSpiP__SpiResource__granted(void )
#line 238
{

  if (!Stm25pSpiP__m_is_writing) {
    Stm25pSpiP__ClientResource__granted();
    }
  else {
#line 242
    if (Stm25pSpiP__sendCmd(0x5, 2) & 0x1) {
      Stm25pSpiP__releaseAndRequest();
      }
    else {
#line 245
      Stm25pSpiP__signalDone(SUCCESS);
      }
    }
}

# 119 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id)
#line 119
{
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(uint8_t arg_0x2ba081e956f8){
#line 92
  switch (arg_0x2ba081e956f8) {
#line 92
    case /*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 92
      Stm25pSpiP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(arg_0x2ba081e956f8);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 95 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id)
#line 95
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(id);
}

# 199 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id)
#line 199
{
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(uint8_t arg_0x2ba081d44d40){
#line 92
  switch (arg_0x2ba081d44d40) {
#line 92
    case /*HplStm25pSpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 92
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(/*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(arg_0x2ba081d44d40);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 115 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 115
{
  return &msp430_spi_default_config;
}

# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x2ba081e90e18){
#line 39
  union __nesc_unnamed4277 *__nesc_result;
#line 39

#line 39
    __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0x2ba081e90e18);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 357 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableIntr(void )
#line 357
{
  HplMsp430Usart0P__IE1 &= ~((1 << 7) | (1 << 6));
}

#line 345
static inline void HplMsp430Usart0P__Usart__clrIntr(void )
#line 345
{
  HplMsp430Usart0P__IFG1 &= ~((1 << 7) | (1 << 6));
}

#line 151
static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset)
#line 151
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 78
}
#line 78
# 238 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableSpi(void )
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    HplMsp430Usart0P__SIMO__selectModuleFunc();
    HplMsp430Usart0P__SOMI__selectModuleFunc();
    HplMsp430Usart0P__UCLK__selectModuleFunc();
  }
  HplMsp430Usart0P__ME1 |= 1 << 6;
}

#line 143
static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control)
#line 143
{
  U0MCTL = control;
}

#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control)
#line 132
{
  /* atomic removed: atomic calls only */
#line 133
  {
    U0BR0 = control & 0x00FF;
    U0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 256
static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config)
#line 256
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P__U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P__Usart__setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P__Usart__setUmctl(0x00);
}

# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 85
}
#line 85
# 207 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableUart(void )
#line 207
{
  /* atomic removed: atomic calls only */
#line 208
  {
    HplMsp430Usart0P__ME1 &= ~((1 << 7) | (1 << 6));
    HplMsp430Usart0P__UTXD__selectIOFunc();
    HplMsp430Usart0P__URXD__selectIOFunc();
  }
}

# 97 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 60
  {
    HplMsp430I2C0P__U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P__HplUsart__resetUsart(TRUE);
  }
}

# 7 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 265 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config)
#line 265
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 267
    {
      HplMsp430Usart0P__Usart__resetUsart(TRUE);
      HplMsp430Usart0P__HplI2C__clearModeI2C();
      HplMsp430Usart0P__Usart__disableUart();
      HplMsp430Usart0P__configSpi(config);
      HplMsp430Usart0P__Usart__enableSpi();
      HplMsp430Usart0P__Usart__resetUsart(FALSE);
      HplMsp430Usart0P__Usart__clrIntr();
      HplMsp430Usart0P__Usart__disableIntr();
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 168 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 85
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 213 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(uint8_t arg_0x2ba081d3f340){
#line 49
  switch (arg_0x2ba081d3f340) {
#line 49
    case /*HplStm25pSpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(/*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(arg_0x2ba081d3f340);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 187 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
}

# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__clr();
#line 39
}
#line 39
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__HplGeneralIO__clr();
}

# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void Stm25pSpiP__CSN__clr(void ){
#line 30
  /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__clr();
#line 30
}
#line 30
# 231 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void ){
#line 231
  unsigned char __nesc_result;
#line 231

#line 231
  __nesc_result = HplMsp430Usart0P__Usart__rx();
#line 231

#line 231
  return __nesc_result;
#line 231
}
#line 231
# 341 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__clrRxIntr(void )
#line 341
{
  HplMsp430Usart0P__IFG1 &= ~(1 << 6);
}

# 197 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void ){
#line 197
  HplMsp430Usart0P__Usart__clrRxIntr();
#line 197
}
#line 197
# 330 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 330
{
  if (HplMsp430Usart0P__IFG1 & (1 << 6)) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void ){
#line 192
  unsigned char __nesc_result;
#line 192

#line 192
  __nesc_result = HplMsp430Usart0P__Usart__isRxIntrPending();
#line 192

#line 192
  return __nesc_result;
#line 192
}
#line 192
# 382 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 382
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 383
    HplMsp430Usart0P__U0TXBUF = data;
#line 383
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 99 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx)
#line 99
{
  uint8_t byte;


  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(tx);
  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr();
  byte = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();

  return byte;
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiByte.nc"
inline static uint8_t Stm25pSpiP__SpiByte__write(uint8_t tx){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(tx);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead;

#line 62
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 65
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 97 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
#line 158
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 89
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi();
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(FALSE);
}

# 215 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0x2ba081d3f340){
#line 55
  switch (arg_0x2ba081d3f340) {
#line 55
    case /*HplStm25pSpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 55
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(/*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(arg_0x2ba081d3f340);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 85
}
#line 85
# 205 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void )
#line 205
{
}

# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 201 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id)
#line 201
{
}

# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(uint8_t arg_0x2ba081d41020){
#line 43
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(arg_0x2ba081d41020);
#line 43
}
#line 43
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = id;
          }
        else {
#line 78
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail] = id;
          }
#line 79
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 130 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void )
#line 130
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id) {
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 135
                SUCCESS;

                {
#line 135
                  __nesc_atomic_end(__nesc_atomic); 
#line 135
                  return __nesc_temp;
                }
              }
            }
          else {
#line 137
            if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING) {
                /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
                /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 140
                  SUCCESS;

                  {
#line 140
                    __nesc_atomic_end(__nesc_atomic); 
#line 140
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 144
  return FAIL;
}

#line 207
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void )
#line 207
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
}

# 73 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 230 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline error_t Stm25pBlockP__Sector__default__read(uint8_t id, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len)
#line 230
{
#line 230
  return FAIL;
}

# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static error_t Stm25pBlockP__Sector__read(uint8_t arg_0x2ba08211da70, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  switch (arg_0x2ba08211da70) {
#line 68
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 68
      __nesc_result = Stm25pSectorP__Sector__read(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID, addr, buf, len);
#line 68
      break;
#line 68
    default:
#line 68
      __nesc_result = Stm25pBlockP__Sector__default__read(arg_0x2ba08211da70, addr, buf, len);
#line 68
      break;
#line 68
    }
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
inline static error_t Stm25pSectorP__Spi__read(stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = Stm25pSpiP__Spi__read(addr, buf, len);
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
inline static error_t Stm25pSpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(/*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 361 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableRxIntr(void )
#line 361
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 362
    {
      HplMsp430Usart0P__IFG1 &= ~(1 << 6);
      HplMsp430Usart0P__IE1 |= 1 << 6;
    }
#line 365
    __nesc_atomic_end(__nesc_atomic); }
}

# 180 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void ){
#line 180
  HplMsp430Usart0P__Usart__enableRxIntr();
#line 180
}
#line 180
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBinderP.nc"
static inline volume_id_t /*FlashSamplerAppC.LogStorageC.BinderP*/Stm25pBinderP__0__Volume__getVolumeId(void )
#line 45
{
  return 100;
}

#line 45
static inline volume_id_t /*FlashSamplerAppC.BlockStorageC.BinderP*/Stm25pBinderP__1__Volume__getVolumeId(void )
#line 45
{
  return 100;
}

# 289 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline volume_id_t Stm25pSectorP__Volume__default__getVolumeId(uint8_t id)
#line 289
{
#line 289
  return 0xff;
}

# 48 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pVolume.nc"
inline static volume_id_t Stm25pSectorP__Volume__getVolumeId(uint8_t arg_0x2ba081c8ed10){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  switch (arg_0x2ba081c8ed10) {
#line 48
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID:
#line 48
      __nesc_result = /*FlashSamplerAppC.LogStorageC.BinderP*/Stm25pBinderP__0__Volume__getVolumeId();
#line 48
      break;
#line 48
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID:
#line 48
      __nesc_result = /*FlashSamplerAppC.BlockStorageC.BinderP*/Stm25pBinderP__1__Volume__getVolumeId();
#line 48
      break;
#line 48
    default:
#line 48
      __nesc_result = Stm25pSectorP__Volume__default__getVolumeId(arg_0x2ba081c8ed10);
#line 48
      break;
#line 48
    }
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 126 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline uint8_t Stm25pSectorP__getVolumeId(uint8_t client)
#line 126
{
  return Stm25pSectorP__Volume__getVolumeId(client);
}

#line 153
static inline stm25p_addr_t Stm25pSectorP__physicalAddr(uint8_t id, stm25p_addr_t addr)
#line 153
{
  return addr + ((stm25p_addr_t )STM25P_VMAP[Stm25pSectorP__getVolumeId(id)].base
   << STM25P_SECTOR_SIZE_LOG2);
}

# 124 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static inline stm25p_len_t Stm25pSpiP__calcReadLen(void )
#line 124
{
  return Stm25pSpiP__m_cur_len < Stm25pSpiP__CRC_BUF_SIZE ? Stm25pSpiP__m_cur_len : Stm25pSpiP__CRC_BUF_SIZE;
}

#line 147
static inline error_t Stm25pSpiP__Spi__computeCrc(uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len)
#line 148
{
  Stm25pSpiP__m_computing_crc = TRUE;
  Stm25pSpiP__m_crc = crc;
  Stm25pSpiP__m_addr = Stm25pSpiP__m_cur_addr = addr;
  Stm25pSpiP__m_len = Stm25pSpiP__m_cur_len = len;
  return Stm25pSpiP__Spi__read(addr, Stm25pSpiP__m_crc_buf, Stm25pSpiP__calcReadLen());
}

# 90 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
inline static error_t Stm25pSectorP__Spi__computeCrc(uint16_t crc, stm25p_addr_t addr, stm25p_len_t len){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = Stm25pSpiP__Spi__computeCrc(crc, addr, len);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 234 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline error_t Stm25pSectorP__Sector__computeCrc(uint8_t id, uint16_t crc, 
stm25p_addr_t addr, 
stm25p_len_t len)
#line 236
{

  Stm25pSectorP__m_state = Stm25pSectorP__S_CRC;
  Stm25pSectorP__m_addr = addr;
  Stm25pSectorP__m_len = len;

  return Stm25pSectorP__Spi__computeCrc(crc, Stm25pSectorP__physicalAddr(id, addr), Stm25pSectorP__m_len);
}

# 233 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline error_t Stm25pBlockP__Sector__default__computeCrc(uint8_t id, uint16_t crc, storage_addr_t addr, storage_len_t len)
#line 233
{
#line 233
  return FAIL;
}

# 133 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static error_t Stm25pBlockP__Sector__computeCrc(uint8_t arg_0x2ba08211da70, uint16_t crc, stm25p_addr_t addr, stm25p_len_t len){
#line 133
  unsigned char __nesc_result;
#line 133

#line 133
  switch (arg_0x2ba08211da70) {
#line 133
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 133
      __nesc_result = Stm25pSectorP__Sector__computeCrc(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID, crc, addr, len);
#line 133
      break;
#line 133
    default:
#line 133
      __nesc_result = Stm25pBlockP__Sector__default__computeCrc(arg_0x2ba08211da70, crc, addr, len);
#line 133
      break;
#line 133
    }
#line 133

#line 133
  return __nesc_result;
#line 133
}
#line 133
# 231 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline error_t Stm25pBlockP__Sector__default__write(uint8_t id, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len)
#line 231
{
#line 231
  return FAIL;
}

# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static error_t Stm25pBlockP__Sector__write(uint8_t arg_0x2ba08211da70, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len){
#line 91
  unsigned char __nesc_result;
#line 91

#line 91
  switch (arg_0x2ba08211da70) {
#line 91
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 91
      __nesc_result = Stm25pSectorP__Sector__write(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID, addr, buf, len);
#line 91
      break;
#line 91
    default:
#line 91
      __nesc_result = Stm25pBlockP__Sector__default__write(arg_0x2ba08211da70, addr, buf, len);
#line 91
      break;
#line 91
    }
#line 91

#line 91
  return __nesc_result;
#line 91
}
#line 91
# 232 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline error_t Stm25pBlockP__Sector__default__erase(uint8_t id, uint8_t sector, uint8_t num_sectors)
#line 232
{
#line 232
  return FAIL;
}

# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static error_t Stm25pBlockP__Sector__erase(uint8_t arg_0x2ba08211da70, uint8_t sector, uint8_t num_sectors){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  switch (arg_0x2ba08211da70) {
#line 112
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 112
      __nesc_result = Stm25pSectorP__Sector__erase(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID, sector, num_sectors);
#line 112
      break;
#line 112
    default:
#line 112
      __nesc_result = Stm25pBlockP__Sector__default__erase(arg_0x2ba08211da70, sector, num_sectors);
#line 112
      break;
#line 112
    }
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 229 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline uint8_t Stm25pBlockP__Sector__default__getNumSectors(uint8_t id)
#line 229
{
#line 229
  return 0;
}

# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static uint8_t Stm25pBlockP__Sector__getNumSectors(uint8_t arg_0x2ba08211da70){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  switch (arg_0x2ba08211da70) {
#line 56
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 56
      __nesc_result = Stm25pSectorP__Sector__getNumSectors(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID);
#line 56
      break;
#line 56
    default:
#line 56
      __nesc_result = Stm25pBlockP__Sector__default__getNumSectors(arg_0x2ba08211da70);
#line 56
      break;
#line 56
    }
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 235 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline error_t Stm25pBlockP__ClientResource__default__release(uint8_t id)
#line 235
{
#line 235
  return FAIL;
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pBlockP__ClientResource__release(uint8_t arg_0x2ba08211b530){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x2ba08211b530) {
#line 110
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 110
      __nesc_result = Stm25pSectorP__ClientResource__release(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = Stm25pBlockP__ClientResource__default__release(arg_0x2ba08211b530);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 104 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__granted(void )
#line 104
{
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask__postTask();
}

# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__granted();
#line 46
}
#line 46
# 215 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x2ba081d3f340){
#line 55
    /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x2ba081d3f340);
#line 55
}
#line 55
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead != /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
        uint8_t id = /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead;

#line 62
        /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead];
        if (/*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
          }
#line 65
        /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] = /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static bool /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 108 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 108
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state == /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
          if (/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 124
            SUCCESS;

            {
#line 124
              __nesc_atomic_end(__nesc_atomic); 
#line 124
              return __nesc_temp;
            }
          }
        }
    }
#line 128
    __nesc_atomic_end(__nesc_atomic); }
#line 127
  return FAIL;
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pSectorP__Stm25pResource__release(uint8_t arg_0x2ba081c8ca68){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(arg_0x2ba081c8ca68);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 528 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline error_t Stm25pLogP__ClientResource__default__request(uint8_t id)
#line 528
{
#line 528
  return FAIL;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pLogP__ClientResource__request(uint8_t arg_0x2ba081bdb220){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0x2ba081bdb220) {
#line 78
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 78
      __nesc_result = Stm25pSectorP__ClientResource__request(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = Stm25pLogP__ClientResource__default__request(arg_0x2ba081bdb220);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 190 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline error_t Stm25pLogP__newRequest(uint8_t client)
#line 190
{

  if (Stm25pLogP__m_log_state[client].req != Stm25pLogP__S_IDLE) {
    return FAIL;
    }
  Stm25pLogP__ClientResource__request(client);
  Stm25pLogP__m_log_state[client] = Stm25pLogP__m_req;

  return SUCCESS;
}

#line 523
static inline uint8_t Stm25pLogP__Sector__default__getNumSectors(uint8_t id)
#line 523
{
#line 523
  return 0;
}

# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static uint8_t Stm25pLogP__Sector__getNumSectors(uint8_t arg_0x2ba081bb6468){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  switch (arg_0x2ba081bb6468) {
#line 56
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 56
      __nesc_result = Stm25pSectorP__Sector__getNumSectors(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID);
#line 56
      break;
#line 56
    default:
#line 56
      __nesc_result = Stm25pLogP__Sector__default__getNumSectors(arg_0x2ba081bb6468);
#line 56
      break;
#line 56
    }
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogConfigP.nc"
static inline bool /*FlashSamplerAppC.LogStorageC.ConfigP*/Stm25pLogConfigP__0__Circular__get(void )
#line 45
{
  return 1;
}

# 530 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline bool Stm25pLogP__Circular__default__get(uint8_t id)
#line 530
{
#line 530
  return FALSE;
}

# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Get.nc"
inline static Stm25pLogP__Circular__val_t Stm25pLogP__Circular__get(uint8_t arg_0x2ba081bd3858){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  switch (arg_0x2ba081bd3858) {
#line 55
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 55
      __nesc_result = /*FlashSamplerAppC.LogStorageC.ConfigP*/Stm25pLogConfigP__0__Circular__get();
#line 55
      break;
#line 55
    default:
#line 55
      __nesc_result = Stm25pLogP__Circular__default__get(arg_0x2ba081bd3858);
#line 55
      break;
#line 55
    }
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 157 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline error_t Stm25pLogP__Write__append(uint8_t id, void *buf, storage_len_t len)
#line 157
{

  uint16_t bytes_left = (uint16_t )Stm25pLogP__m_log_info[id].write_addr % Stm25pLogP__BLOCK_SIZE;

#line 160
  bytes_left = Stm25pLogP__BLOCK_SIZE - bytes_left;


  if (len > Stm25pLogP__MAX_RECORD_SIZE) {
    return EINVAL;
    }

  if (sizeof Stm25pLogP__m_header + len > bytes_left) {
    Stm25pLogP__m_log_info[id].write_addr += bytes_left;
    }

  if (!Stm25pLogP__Circular__get(id) && 
  (uint8_t )(Stm25pLogP__m_log_info[id].write_addr >> STM25P_SECTOR_SIZE_LOG2) >= 
  Stm25pLogP__Sector__getNumSectors(id)) {
    return ESIZE;
    }
  Stm25pLogP__m_records_lost = FALSE;
  Stm25pLogP__m_req.req = Stm25pLogP__S_APPEND;
  Stm25pLogP__m_req.buf = buf;
  Stm25pLogP__m_req.len = len;

  return Stm25pLogP__newRequest(id);
}

# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
inline static error_t SummarizerC__LogWrite__append(void * buf, storage_len_t len){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = Stm25pLogP__Write__append(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID, buf, len);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 35 "SummarizerC.nc"
static inline void SummarizerC__BlockRead__readDone(storage_addr_t addr, void *buf, storage_len_t len, 
error_t error)
#line 36
{

  uint32_t sum = 0;
  uint16_t i;

  for (i = 0; i < DFACTOR; i++) sum += SummarizerC__samples[i];
  SummarizerC__summary[SummarizerC__index++] = sum / DFACTOR;


  if (SummarizerC__index < SUMMARY_SAMPLES) {
    SummarizerC__nextSummarySample();
    }
  else {
#line 48
    SummarizerC__LogWrite__append(SummarizerC__summary, sizeof SummarizerC__summary);
    }
}

# 222 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline void Stm25pBlockP__Read__default__readDone(uint8_t id, storage_addr_t addr, void *buf, storage_len_t len, error_t error)
#line 222
{
}

# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
inline static void Stm25pBlockP__Read__readDone(uint8_t arg_0x2ba082125970, storage_addr_t addr, void * buf, storage_len_t len, error_t error){
#line 67
  switch (arg_0x2ba082125970) {
#line 67
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 67
      SummarizerC__BlockRead__readDone(addr, buf, len, error);
#line 67
      break;
#line 67
    default:
#line 67
      Stm25pBlockP__Read__default__readDone(arg_0x2ba082125970, addr, buf, len, error);
#line 67
      break;
#line 67
    }
#line 67
}
#line 67
# 57 "SummarizerC.nc"
static inline void SummarizerC__BlockRead__computeCrcDone(storage_addr_t addr, storage_len_t len, 
uint16_t crc, error_t error)
#line 58
{
}

# 223 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline void Stm25pBlockP__Read__default__computeCrcDone(uint8_t id, storage_addr_t addr, storage_len_t len, uint16_t crc, error_t error)
#line 223
{
}

# 95 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockRead.nc"
inline static void Stm25pBlockP__Read__computeCrcDone(uint8_t arg_0x2ba082125970, storage_addr_t addr, storage_len_t len, uint16_t crc, error_t error){
#line 95
  switch (arg_0x2ba082125970) {
#line 95
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 95
      SummarizerC__BlockRead__computeCrcDone(addr, len, crc, error);
#line 95
      break;
#line 95
    default:
#line 95
      Stm25pBlockP__Read__default__computeCrcDone(arg_0x2ba082125970, addr, len, crc, error);
#line 95
      break;
#line 95
    }
#line 95
}
#line 95
# 114 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline error_t Stm25pBlockP__Write__sync(uint8_t id)
#line 114
{
  Stm25pBlockP__m_req.req = Stm25pBlockP__S_SYNC;
  return Stm25pBlockP__newRequest(id);
}

# 103 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
inline static error_t AccelSamplerC__BlockWrite__sync(void ){
#line 103
  unsigned char __nesc_result;
#line 103

#line 103
  __nesc_result = Stm25pBlockP__Write__sync(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID);
#line 103

#line 103
  return __nesc_result;
#line 103
}
#line 103
# 115 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__ReadStream__postBuffer(uint8_t c, uint16_t *buf, uint16_t n)
#line 115
{
  return AdcStreamP__postBuffer(c, buf, n);
}

# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__postBuffer(uint8_t arg_0x2ba0824fb148, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__val_t * buf, uint16_t count){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  __nesc_result = AdcStreamP__ReadStream__postBuffer(arg_0x2ba0824fb148, buf, count);
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 33 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__postBuffer(uint8_t client, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count)
{
  return /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__postBuffer(client, buf, count);
}

# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
inline static error_t AccelSamplerC__Accel__postBuffer(AccelSamplerC__Accel__val_t * buf, uint16_t count){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  __nesc_result = /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__postBuffer(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT, buf, count);
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 45 "AccelSamplerC.nc"
static inline void AccelSamplerC__BlockWrite__writeDone(storage_addr_t addr, void *buf, storage_len_t len, 
error_t error)
#line 46
{



  if (++AccelSamplerC__nbuffers <= TOTAL_SAMPLES / BUFFER_SIZE - 2) {
    AccelSamplerC__Accel__postBuffer(buf, BUFFER_SIZE);
    }
  else {
#line 52
    if (AccelSamplerC__nbuffers == TOTAL_SAMPLES / BUFFER_SIZE) {

      AccelSamplerC__BlockWrite__sync();
      }
    }
}

# 224 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline void Stm25pBlockP__Write__default__writeDone(uint8_t id, storage_addr_t addr, void *buf, storage_len_t len, error_t error)
#line 224
{
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
inline static void Stm25pBlockP__Write__writeDone(uint8_t arg_0x2ba082120b68, storage_addr_t addr, void * buf, storage_len_t len, error_t error){
#line 71
  switch (arg_0x2ba082120b68) {
#line 71
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 71
      AccelSamplerC__BlockWrite__writeDone(addr, buf, len, error);
#line 71
      break;
#line 71
    default:
#line 71
      Stm25pBlockP__Write__default__writeDone(arg_0x2ba082120b68, addr, buf, len, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 98 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Off(void )
#line 98
{
  LedsP__Led2__set();
  ;
#line 100
  ;
}

# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
inline static void AccelSamplerC__Leds__led2Off(void ){
#line 83
  LedsP__Leds__led2Off();
#line 83
}
#line 83
# 57 "AccelSamplerC.nc"
static inline void AccelSamplerC__BlockWrite__syncDone(error_t error)
#line 57
{
  AccelSamplerC__Leds__led2Off();
  AccelSamplerC__Sample__sampled(error);
}

# 226 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline void Stm25pBlockP__Write__default__syncDone(uint8_t id, error_t error)
#line 226
{
}

# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
inline static void Stm25pBlockP__Write__syncDone(uint8_t arg_0x2ba082120b68, error_t error){
#line 112
  switch (arg_0x2ba082120b68) {
#line 112
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 112
      AccelSamplerC__BlockWrite__syncDone(error);
#line 112
      break;
#line 112
    default:
#line 112
      Stm25pBlockP__Write__default__syncDone(arg_0x2ba082120b68, error);
#line 112
      break;
#line 112
    }
#line 112
}
#line 112
# 161 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__request(uint8_t client)
{
  return FAIL;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Msp430RefVoltArbiterImplP__AdcResource__request(uint8_t arg_0x2ba0823db538){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0x2ba0823db538) {
#line 78
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 78
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(/*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 78
      break;
#line 78
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 78
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 78
      break;
#line 78
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 78
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(/*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = Msp430RefVoltArbiterImplP__AdcResource__default__request(arg_0x2ba0823db538);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 53 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP__ClientResource__request(uint8_t client)
{
  return Msp430RefVoltArbiterImplP__AdcResource__request(client);
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__request(uint8_t client)
#line 64
{
  return SUCCESS;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__request(uint8_t arg_0x2ba0824f7020){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0x2ba0824f7020) {
#line 78
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 78
      __nesc_result = Msp430RefVoltArbiterImplP__ClientResource__request(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__request(arg_0x2ba0824f7020);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__read(uint8_t client, uint32_t usPeriod)
{
  error_t ok = /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__request(client);

  if (ok == SUCCESS) {
    /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__period[client] = usPeriod;
    }
  return ok;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
inline static error_t AccelSamplerC__Accel__read(uint32_t usPeriod){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__read(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT, usPeriod);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )49U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr();
#line 39
}
#line 39
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr();
}

# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__clr(void ){
#line 30
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr();
#line 30
}
#line 30
# 93 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2On(void )
#line 93
{
  LedsP__Led2__clr();
  ;
#line 95
  ;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
inline static void AccelSamplerC__Leds__led2On(void ){
#line 78
  LedsP__Leds__led2On();
#line 78
}
#line 78
# 30 "AccelSamplerC.nc"
static inline void AccelSamplerC__BlockWrite__eraseDone(error_t ok)
#line 30
{
  AccelSamplerC__Leds__led2On();

  AccelSamplerC__Accel__postBuffer(AccelSamplerC__buffer1, BUFFER_SIZE);
  AccelSamplerC__Accel__postBuffer(AccelSamplerC__buffer2, BUFFER_SIZE);
  AccelSamplerC__nbuffers = 0;
  AccelSamplerC__Accel__read(SAMPLE_PERIOD);
}

# 225 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline void Stm25pBlockP__Write__default__eraseDone(uint8_t id, error_t error)
#line 225
{
}

# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
inline static void Stm25pBlockP__Write__eraseDone(uint8_t arg_0x2ba082120b68, error_t error){
#line 91
  switch (arg_0x2ba082120b68) {
#line 91
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID:
#line 91
      AccelSamplerC__BlockWrite__eraseDone(error);
#line 91
      break;
#line 91
    default:
#line 91
      Stm25pBlockP__Write__default__eraseDone(arg_0x2ba082120b68, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 529 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline error_t Stm25pLogP__ClientResource__default__release(uint8_t id)
#line 529
{
#line 529
  return FAIL;
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pLogP__ClientResource__release(uint8_t arg_0x2ba081bdb220){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x2ba081bdb220) {
#line 110
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 110
      __nesc_result = Stm25pSectorP__ClientResource__release(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = Stm25pLogP__ClientResource__default__release(arg_0x2ba081bdb220);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 516 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline void Stm25pLogP__Read__default__readDone(uint8_t id, void *data, storage_len_t len, error_t error)
#line 516
{
}

# 75 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogRead.nc"
inline static void Stm25pLogP__Read__readDone(uint8_t arg_0x2ba081bb91a0, void * buf, storage_len_t len, error_t error){
#line 75
    Stm25pLogP__Read__default__readDone(arg_0x2ba081bb91a0, buf, len, error);
#line 75
}
#line 75
# 517 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline void Stm25pLogP__Read__default__seekDone(uint8_t id, error_t error)
#line 517
{
}

# 115 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogRead.nc"
inline static void Stm25pLogP__Read__seekDone(uint8_t arg_0x2ba081bb91a0, error_t error){
#line 115
    Stm25pLogP__Read__default__seekDone(arg_0x2ba081bb91a0, error);
#line 115
}
#line 115
# 60 "SummarizerC.nc"
static inline void SummarizerC__LogWrite__eraseDone(error_t error)
#line 60
{
}

# 518 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline void Stm25pLogP__Write__default__eraseDone(uint8_t id, error_t error)
#line 518
{
}

# 100 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
inline static void Stm25pLogP__Write__eraseDone(uint8_t arg_0x2ba081bb8538, error_t error){
#line 100
  switch (arg_0x2ba081bb8538) {
#line 100
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 100
      SummarizerC__LogWrite__eraseDone(error);
#line 100
      break;
#line 100
    default:
#line 100
      Stm25pLogP__Write__default__eraseDone(arg_0x2ba081bb8538, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void FlashSamplerC__Timer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(0U, dt);
#line 62
}
#line 62
# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Off(void )
#line 83
{
  LedsP__Led1__set();
  ;
#line 85
  ;
}

# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
inline static void FlashSamplerC__Leds__led1Off(void ){
#line 66
  LedsP__Leds__led1Off();
#line 66
}
#line 66
# 39 "FlashSamplerC.nc"
static inline void FlashSamplerC__Summary__summarized(error_t ok)
#line 39
{
  FlashSamplerC__Leds__led1Off();
  FlashSamplerC__Timer__startOneShot(SAMPLE_INTERVAL);
}

# 13 "Summary.nc"
inline static void SummarizerC__Summary__summarized(error_t ok){
#line 13
  FlashSamplerC__Summary__summarized(ok);
#line 13
}
#line 13
# 51 "SummarizerC.nc"
static inline void SummarizerC__LogWrite__appendDone(void *buf, storage_len_t len, bool recordsLost, 
error_t error)
#line 52
{

  SummarizerC__Summary__summarized(error);
}

# 519 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline void Stm25pLogP__Write__default__appendDone(uint8_t id, void *data, storage_len_t len, bool recordsLost, error_t error)
#line 519
{
}

# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
inline static void Stm25pLogP__Write__appendDone(uint8_t arg_0x2ba081bb8538, void * buf, storage_len_t len, bool recordsLost, error_t error){
#line 68
  switch (arg_0x2ba081bb8538) {
#line 68
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 68
      SummarizerC__LogWrite__appendDone(buf, len, recordsLost, error);
#line 68
      break;
#line 68
    default:
#line 68
      Stm25pLogP__Write__default__appendDone(arg_0x2ba081bb8538, buf, len, recordsLost, error);
#line 68
      break;
#line 68
    }
#line 68
}
#line 68
# 62 "SummarizerC.nc"
static inline void SummarizerC__LogWrite__syncDone(error_t error)
#line 62
{
}

# 520 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline void Stm25pLogP__Write__default__syncDone(uint8_t id, error_t error)
#line 520
{
}

# 118 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/LogWrite.nc"
inline static void Stm25pLogP__Write__syncDone(uint8_t arg_0x2ba081bb8538, error_t error){
#line 118
  switch (arg_0x2ba081bb8538) {
#line 118
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 118
      SummarizerC__LogWrite__syncDone(error);
#line 118
      break;
#line 118
    default:
#line 118
      Stm25pLogP__Write__default__syncDone(arg_0x2ba081bb8538, error);
#line 118
      break;
#line 118
    }
#line 118
}
#line 118
# 202 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline uint8_t Stm25pLogP__calcSector(uint8_t client, stm25p_addr_t addr)
#line 202
{
  uint8_t sector = Stm25pLogP__Sector__getNumSectors(client);

#line 204
  return (uint8_t )((addr >> STM25P_SECTOR_SIZE_LOG2) % sector);
}

#line 526
static inline error_t Stm25pLogP__Sector__default__erase(uint8_t id, uint8_t sector, uint8_t num_sectors)
#line 526
{
#line 526
  return FAIL;
}

# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static error_t Stm25pLogP__Sector__erase(uint8_t arg_0x2ba081bb6468, uint8_t sector, uint8_t num_sectors){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  switch (arg_0x2ba081bb6468) {
#line 112
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 112
      __nesc_result = Stm25pSectorP__Sector__erase(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID, sector, num_sectors);
#line 112
      break;
#line 112
    default:
#line 112
      __nesc_result = Stm25pLogP__Sector__default__erase(arg_0x2ba081bb6468, sector, num_sectors);
#line 112
      break;
#line 112
    }
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t Stm25pSectorP__signalDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(Stm25pSectorP__signalDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 256 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__signalDone(error_t error)
#line 256
{
  Stm25pSectorP__m_error = error;
  Stm25pSectorP__signalDone_task__postTask();
}

#line 246
static inline void Stm25pSectorP__Spi__computeCrcDone(uint16_t crc, stm25p_addr_t addr, 
stm25p_len_t len, error_t error)
#line 247
{
  Stm25pSectorP__m_crc = crc;
  Stm25pSectorP__signalDone(SUCCESS);
}

# 101 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
inline static void Stm25pSpiP__Spi__computeCrcDone(uint16_t crc, stm25p_addr_t addr, stm25p_len_t len, error_t error){
#line 101
  Stm25pSectorP__Spi__computeCrcDone(crc, addr, len, error);
#line 101
}
#line 101
# 183 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__Spi__readDone(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error)
#line 184
{
  Stm25pSectorP__signalDone(error);
}

# 77 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
inline static void Stm25pSpiP__Spi__readDone(stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, error_t error){
#line 77
  Stm25pSectorP__Spi__readDone(addr, buf, len, error);
#line 77
}
#line 77
#line 114
inline static error_t Stm25pSectorP__Spi__pageProgram(stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len){
#line 114
  unsigned char __nesc_result;
#line 114

#line 114
  __nesc_result = Stm25pSpiP__Spi__pageProgram(addr, buf, len);
#line 114

#line 114
  return __nesc_result;
#line 114
}
#line 114
# 202 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__Spi__pageProgramDone(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error)
#line 203
{
  addr += len;
  buf += len;
  Stm25pSectorP__m_cur_len -= len;
  if (!Stm25pSectorP__m_cur_len) {
    Stm25pSectorP__signalDone(SUCCESS);
    }
  else {
#line 210
    Stm25pSectorP__Spi__pageProgram(addr, buf, Stm25pSectorP__calcWriteLen(addr));
    }
}

# 125 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
inline static void Stm25pSpiP__Spi__pageProgramDone(stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, error_t error){
#line 125
  Stm25pSectorP__Spi__pageProgramDone(addr, buf, len, error);
#line 125
}
#line 125
#line 136
inline static error_t Stm25pSectorP__Spi__sectorErase(uint8_t sector){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = Stm25pSpiP__Spi__sectorErase(sector);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 226 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__Spi__sectorEraseDone(uint8_t sector, error_t error)
#line 226
{
  if (++Stm25pSectorP__m_cur_len < Stm25pSectorP__m_len) {
    Stm25pSectorP__Spi__sectorErase(STM25P_VMAP[Stm25pSectorP__getVolumeId(Stm25pSectorP__m_client)].base + Stm25pSectorP__m_addr + 
    Stm25pSectorP__m_cur_len);
    }
  else {
#line 231
    Stm25pSectorP__signalDone(error);
    }
}

# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
inline static void Stm25pSpiP__Spi__sectorEraseDone(uint8_t sector, error_t error){
#line 144
  Stm25pSectorP__Spi__sectorEraseDone(sector, error);
#line 144
}
#line 144
# 252 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__Spi__bulkEraseDone(error_t error)
#line 252
{
}

# 159 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpi.nc"
inline static void Stm25pSpiP__Spi__bulkEraseDone(error_t error){
#line 159
  Stm25pSectorP__Spi__bulkEraseDone(error);
#line 159
}
#line 159
# 190 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 190
{
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(uint8_t arg_0x2ba081e92df8, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 71
  switch (arg_0x2ba081e92df8) {
#line 71
    case /*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 71
      Stm25pSpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    default:
#line 71
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(arg_0x2ba081e92df8, txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 183 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void )
#line 183
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len, 
  SUCCESS);
}

#line 166
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void )
#line 166
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 167
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
#line 167
    __nesc_atomic_end(__nesc_atomic); }
}

# 80 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/crc.h"
static inline uint16_t crcByte(uint16_t crc, uint8_t b)
#line 80
{
  crc = (uint8_t )(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t )(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = Stm25pSectorP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static inline error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__default__start(void )
#line 92
{
  return SUCCESS;
}

# 74 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/StdControl.nc"
inline static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__default__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(1U);
#line 67
}
#line 67
# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask__runTask(void )
#line 69
{
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__stop();
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__stopTimer = FALSE;
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__start();
  if (/*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__start() == EALREADY) {
    /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__ResourceDefaultOwner__release();
    }
}

# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(1U, dt);
#line 62
}
#line 62
# 77 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask__runTask(void )
#line 77
{
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__startOneShot(1024);
}

# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id)
#line 112
{
#line 112
  return FAIL;
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(uint8_t arg_0x2ba081e91be8){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0x2ba081e91be8) {
#line 78
    case /*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 78
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(/*HplStm25pSpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(arg_0x2ba081e91be8);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 73 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id)
#line 73
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(id);
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pSpiP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(/*HplStm25pSpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 108 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static inline error_t Stm25pSpiP__ClientResource__request(void )
#line 108
{
  return Stm25pSpiP__SpiResource__request();
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Stm25pSectorP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = Stm25pSpiP__ClientResource__request();
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 121 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__Stm25pResource__granted(uint8_t id)
#line 121
{
  Stm25pSectorP__m_client = id;
  Stm25pSectorP__SpiResource__request();
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x2ba081d44d40){
#line 92
  Stm25pSectorP__Stm25pResource__granted(arg_0x2ba081d44d40);
#line 92
}
#line 92
# 213 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x2ba081d3f340){
#line 49
    /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x2ba081d3f340);
#line 49
}
#line 49
# 187 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 525 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline error_t Stm25pLogP__Sector__default__write(uint8_t id, storage_addr_t addr, uint8_t *buf, storage_len_t len)
#line 525
{
#line 525
  return FAIL;
}

# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static error_t Stm25pLogP__Sector__write(uint8_t arg_0x2ba081bb6468, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len){
#line 91
  unsigned char __nesc_result;
#line 91

#line 91
  switch (arg_0x2ba081bb6468) {
#line 91
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 91
      __nesc_result = Stm25pSectorP__Sector__write(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID, addr, buf, len);
#line 91
      break;
#line 91
    default:
#line 91
      __nesc_result = Stm25pLogP__Sector__default__write(arg_0x2ba081bb6468, addr, buf, len);
#line 91
      break;
#line 91
    }
#line 91

#line 91
  return __nesc_result;
#line 91
}
#line 91
# 446 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline void Stm25pLogP__Sector__eraseDone(uint8_t id, uint8_t sector, 
uint8_t num_sectors, 
error_t error)
#line 448
{
  if (Stm25pLogP__m_log_state[id].req == Stm25pLogP__S_ERASE) {
      Stm25pLogP__m_log_info[id].read_addr = 0;
      Stm25pLogP__m_log_info[id].write_addr = 0;
      Stm25pLogP__signalDone(id, error);
    }
  else {


      stm25p_addr_t volume_size = 
      STM25P_SECTOR_SIZE * (Stm25pLogP__Sector__getNumSectors(id) - 1);

#line 459
      if (Stm25pLogP__m_log_info[id].write_addr > volume_size) {
          stm25p_addr_t read_addr = Stm25pLogP__m_log_info[id].write_addr - volume_size;

#line 461
          if (Stm25pLogP__m_log_info[id].read_addr < read_addr) {
            Stm25pLogP__m_log_info[id].read_addr = read_addr;
            }
        }
#line 464
      Stm25pLogP__m_addr = Stm25pLogP__m_log_info[id].write_addr;
      Stm25pLogP__Sector__write(id, Stm25pLogP__calcAddr(id, Stm25pLogP__m_addr), (uint8_t *)&Stm25pLogP__m_addr, 
      sizeof Stm25pLogP__m_addr);
    }
}

# 176 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline void Stm25pBlockP__Sector__eraseDone(uint8_t id, uint8_t sector, 
uint8_t num_sectors, 
error_t error)
#line 178
{
  Stm25pBlockP__signalDone(id, 0, error);
}

# 287 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__Sector__default__eraseDone(uint8_t id, uint8_t sector, uint8_t num_sectors, error_t error)
#line 287
{
}

# 121 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static void Stm25pSectorP__Sector__eraseDone(uint8_t arg_0x2ba081c93328, uint8_t sector, uint8_t num_sectors, error_t error){
#line 121
  switch (arg_0x2ba081c93328) {
#line 121
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID:
#line 121
      Stm25pLogP__Sector__eraseDone(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID, sector, num_sectors, error);
#line 121
      break;
#line 121
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID:
#line 121
      Stm25pBlockP__Sector__eraseDone(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID, sector, num_sectors, error);
#line 121
      break;
#line 121
    default:
#line 121
      Stm25pSectorP__Sector__default__eraseDone(arg_0x2ba081c93328, sector, num_sectors, error);
#line 121
      break;
#line 121
    }
#line 121
}
#line 121
# 470 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline void Stm25pLogP__Sector__writeDone(uint8_t id, storage_addr_t addr, 
uint8_t *buf, 
storage_len_t len, 
error_t error)
#line 473
{
  Stm25pLogP__m_log_info[id].write_addr += len;
  if (Stm25pLogP__m_rw_state == Stm25pLogP__S_HEADER) {
      if (len == sizeof Stm25pLogP__m_header) {
        Stm25pLogP__m_rw_state = Stm25pLogP__S_DATA;
        }
#line 478
      Stm25pLogP__continueAppendOp(id);
    }
  else {
      Stm25pLogP__signalDone(id, error);
    }
}

# 171 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline void Stm25pBlockP__Sector__writeDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error)
#line 172
{
  Stm25pBlockP__signalDone(id, 0, error);
}

# 286 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__Sector__default__writeDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, error_t error)
#line 286
{
}

# 101 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static void Stm25pSectorP__Sector__writeDone(uint8_t arg_0x2ba081c93328, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, error_t error){
#line 101
  switch (arg_0x2ba081c93328) {
#line 101
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID:
#line 101
      Stm25pLogP__Sector__writeDone(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID, addr, buf, len, error);
#line 101
      break;
#line 101
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID:
#line 101
      Stm25pBlockP__Sector__writeDone(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID, addr, buf, len, error);
#line 101
      break;
#line 101
    default:
#line 101
      Stm25pSectorP__Sector__default__writeDone(arg_0x2ba081c93328, addr, buf, len, error);
#line 101
      break;
#line 101
    }
#line 101
}
#line 101
# 514 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline void Stm25pLogP__Sector__computeCrcDone(uint8_t id, stm25p_addr_t addr, stm25p_len_t len, uint16_t crc, error_t error)
#line 514
{
}

# 182 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline void Stm25pBlockP__Sector__computeCrcDone(uint8_t id, stm25p_addr_t addr, 
stm25p_len_t len, 
uint16_t crc, 
error_t error)
#line 185
{
  Stm25pBlockP__signalDone(id, crc, error);
}

# 288 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__Sector__default__computeCrcDone(uint8_t id, stm25p_addr_t addr, stm25p_len_t len, uint16_t crc, error_t error)
#line 288
{
}

# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static void Stm25pSectorP__Sector__computeCrcDone(uint8_t arg_0x2ba081c93328, stm25p_addr_t addr, stm25p_len_t len, uint16_t crc, error_t error){
#line 144
  switch (arg_0x2ba081c93328) {
#line 144
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID:
#line 144
      Stm25pLogP__Sector__computeCrcDone(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID, addr, len, crc, error);
#line 144
      break;
#line 144
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID:
#line 144
      Stm25pBlockP__Sector__computeCrcDone(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID, addr, len, crc, error);
#line 144
      break;
#line 144
    default:
#line 144
      Stm25pSectorP__Sector__default__computeCrcDone(arg_0x2ba081c93328, addr, len, crc, error);
#line 144
      break;
#line 144
    }
#line 144
}
#line 144
# 524 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline error_t Stm25pLogP__Sector__default__read(uint8_t id, storage_addr_t addr, uint8_t *buf, storage_len_t len)
#line 524
{
#line 524
  return FAIL;
}

# 68 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static error_t Stm25pLogP__Sector__read(uint8_t arg_0x2ba081bb6468, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  switch (arg_0x2ba081bb6468) {
#line 68
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID:
#line 68
      __nesc_result = Stm25pSectorP__Sector__read(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID, addr, buf, len);
#line 68
      break;
#line 68
    default:
#line 68
      __nesc_result = Stm25pLogP__Sector__default__read(arg_0x2ba081bb6468, addr, buf, len);
#line 68
      break;
#line 68
    }
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 313 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline void Stm25pLogP__Sector__readDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error)
#line 314
{

  Stm25pLogP__stm25p_log_info_t *log_info = &Stm25pLogP__m_log_info[id];


  switch (Stm25pLogP__m_rw_state) {
      case Stm25pLogP__S_SEARCH_BLOCKS: 
        {
          uint16_t block = addr >> Stm25pLogP__BLOCK_SIZE_LOG2;

          if (Stm25pLogP__m_addr != STM25P_INVALID_ADDRESS) {
              if (Stm25pLogP__m_addr < log_info->read_addr) {
                log_info->read_addr = Stm25pLogP__m_addr;
                }
#line 327
              if (Stm25pLogP__m_addr > log_info->write_addr) {
                log_info->write_addr = Stm25pLogP__m_addr;
                }
            }
          if (++block < Stm25pLogP__Sector__getNumSectors(id) * Stm25pLogP__BLOCKS_PER_SECTOR) {
              addr += Stm25pLogP__BLOCK_SIZE;
              Stm25pLogP__Sector__read(id, addr, (uint8_t *)&Stm25pLogP__m_addr, sizeof Stm25pLogP__m_addr);
            }
          else {
            if (log_info->read_addr == STM25P_INVALID_ADDRESS) {
                log_info->read_addr = 0;
                log_info->write_addr = 0;
                Stm25pLogP__ClientResource__granted(id);
              }
            else 
              {
                log_info->write_addr += sizeof Stm25pLogP__m_addr;
                Stm25pLogP__m_rw_state = Stm25pLogP__S_SEARCH_RECORDS;
                Stm25pLogP__Sector__read(id, Stm25pLogP__calcAddr(id, log_info->write_addr), &Stm25pLogP__m_header, 
                sizeof Stm25pLogP__m_header);
              }
            }
        }
#line 349
      break;

      case Stm25pLogP__S_SEARCH_RECORDS: 
        {

          uint16_t cur_block = log_info->write_addr >> Stm25pLogP__BLOCK_SIZE_LOG2;
          uint16_t new_block = (log_info->write_addr + sizeof Stm25pLogP__m_header + 
          Stm25pLogP__m_header) >> Stm25pLogP__BLOCK_SIZE_LOG2;

          if (Stm25pLogP__m_header != Stm25pLogP__INVALID_HEADER && cur_block == new_block) {
              log_info->write_addr += sizeof Stm25pLogP__m_header + Stm25pLogP__m_header;
              Stm25pLogP__Sector__read(id, Stm25pLogP__calcAddr(id, log_info->write_addr), 
              &Stm25pLogP__m_header, sizeof Stm25pLogP__m_header);
            }
          else 
            {
              Stm25pLogP__ClientResource__granted(id);
            }
        }
      break;

      case Stm25pLogP__S_SEARCH_SEEK: 
        {

          log_info->read_addr += sizeof Stm25pLogP__m_header + Stm25pLogP__m_header;

          if (log_info->read_addr < Stm25pLogP__m_log_state[id].cookie) {
              Stm25pLogP__Sector__read(id, Stm25pLogP__calcAddr(id, log_info->read_addr), &Stm25pLogP__m_header, 
              sizeof Stm25pLogP__m_header);
            }
          else 
            {
              log_info->remaining = log_info->read_addr - Stm25pLogP__m_log_state[id].cookie;
              log_info->read_addr = Stm25pLogP__m_log_state[id].cookie;
              Stm25pLogP__signalDone(id, error);
            }
        }
      break;

      case Stm25pLogP__S_HEADER: 
        {

          if (Stm25pLogP__m_header == Stm25pLogP__INVALID_HEADER) {
              log_info->read_addr += Stm25pLogP__BLOCK_SIZE;
              log_info->read_addr &= ~Stm25pLogP__BLOCK_MASK;
            }
          else {
              log_info->read_addr += sizeof Stm25pLogP__m_header;
              log_info->remaining = Stm25pLogP__m_header;
              Stm25pLogP__m_rw_state = Stm25pLogP__S_DATA;
            }
          Stm25pLogP__continueReadOp(id);
        }
      break;

      case Stm25pLogP__S_DATA: 
        {
          log_info->read_addr += len;
          log_info->remaining -= len;
          Stm25pLogP__m_len -= len;
          Stm25pLogP__m_rw_state = Stm25pLogP__S_HEADER;
          Stm25pLogP__continueReadOp(id);
          break;
        }
    }
}

# 166 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline void Stm25pBlockP__Sector__readDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len, error_t error)
#line 167
{
  Stm25pBlockP__signalDone(id, 0, error);
}

# 285 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__Sector__default__readDone(uint8_t id, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, error_t error)
#line 285
{
}

# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSector.nc"
inline static void Stm25pSectorP__Sector__readDone(uint8_t arg_0x2ba081c93328, stm25p_addr_t addr, uint8_t *buf, stm25p_len_t len, error_t error){
#line 78
  switch (arg_0x2ba081c93328) {
#line 78
    case /*FlashSamplerAppC.LogStorageC*/LogStorageC__0__VOLUME_ID:
#line 78
      Stm25pLogP__Sector__readDone(/*FlashSamplerAppC.LogStorageC*/LogStorageC__0__LOG_ID, addr, buf, len, error);
#line 78
      break;
#line 78
    case /*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__VOLUME_ID:
#line 78
      Stm25pBlockP__Sector__readDone(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID, addr, buf, len, error);
#line 78
      break;
#line 78
    default:
#line 78
      Stm25pSectorP__Sector__default__readDone(arg_0x2ba081c93328, addr, buf, len, error);
#line 78
      break;
#line 78
    }
#line 78
}
#line 78
# 261 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline void Stm25pSectorP__signalDone_task__runTask(void )
#line 261
{
  switch (Stm25pSectorP__m_state) {
      case Stm25pSectorP__S_IDLE: 
        Stm25pSectorP__ClientResource__granted(Stm25pSectorP__m_client);
      break;
      case Stm25pSectorP__S_READ: 
        Stm25pSectorP__Sector__readDone(Stm25pSectorP__m_client, Stm25pSectorP__m_addr, Stm25pSectorP__m_buf, Stm25pSectorP__m_len, Stm25pSectorP__m_error);
      break;
      case Stm25pSectorP__S_CRC: 
        Stm25pSectorP__Sector__computeCrcDone(Stm25pSectorP__m_client, Stm25pSectorP__m_addr, Stm25pSectorP__m_len, 
        Stm25pSectorP__m_crc, Stm25pSectorP__m_error);
      break;
      case Stm25pSectorP__S_WRITE: 
        Stm25pSectorP__Sector__writeDone(Stm25pSectorP__m_client, Stm25pSectorP__m_addr, Stm25pSectorP__m_buf, Stm25pSectorP__m_len, Stm25pSectorP__m_error);
      break;
      case Stm25pSectorP__S_ERASE: 
        Stm25pSectorP__Sector__eraseDone(Stm25pSectorP__m_client, Stm25pSectorP__m_addr, Stm25pSectorP__m_len, Stm25pSectorP__m_error);
      break;
      default: 
        break;
    }
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 118 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 91 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 60 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 67
}
#line 67
# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 124
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 119 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static inline error_t Stm25pBlockP__Write__erase(uint8_t id)
#line 119
{
  Stm25pBlockP__m_req.req = Stm25pBlockP__S_ERASE;
  return Stm25pBlockP__newRequest(id);
}

# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/BlockWrite.nc"
inline static error_t AccelSamplerC__BlockWrite__erase(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = Stm25pBlockP__Write__erase(/*FlashSamplerAppC.BlockStorageC*/BlockStorageC__0__BLOCK_ID);
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 25 "AccelSamplerC.nc"
static inline void AccelSamplerC__Sample__sample(void )
#line 25
{

  AccelSamplerC__BlockWrite__erase();
}

# 12 "Sample.nc"
inline static void FlashSamplerC__Sample__sample(void ){
#line 12
  AccelSamplerC__Sample__sample();
#line 12
}
#line 12
# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr();
#line 39
}
#line 39
# 38 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr();
}

# 30 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__clr(void ){
#line 30
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr();
#line 30
}
#line 30
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0On(void )
#line 63
{
  LedsP__Led0__clr();
  ;
#line 65
  ;
}

# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Leds.nc"
inline static void FlashSamplerC__Leds__led0On(void ){
#line 45
  LedsP__Leds__led0On();
#line 45
}
#line 45
# 28 "FlashSamplerC.nc"
static inline void FlashSamplerC__Timer__fired(void )
#line 28
{
  FlashSamplerC__Leds__led0On();
  FlashSamplerC__Sample__sample();
}

# 95 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static inline error_t Stm25pSectorP__SplitControl__stop(void )
#line 95
{
  error_t error = Stm25pSectorP__SpiResource__request();

#line 97
  if (error == SUCCESS) {
    Stm25pSectorP__m_power_state = Stm25pSectorP__S_STOP;
    }
#line 99
  return error;
}

# 109 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__stop(void ){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = Stm25pSectorP__SplitControl__stop();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 131 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static inline error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__default__stop(void )
#line 131
{
  return SUCCESS;
}

# 84 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/StdControl.nc"
inline static error_t /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__default__stop();
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 139 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__PowerDownCleanup__default__cleanup(void )
#line 139
{
}

# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/PowerDownCleanup.nc"
inline static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__PowerDownCleanup__cleanup(void ){
#line 52
  /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__PowerDownCleanup__default__cleanup();
#line 52
}
#line 52
# 108 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static inline void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__fired(void )
#line 108
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (/*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__stopTimer == FALSE) {
          /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__stopping = TRUE;
          /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__PowerDownCleanup__cleanup();
          /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__stop();
          if (/*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__stop() == EALREADY) {
            /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__stopDone(SUCCESS);
            }
        }
    }
#line 118
    __nesc_atomic_end(__nesc_atomic); }
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(error_t error){
#line 92
  Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error);
#line 92
}
#line 92
# 78 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadStream.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__read(uint8_t arg_0x2ba0824fb148, uint32_t usPeriod){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = AdcStreamP__ReadStream__read(arg_0x2ba0824fb148, usPeriod);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(uint8_t client)
#line 59
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__read(client, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__period[client]);
}

# 160 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__ClientResource__default__granted(uint8_t client)
#line 160
{
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void Msp430RefVoltArbiterImplP__ClientResource__granted(uint8_t arg_0x2ba0823dc298){
#line 92
  switch (arg_0x2ba0823dc298) {
#line 92
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 92
      AdcP__ResourceRead__granted(/*FlashSamplerAppC.AccelXStreamC.AdcReadClientC*/AdcReadClientC__0__CLIENT);
#line 92
      break;
#line 92
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 92
      /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT);
#line 92
      break;
#line 92
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 92
      AdcP__SubResourceReadNow__granted(/*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT);
#line 92
      break;
#line 92
    default:
#line 92
      Msp430RefVoltArbiterImplP__ClientResource__default__granted(arg_0x2ba0823dc298);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 98 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {


      Msp430RefVoltArbiterImplP__ClientResource__granted(Msp430RefVoltArbiterImplP__syncOwner);
    }
}

# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(error_t error){
#line 92
  Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error);
#line 92
}
#line 92
# 168 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP__SwitchOnTimer__fired(void )
#line 168
{
  switch (Msp430RefVoltGeneratorP__m_state) {
      case Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING: 
        Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE;
      Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(SUCCESS);
      break;

      case Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING: 
        Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE;
      Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(SUCCESS);
      break;

      default: 
        return;
    }
}

# 151 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error_t error)
{
}

# 117 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(error_t error){
#line 117
  Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error);
#line 117
}
#line 117
# 147 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error_t error)
{
}

# 117 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(error_t error){
#line 117
  Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error);
#line 117
}
#line 117
# 185 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP__SwitchOffTimer__fired(void )
#line 185
{
  switch (Msp430RefVoltGeneratorP__m_state) {
      case Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE: 
        if (Msp430RefVoltGeneratorP__switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__GENERATOR_OFF;
            Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(SUCCESS);
          }
        else {
            Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(20);
          }
      break;

      case Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE: 
        if (Msp430RefVoltGeneratorP__switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__GENERATOR_OFF;
            Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(SUCCESS);
          }
        else {
            Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(20);
          }
      break;

      default: 
        break;
    }
}

# 193 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x2ba081b2b5d8){
#line 72
  switch (arg_0x2ba081b2b5d8) {
#line 72
    case 0U:
#line 72
      FlashSamplerC__Timer__fired();
#line 72
      break;
#line 72
    case 1U:
#line 72
      /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__TimerMilli__fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      Msp430RefVoltGeneratorP__SwitchOnTimer__fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      Msp430RefVoltGeneratorP__SwitchOffTimer__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x2ba081b2b5d8);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 128 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 72 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 72
}
#line 72
# 80 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 63 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 58 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4367 {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 36 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 109 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static inline error_t Stm25pLogP__Init__init(void )
#line 109
{
  int i;

#line 111
  for (i = 0; i < Stm25pLogP__NUM_LOGS; i++) {
      Stm25pLogP__m_log_info[i].read_addr = STM25P_INVALID_ADDRESS;
      Stm25pLogP__m_log_info[i].write_addr = 0;
    }
  return SUCCESS;
}

# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P47*/HplMsp430GeneralIOP__31__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )29U |= 0x01 << 7;
}

# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P47*/HplMsp430GeneralIOP__31__IO__set();
#line 34
}
#line 34
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__HplGeneralIO__set();
}

# 29 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void Stm25pSpiP__Hold__set(void ){
#line 29
  /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 34 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__set();
#line 34
}
#line 34
# 37 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__HplGeneralIO__set();
}

# 29 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void Stm25pSpiP__CSN__set(void ){
#line 29
  /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__set();
#line 29
}
#line 29
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P47*/HplMsp430GeneralIOP__31__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 7;
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P47*/HplMsp430GeneralIOP__31__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void Stm25pSpiP__Hold__makeOutput(void ){
#line 35
  /*HplStm25pPinsC.HoldM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 4;
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__HplGeneralIO__makeOutput();
}

# 35 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void Stm25pSpiP__CSN__makeOutput(void ){
#line 35
  /*HplStm25pPinsC.CSNM*/Msp430GpioC__3__GeneralIO__makeOutput();
#line 35
}
#line 35
# 100 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static inline error_t Stm25pSpiP__Init__init(void )
#line 100
{
  Stm25pSpiP__CSN__makeOutput();
  Stm25pSpiP__Hold__makeOutput();
  Stm25pSpiP__CSN__set();
  Stm25pSpiP__Hold__set();
  return SUCCESS;
}

# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 45
{
  memset(/*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

#line 45
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 45
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 123 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__stopConversion(void ){
#line 123
  HplAdc12P__HplAdc12__stopConversion();
#line 123
}
#line 123
# 92 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP__Init__init(void )
{
  adc12ctl0_t ctl0;

#line 95
  Msp430Adc12ImplP__HplAdc12__stopConversion();
  ctl0 = Msp430Adc12ImplP__HplAdc12__getCtl0();
  ctl0.adc12tovie = 1;
  ctl0.adc12ovie = 1;
  Msp430Adc12ImplP__HplAdc12__setCtl0(ctl0);
  return SUCCESS;
}

# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/RoundRobinResourceQueueC.nc"
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init(void )
#line 51
{
  memset(/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ, 0, sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ);
  return SUCCESS;
}

# 83 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__Init__init(void )
#line 83
{
  uint8_t i;

  for (i = 0; i != AdcStreamP__NSTREAM; i++) 
    AdcStreamP__bufferQueueEnd[i] = &AdcStreamP__bufferQueue[i];

  return SUCCESS;
}

# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = AdcStreamP__Init__init();
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, Msp430Adc12ImplP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Stm25pSectorC.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, Stm25pSpiP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, Stm25pLogP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 24 "FlashSamplerC.nc"
static inline void FlashSamplerC__Boot__booted(void )
#line 24
{
  FlashSamplerC__Timer__fired();
}

# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 49
  FlashSamplerC__Boot__booted();
#line 49
}
#line 49
# 206 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 52
{
  return MSP430_POWER_LPM3;
}

# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 69
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 91
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 92
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 194 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 194
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 104
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 109
{
  uint16_t temp;

#line 111
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 59
  McuSleepC__McuSleep__sleep();
#line 59
}
#line 59
# 67 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 72
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 61 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 61
  SchedulerBasicP__Scheduler__taskLoop();
#line 61
}
#line 61
# 88 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 349 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableRxIntr(void )
#line 349
{
  HplMsp430Usart0P__IE1 &= ~(1 << 6);
}

# 177 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void ){
#line 177
  HplMsp430Usart0P__Usart__disableRxIntr();
#line 177
}
#line 177
# 170 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 170
{

  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else 
#line 177
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
    }
}

# 65 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x2ba081fe8108, uint8_t data){
#line 54
  switch (arg_0x2ba081fe8108) {
#line 54
    case /*HplStm25pSpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x2ba081fe8108, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 80 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse();
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isI2C(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  {
    unsigned char __nesc_temp = 
#line 56
    HplMsp430I2C0P__U0CTL & 0x20 && HplMsp430I2C0P__U0CTL & 0x04 && HplMsp430I2C0P__U0CTL & 0x01;

#line 56
    return __nesc_temp;
  }
}

# 6 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool HplMsp430Usart0P__HplI2C__isI2C(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = HplMsp430I2C0P__HplI2C__isI2C();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(uint8_t arg_0x2ba081fe7020){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(arg_0x2ba081fe7020);
#line 39
}
#line 39
# 59 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 39 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 188 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void )
#line 188
{
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x2ba081fe8108){
#line 49
  switch (arg_0x2ba081fe8108) {
#line 49
    case /*HplStm25pSpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x2ba081fe8108);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 161 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static inline uint16_t *AdcP__SingleChannel__multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples)
{

  return 0;
}

# 645 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline uint16_t *Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples)
{
  return 0;
}

# 227 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static uint16_t * Msp430Adc12ImplP__SingleChannel__multipleDataReady(uint8_t arg_0x2ba082223ac0, uint16_t * buffer, uint16_t numSamples){
#line 227
  unsigned int *__nesc_result;
#line 227

#line 227
  switch (arg_0x2ba082223ac0) {
#line 227
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 227
      __nesc_result = AdcP__SingleChannel__multipleDataReady(/*FlashSamplerAppC.AccelXStreamC.AdcReadClientC*/AdcReadClientC__0__CLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 227
      __nesc_result = AdcStreamP__SingleChannel__multipleDataReady(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 227
      __nesc_result = AdcP__SingleChannel__multipleDataReady(/*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    default:
#line 227
      __nesc_result = Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(arg_0x2ba082223ac0, buffer, numSamples);
#line 227
      break;
#line 227
    }
#line 227

#line 227
  return __nesc_result;
#line 227
}
#line 227
# 88 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline uint16_t HplAdc12P__HplAdc12__getMem(uint8_t i)
#line 88
{
  return ((int *)0x0140)[i];
}

# 89 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static uint16_t Msp430Adc12ImplP__HplAdc12__getMem(uint8_t idx){
#line 89
  unsigned int __nesc_result;
#line 89

#line 89
  __nesc_result = HplAdc12P__HplAdc12__getMem(idx);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  adc12memctl_t HplAdc12P__int2adc12memctl(uint8_t x)
#line 62
{
#line 62
  union __nesc_unnamed4368 {
#line 62
    uint8_t f;
#line 62
    adc12memctl_t t;
  } 
#line 62
  c = { .f = x };

#line 62
  return c.t;
}

#line 84
static inline adc12memctl_t HplAdc12P__HplAdc12__getMCtl(uint8_t i)
#line 84
{
  return HplAdc12P__int2adc12memctl(((char *)0x0080)[i]);
}

# 82 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static adc12memctl_t Msp430Adc12ImplP__HplAdc12__getMCtl(uint8_t idx){
#line 82
  struct __nesc_unnamed4290 __nesc_result;
#line 82

#line 82
  __nesc_result = HplAdc12P__HplAdc12__getMCtl(idx);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 651 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__MultiChannel__default__dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples)
#line 651
{
}

# 107 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
inline static void Msp430Adc12ImplP__MultiChannel__dataReady(uint8_t arg_0x2ba0822210c8, uint16_t *buffer, uint16_t numSamples){
#line 107
    Msp430Adc12ImplP__MultiChannel__default__dataReady(arg_0x2ba0822210c8, buffer, numSamples);
#line 107
}
#line 107
# 640 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP__SingleChannel__default__singleDataReady(uint8_t id, uint16_t data)
{
  return FAIL;
}

# 206 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t Msp430Adc12ImplP__SingleChannel__singleDataReady(uint8_t arg_0x2ba082223ac0, uint16_t data){
#line 206
  unsigned char __nesc_result;
#line 206

#line 206
  switch (arg_0x2ba082223ac0) {
#line 206
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 206
      __nesc_result = AdcP__SingleChannel__singleDataReady(/*FlashSamplerAppC.AccelXStreamC.AdcReadClientC*/AdcReadClientC__0__CLIENT, data);
#line 206
      break;
#line 206
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 206
      __nesc_result = AdcStreamP__SingleChannel__singleDataReady(/*FlashSamplerAppC.AccelXStreamC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT, data);
#line 206
      break;
#line 206
    case /*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 206
      __nesc_result = AdcP__SingleChannel__singleDataReady(/*FlashSamplerAppC.AccelXStreamC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT, data);
#line 206
      break;
#line 206
    default:
#line 206
      __nesc_result = Msp430Adc12ImplP__SingleChannel__default__singleDataReady(arg_0x2ba082223ac0, data);
#line 206
      break;
#line 206
    }
#line 206

#line 206
  return __nesc_result;
#line 206
}
#line 206
# 654 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(uint8_t id)
#line 654
{
}

# 54 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static void Msp430Adc12ImplP__Overflow__conversionTimeOverflow(uint8_t arg_0x2ba08221f020){
#line 54
    Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(arg_0x2ba08221f020);
#line 54
}
#line 54
# 653 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__Overflow__default__memOverflow(uint8_t id)
#line 653
{
}

# 49 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static void Msp430Adc12ImplP__Overflow__memOverflow(uint8_t arg_0x2ba08221f020){
#line 49
    Msp430Adc12ImplP__Overflow__default__memOverflow(arg_0x2ba08221f020);
#line 49
}
#line 49
# 544 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__HplAdc12__conversionDone(uint16_t iv)
{
  bool overflow = FALSE;
  uint16_t *resultBuffer;

  if (iv <= 4) {
      if (iv == 2) {
        Msp430Adc12ImplP__Overflow__memOverflow(Msp430Adc12ImplP__clientID);
        }
      else {
#line 553
        Msp430Adc12ImplP__Overflow__conversionTimeOverflow(Msp430Adc12ImplP__clientID);
        }
      if (! Msp430Adc12ImplP__HplAdc12__getCtl0().msc) {
        overflow = TRUE;
        }
    }
#line 558
  switch (Msp430Adc12ImplP__state & Msp430Adc12ImplP__CONVERSION_MODE_MASK) 
    {
      case Msp430Adc12ImplP__SINGLE_DATA: 
        Msp430Adc12ImplP__stopConversion();
      Msp430Adc12ImplP__SingleChannel__singleDataReady(Msp430Adc12ImplP__clientID, Msp430Adc12ImplP__HplAdc12__getMem(0));
      break;
      case Msp430Adc12ImplP__SINGLE_DATA_REPEAT: 
        {
          error_t repeatContinue;

#line 567
          repeatContinue = Msp430Adc12ImplP__SingleChannel__singleDataReady(Msp430Adc12ImplP__clientID, 
          Msp430Adc12ImplP__HplAdc12__getMem(0));
          if (repeatContinue != SUCCESS) {
            Msp430Adc12ImplP__stopConversion();
            }
#line 571
          break;
        }

      case Msp430Adc12ImplP__MULTI_CHANNEL: 
        {
          uint16_t i = 0;
#line 576
          uint16_t k;

#line 577
          resultBuffer = Msp430Adc12ImplP__resultBufferStart + Msp430Adc12ImplP__resultBufferIndex;
          do {
              * resultBuffer++ = Msp430Adc12ImplP__HplAdc12__getMem(i);
            }
          while (
#line 580
          ++i < Msp430Adc12ImplP__numChannels);
          Msp430Adc12ImplP__resultBufferIndex += Msp430Adc12ImplP__numChannels;
          if (overflow || Msp430Adc12ImplP__resultBufferLength == Msp430Adc12ImplP__resultBufferIndex) {
              Msp430Adc12ImplP__stopConversion();
              resultBuffer -= Msp430Adc12ImplP__resultBufferIndex;
              k = Msp430Adc12ImplP__resultBufferIndex - Msp430Adc12ImplP__numChannels;
              Msp430Adc12ImplP__resultBufferIndex = 0;
              Msp430Adc12ImplP__MultiChannel__dataReady(Msp430Adc12ImplP__clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP__resultBufferLength);
            }
        }
      break;
      case Msp430Adc12ImplP__MULTIPLE_DATA: 
        {
          uint16_t i = 0;
#line 594
          uint16_t length;
#line 594
          uint16_t k;

#line 595
          resultBuffer = Msp430Adc12ImplP__resultBufferStart + Msp430Adc12ImplP__resultBufferIndex;
          if (Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex > 16) {
            length = 16;
            }
          else {
#line 599
            length = Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex;
            }
#line 600
          do {
              * resultBuffer++ = Msp430Adc12ImplP__HplAdc12__getMem(i);
            }
          while (
#line 602
          ++i < length);
          Msp430Adc12ImplP__resultBufferIndex += length;
          if (overflow || Msp430Adc12ImplP__resultBufferLength == Msp430Adc12ImplP__resultBufferIndex) {
              Msp430Adc12ImplP__stopConversion();
              resultBuffer -= Msp430Adc12ImplP__resultBufferIndex;
              k = Msp430Adc12ImplP__resultBufferIndex - length;
              Msp430Adc12ImplP__resultBufferIndex = 0;
              Msp430Adc12ImplP__SingleChannel__multipleDataReady(Msp430Adc12ImplP__clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP__resultBufferLength);
            }
          else {
#line 611
            if (Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex > 15) {
              return;
              }
            else 
#line 613
              {

                adc12memctl_t memctl = Msp430Adc12ImplP__HplAdc12__getMCtl(0);

#line 616
                memctl.eos = 1;
                Msp430Adc12ImplP__HplAdc12__setMCtl(Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex, memctl);
              }
            }
        }
#line 620
      break;
      case Msp430Adc12ImplP__MULTIPLE_DATA_REPEAT: 
        {
          uint8_t i = 0;

#line 624
          resultBuffer = Msp430Adc12ImplP__resultBufferStart;
          do {
              * resultBuffer++ = Msp430Adc12ImplP__HplAdc12__getMem(i);
            }
          while (
#line 627
          ++i < Msp430Adc12ImplP__resultBufferLength);

          Msp430Adc12ImplP__resultBufferStart = Msp430Adc12ImplP__SingleChannel__multipleDataReady(Msp430Adc12ImplP__clientID, 
          resultBuffer - Msp430Adc12ImplP__resultBufferLength, 
          overflow ? 0 : Msp430Adc12ImplP__resultBufferLength);
          if (!Msp430Adc12ImplP__resultBufferStart) {
            Msp430Adc12ImplP__stopConversion();
            }
#line 634
          break;
        }
    }
}

# 213 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP__HplAdc12__conversionDone(uint16_t iv)
#line 213
{
}

# 112 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void HplAdc12P__HplAdc12__conversionDone(uint16_t iv){
#line 112
  Msp430RefVoltGeneratorP__HplAdc12__conversionDone(iv);
#line 112
  Msp430Adc12ImplP__HplAdc12__conversionDone(iv);
#line 112
}
#line 112
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 0);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port60__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 1);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port61__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 2);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port62__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 3);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port63__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 4);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port64__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 5);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port65__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 6);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port66__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )55U &= ~(0x01 << 7);
}

# 85 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port67__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc();
#line 85
}
#line 85
# 95 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P__HplAdc12__resetIFGs(void )
#line 95
{
  HplAdc12P__ADC12IV = 0;
  HplAdc12P__ADC12IFG = 0;
}

# 106 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__resetIFGs(void ){
#line 106
  HplAdc12P__HplAdc12__resetIFGs();
#line 106
}
#line 106
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t AdcP__readDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcP__readDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 178 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__ReadNow__default__readDone(uint8_t client, error_t result, uint16_t val)
#line 178
{
}

# 66 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/ReadNow.nc"
inline static void AdcP__ReadNow__readDone(uint8_t arg_0x2ba0821af318, error_t result, AdcP__ReadNow__val_t val){
#line 66
    AdcP__ReadNow__default__readDone(arg_0x2ba0821af318, result, val);
#line 66
}
#line 66
# 56 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP__bufferDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcStreamP__bufferDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 226 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 228
    uint16_t __x;

#line 228
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 228
   & 0x0008) != 0;

#line 229
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(12)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 169 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(10)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(26)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 135 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 28 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x2ba0816dac98){
#line 28
  switch (arg_0x2ba0816dac98) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x2ba0816dac98);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 159 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 69 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t * )400U;
    }
}

# 394 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static error_t Msp430Adc12ImplP__SingleChannel__getData(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 396
    {
      if (Msp430Adc12ImplP__ADCArbiterInfo__userId() == id) {
          if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__MULTIPLE_DATA_REPEAT && !Msp430Adc12ImplP__resultBufferStart) 
            {
              unsigned char __nesc_temp = 
#line 399
              EINVAL;

              {
#line 399
                __nesc_atomic_end(__nesc_atomic); 
#line 399
                return __nesc_temp;
              }
            }
#line 400
          if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__ADC_BUSY) 
            {
              unsigned char __nesc_temp = 
#line 401
              EBUSY;

              {
#line 401
                __nesc_atomic_end(__nesc_atomic); 
#line 401
                return __nesc_temp;
              }
            }
#line 402
          Msp430Adc12ImplP__state |= Msp430Adc12ImplP__ADC_BUSY;
          Msp430Adc12ImplP__clientID = id;
          Msp430Adc12ImplP__configureAdcPin(Msp430Adc12ImplP__HplAdc12__getMCtl(0).inch);
          Msp430Adc12ImplP__HplAdc12__startConversion();
          if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__USE_TIMERA) {
            Msp430Adc12ImplP__startTimerA();
            }
#line 408
          {
            unsigned char __nesc_temp = 
#line 408
            SUCCESS;

            {
#line 408
              __nesc_atomic_end(__nesc_atomic); 
#line 408
              return __nesc_temp;
            }
          }
        }
    }
#line 412
    __nesc_atomic_end(__nesc_atomic); }
#line 411
  return FAIL;
}

# 137 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId(void )
#line 137
{
  /* atomic removed: atomic calls only */
#line 138
  {
    if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state != /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 140
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;

#line 140
        return __nesc_temp;
      }
#line 141
    {
      unsigned char __nesc_temp = 
#line 141
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId;

#line 141
      return __nesc_temp;
    }
  }
}

# 80 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(int mode)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

# 96 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type now = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__get();
#line 98
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type expires;
#line 98
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 = now + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt = remaining - /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 += /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type )now << 5, 
  (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type )remaining << 5);
}

# 14 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(24)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 52 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 164 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )49U |= 0x01 << 4;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )49U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )49U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 123 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 164
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 64 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x2ba081571d50){
#line 64
  switch (arg_0x2ba081571d50) {
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 64
      break;
#line 64
    case Stm25pSectorP__signalDone_task:
#line 64
      Stm25pSectorP__signalDone_task__runTask();
#line 64
      break;
#line 64
    case /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 64
      /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 64
      break;
#line 64
    case /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask:
#line 64
      /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__startTask__runTask();
#line 64
      break;
#line 64
    case /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask:
#line 64
      /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__timerTask__runTask();
#line 64
      break;
#line 64
    case /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task:
#line 64
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask();
#line 64
      break;
#line 64
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask:
#line 64
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask();
#line 64
      break;
#line 64
    case AdcP__readDone:
#line 64
      AdcP__readDone__runTask();
#line 64
      break;
#line 64
    case /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask:
#line 64
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask();
#line 64
      break;
#line 64
    case Msp430RefVoltArbiterImplP__switchOff:
#line 64
      Msp430RefVoltArbiterImplP__switchOff__runTask();
#line 64
      break;
#line 64
    case AdcStreamP__readStreamDone:
#line 64
      AdcStreamP__readStreamDone__runTask();
#line 64
      break;
#line 64
    case AdcStreamP__readStreamFail:
#line 64
      AdcStreamP__readStreamFail__runTask();
#line 64
      break;
#line 64
    case AdcStreamP__bufferDone:
#line 64
      AdcStreamP__bufferDone__runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x2ba081571d50);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 47 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 6;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static error_t Stm25pBlockP__Write__write(uint8_t id, storage_addr_t addr, void *buf, 
storage_len_t len)
#line 106
{
  Stm25pBlockP__m_req.req = Stm25pBlockP__S_WRITE;
  Stm25pBlockP__m_req.addr = addr;
  Stm25pBlockP__m_req.buf = buf;
  Stm25pBlockP__m_req.len = len;
  return Stm25pBlockP__newRequest(id);
}











static error_t Stm25pBlockP__newRequest(uint8_t client)
#line 124
{

  if (Stm25pBlockP__m_block_state[client].req != Stm25pBlockP__S_IDLE) {
    return FAIL;
    }
  Stm25pBlockP__ClientResource__request(client);
  Stm25pBlockP__m_block_state[client] = Stm25pBlockP__m_req;

  return SUCCESS;
}

# 77 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id)
#line 77
{
  /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      if (/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state == /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
          /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 84
          /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(id);

          {
#line 84
            __nesc_atomic_end(__nesc_atomic); 
#line 84
            return __nesc_temp;
          }
        }
    }
#line 87
    __nesc_atomic_end(__nesc_atomic); }
#line 86
  /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 116 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static error_t Msp430RefVoltArbiterImplP__ClientResource__release(uint8_t client)
{
  error_t error;

#line 119
  if (Msp430RefVoltArbiterImplP__syncOwner == client) {
    Msp430RefVoltArbiterImplP__switchOff__postTask();
    }
#line 121
  error = Msp430RefVoltArbiterImplP__AdcResource__release(client);
#line 133
  return error;
}

# 97 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id)
#line 97
{
  bool released = FALSE;

#line 99
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_BUSY && /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId == id) {
          if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__dequeue();
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
            }
          else {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_IDLE;
            }
          released = TRUE;
        }
    }
#line 113
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 65 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/RoundRobinResourceQueueC.nc"
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(resource_client_id_t id)
#line 65
{
  return /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[id / 8] & (1 << id % 8);
}

# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )49U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 30 "SummarizerC.nc"
static void SummarizerC__nextSummarySample(void )
#line 30
{

  SummarizerC__BlockRead__read(SummarizerC__index * DFACTOR * sizeof(uint16_t ), SummarizerC__samples, sizeof SummarizerC__samples);
}

# 236 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP__switchOff(void )
#line 236
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 237
    {
      if (Msp430RefVoltGeneratorP__HplAdc12__isBusy()) {
          {
            unsigned char __nesc_temp = 
#line 239
            EBUSY;

            {
#line 239
              __nesc_atomic_end(__nesc_atomic); 
#line 239
              return __nesc_temp;
            }
          }
        }
      else 
#line 241
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP__HplAdc12__getCtl0();

#line 243
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          ctl0.refon = 0;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 247
            SUCCESS;

            {
#line 247
              __nesc_atomic_end(__nesc_atomic); 
#line 247
              return __nesc_temp;
            }
          }
        }
    }
#line 251
    __nesc_atomic_end(__nesc_atomic); }
}

# 133 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 153 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static void Msp430RefVoltGeneratorP__signalStartDone(Msp430RefVoltGeneratorP__state_t state, error_t result)
#line 153
{
  if (state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING) {
    Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(result);
    }
  else {
#line 157
    Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(result);
    }
}

# 98 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static void AdcP__SubResourceReadNow__granted(uint8_t nowClient)
{
  if (AdcP__configure(nowClient) == SUCCESS) {
    AdcP__state = AdcP__STATE_READNOW;
    }
  else {
#line 103
    AdcP__state = AdcP__STATE_READNOW_INVALID_CONFIG;
    }
#line 104
  AdcP__ResourceReadNow__granted(nowClient);
}

#line 65
static error_t AdcP__configure(uint8_t client)
{
  error_t result = EINVAL;
  const msp430adc12_channel_config_t * config;

#line 69
  config = AdcP__Config__getConfiguration(client);
  if (config->inch != INPUT_CHANNEL_NONE) {
    result = AdcP__SingleChannel__configureSingle(client, config);
    }
#line 72
  return result;
}

# 176 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static error_t Msp430Adc12ImplP__SingleChannel__configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config)
{
  error_t result = ERESERVE;

  if (!config) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 184
    {
      if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 186
          EBUSY;

          {
#line 186
            __nesc_atomic_end(__nesc_atomic); 
#line 186
            return __nesc_temp;
          }
        }
#line 187
      if (Msp430Adc12ImplP__ADCArbiterInfo__userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = 0, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = 0, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 1 };

          adc12ctl0_t ctl0 = Msp430Adc12ImplP__HplAdc12__getCtl0();

#line 204
          ctl0.msc = 1;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP__state = Msp430Adc12ImplP__SINGLE_DATA;
          Msp430Adc12ImplP__HplAdc12__setCtl0(ctl0);
          Msp430Adc12ImplP__HplAdc12__setCtl1(ctl1);
          Msp430Adc12ImplP__HplAdc12__setMCtl(0, memctl);
          Msp430Adc12ImplP__HplAdc12__setIEFlags(0x01);
          result = SUCCESS;
        }
    }
#line 215
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 221 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP__ReadStream__read(uint8_t c, uint32_t usPeriod)
{
  if (usPeriod & 0xFFFF0000) {

      AdcStreamP__period = usPeriod / 1000;
      AdcStreamP__periodModified = TRUE;
      AdcStreamP__client = c;
      AdcStreamP__now = AdcStreamP__Alarm__getNow();
      AdcStreamP__SingleChannel__configureSingle(c, AdcStreamP__AdcConfigure__getConfiguration(c));
      if (AdcStreamP__nextBuffer(FALSE) == SUCCESS) {
        AdcStreamP__sampleSingle();
        }
    }
  else 
#line 232
    {
      AdcStreamP__period = usPeriod;
      AdcStreamP__periodModified = FALSE;
      AdcStreamP__client = c;
      AdcStreamP__nextMultiple(c);
    }
  return SUCCESS;
}

#line 177
static error_t AdcStreamP__nextBuffer(bool startNextAlarm)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      struct AdcStreamP__list_entry_t *entry = AdcStreamP__bufferQueue[AdcStreamP__client];

      if (!entry) 
        {

          AdcStreamP__bufferQueueEnd[AdcStreamP__client] = (void *)0;
          AdcStreamP__readStreamDone__postTask();
          {
            unsigned char __nesc_temp = 
#line 187
            FAIL;

            {
#line 187
              __nesc_atomic_end(__nesc_atomic); 
#line 187
              return __nesc_temp;
            }
          }
        }
      else 
#line 190
        {
          uint16_t tmp_count;

#line 192
          AdcStreamP__bufferQueue[AdcStreamP__client] = entry->next;
          if (!AdcStreamP__bufferQueue[AdcStreamP__client]) {
            AdcStreamP__bufferQueueEnd[AdcStreamP__client] = &AdcStreamP__bufferQueue[AdcStreamP__client];
            }
#line 195
          AdcStreamP__pos = AdcStreamP__buffer = (void *)0;
          AdcStreamP__count = entry->count;
          tmp_count = AdcStreamP__count;
          AdcStreamP__pos = AdcStreamP__buffer = (uint16_t * )entry;
          if (startNextAlarm) {
            AdcStreamP__nextAlarm();
            }
#line 201
          {
            unsigned char __nesc_temp = 
#line 201
            SUCCESS;

            {
#line 201
              __nesc_atomic_end(__nesc_atomic); 
#line 201
              return __nesc_temp;
            }
          }
        }
    }
#line 205
    __nesc_atomic_end(__nesc_atomic); }
}

#line 206
static void AdcStreamP__nextMultiple(uint8_t c)
{
  if (AdcStreamP__nextBuffer(FALSE) == SUCCESS) {
      msp430adc12_channel_config_t config = *AdcStreamP__AdcConfigure__getConfiguration(c);

#line 210
      config.sampcon_ssel = SAMPCON_SOURCE_SMCLK;
      config.sampcon_id = SAMPCON_CLOCK_DIV_1;
      if (AdcStreamP__SingleChannel__configureMultiple(c, &config, AdcStreamP__pos, AdcStreamP__count, AdcStreamP__period) == SUCCESS) {
        AdcStreamP__SingleChannel__getData(c);
        }
      else 
#line 214
        {
          AdcStreamP__postBuffer(c, AdcStreamP__pos, AdcStreamP__count);
          AdcStreamP__readStreamFail__postTask();
        }
    }
}

#line 96
static error_t AdcStreamP__postBuffer(uint8_t c, uint16_t *buf, uint16_t n)
{
  if (n < sizeof(struct AdcStreamP__list_entry_t )) {
    return ESIZE;
    }
#line 100
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      struct AdcStreamP__list_entry_t * newEntry = (struct AdcStreamP__list_entry_t * )buf;

      if (!AdcStreamP__bufferQueueEnd[c]) 
        {
          unsigned char __nesc_temp = 
#line 105
          FAIL;

          {
#line 105
            __nesc_atomic_end(__nesc_atomic); 
#line 105
            return __nesc_temp;
          }
        }
#line 107
      newEntry->count = n;
      newEntry->next = (void *)0;
      *AdcStreamP__bufferQueueEnd[c] = newEntry;
      AdcStreamP__bufferQueueEnd[c] = & newEntry->next;
    }
#line 111
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 80 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static void AdcP__ResourceRead__granted(uint8_t client)
{

  error_t result = AdcP__configure(client);

#line 84
  if (result == SUCCESS) {
      AdcP__state = AdcP__STATE_READ;
      result = AdcP__SingleChannel__getData(client);
    }
  else 
#line 87
    {
      AdcP__ResourceRead__release(client);
      AdcP__Read__readDone(client, result, 0);
    }
}

# 107 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {


      Msp430RefVoltArbiterImplP__ClientResource__granted(Msp430RefVoltArbiterImplP__syncOwner);
    }
}

# 160 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static void Msp430RefVoltGeneratorP__signalStopDone(Msp430RefVoltGeneratorP__state_t state, error_t result)
#line 160
{
  if (state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING) {
    Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(result);
    }
  else {
#line 164
    Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(result);
    }
}

# 70 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static void Msp430RefVoltArbiterImplP__AdcResource__granted(uint8_t client)
{
  const msp430adc12_channel_config_t *settings = Msp430RefVoltArbiterImplP__Config__getConfiguration(client);

#line 73
  if (settings->sref == REFERENCE_VREFplus_AVss || 
  settings->sref == REFERENCE_VREFplus_VREFnegterm) {
      error_t started;

#line 76
      if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {



          Msp430RefVoltArbiterImplP__AdcResource__release(client);
          Msp430RefVoltArbiterImplP__AdcResource__request(client);
          return;
        }
      Msp430RefVoltArbiterImplP__syncOwner = client;
      if (settings->ref2_5v == REFVOLT_LEVEL_1_5) {
        started = Msp430RefVoltArbiterImplP__RefVolt_1_5V__start();
        }
      else {
#line 88
        started = Msp430RefVoltArbiterImplP__RefVolt_2_5V__start();
        }
#line 89
      if (started != SUCCESS) {
          Msp430RefVoltArbiterImplP__syncOwner = Msp430RefVoltArbiterImplP__NO_OWNER;
          Msp430RefVoltArbiterImplP__AdcResource__release(client);
          Msp430RefVoltArbiterImplP__AdcResource__request(client);
        }
    }
  else {
#line 95
    Msp430RefVoltArbiterImplP__ClientResource__granted(client);
    }
}

# 71 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/SimpleArbiterP.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id)
#line 71
{
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_IDLE) {
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId = id;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
          {
            unsigned char __nesc_temp = 
#line 78
            SUCCESS;

            {
#line 78
              __nesc_atomic_end(__nesc_atomic); 
#line 78
              return __nesc_temp;
            }
          }
        }
#line 80
      {
        unsigned char __nesc_temp = 
#line 80
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__enqueue(id);

        {
#line 80
          __nesc_atomic_end(__nesc_atomic); 
#line 80
          return __nesc_temp;
        }
      }
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
}

# 94 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__state_t targetState)
#line 94
{
  error_t result;

  if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE) {
      if (targetState == Msp430RefVoltGeneratorP__m_state) {
          result = EALREADY;
        }
      else {
#line 100
        if ((result = Msp430RefVoltGeneratorP__switchOn(targetState)) == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = targetState;
            Msp430RefVoltGeneratorP__signalStartDone(targetState, SUCCESS);
          }
        }
    }
  else {
#line 104
    if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__GENERATOR_OFF) {
        if ((result = Msp430RefVoltGeneratorP__switchOn(targetState)) == SUCCESS) {
            Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(17);
            Msp430RefVoltGeneratorP__m_state = targetState + 2;
          }
      }
    else {
#line 109
      if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_OFF_PENDING) {
          if ((result = Msp430RefVoltGeneratorP__switchOn(targetState)) == SUCCESS) {

              Msp430RefVoltGeneratorP__state_t oldState = Msp430RefVoltGeneratorP__m_state;

#line 113
              Msp430RefVoltGeneratorP__SwitchOffTimer__stop();
              Msp430RefVoltGeneratorP__m_state = targetState;
              Msp430RefVoltGeneratorP__signalStopDone(oldState, FAIL);
              Msp430RefVoltGeneratorP__signalStartDone(targetState, SUCCESS);
            }
        }
      else {
#line 118
        if (Msp430RefVoltGeneratorP__m_state == targetState + 2) {
          result = SUCCESS;
          }
        else {
#line 121
          result = EBUSY;
          }
        }
      }
    }
#line 123
  return result;
}

#line 217
static error_t Msp430RefVoltGeneratorP__switchOn(uint8_t level)
#line 217
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 218
    {
      if (Msp430RefVoltGeneratorP__HplAdc12__isBusy()) {
          {
            unsigned char __nesc_temp = 
#line 220
            EBUSY;

            {
#line 220
              __nesc_atomic_end(__nesc_atomic); 
#line 220
              return __nesc_temp;
            }
          }
        }
      else 
#line 222
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP__HplAdc12__getCtl0();

#line 224
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          ctl0.refon = 1;


          ctl0.r2_5v = level - 1;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 231
            SUCCESS;

            {
#line 231
              __nesc_atomic_end(__nesc_atomic); 
#line 231
              return __nesc_temp;
            }
          }
        }
    }
#line 235
    __nesc_atomic_end(__nesc_atomic); }
}

# 86 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static uint8_t Stm25pSpiP__sendCmd(uint8_t cmd, uint8_t len)
#line 86
{

  uint8_t tmp = 0;
  int i;

  Stm25pSpiP__CSN__clr();
  for (i = 0; i < len; i++) 
    tmp = Stm25pSpiP__SpiByte__write(cmd);
  Stm25pSpiP__CSN__set();

  return tmp;
}

# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 4);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 386 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static uint8_t HplMsp430Usart0P__Usart__rx(void )
#line 386
{
  uint8_t value;

#line 388
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 388
    value = U0RXBUF;
#line 388
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 45 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P44*/HplMsp430GeneralIOP__28__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 4;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 108 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id)
#line 108
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty() == FALSE) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue();
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 124
            SUCCESS;

            {
#line 124
              __nesc_atomic_end(__nesc_atomic); 
#line 124
              return __nesc_temp;
            }
          }
        }
    }
#line 128
    __nesc_atomic_end(__nesc_atomic); }
#line 127
  return FAIL;
}

# 247 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__disableSpi(void )
#line 247
{
  /* atomic removed: atomic calls only */
#line 248
  {
    HplMsp430Usart0P__ME1 &= ~(1 << 6);
    HplMsp430Usart0P__SIMO__selectIOFunc();
    HplMsp430Usart0P__SOMI__selectIOFunc();
    HplMsp430Usart0P__UCLK__selectIOFunc();
  }
}

# 130 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static error_t /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 130
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    {
      if (/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
          if (/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state == /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
              /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 135
                SUCCESS;

                {
#line 135
                  __nesc_atomic_end(__nesc_atomic); 
#line 135
                  return __nesc_temp;
                }
              }
            }
          else {
#line 137
            if (/*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state == /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
                /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
                /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__state = /*Stm25pSectorC.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 140
                  SUCCESS;

                  {
#line 140
                    __nesc_atomic_end(__nesc_atomic); 
#line 140
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 144
  return FAIL;
}

# 120 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/power/DeferredPowerManagerP.nc"
static void /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__stopDone(error_t error)
#line 120
{
  if (/*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__requested == TRUE) {
      /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__StdControl__start();
      /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__SplitControl__start();
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 125
    {
      /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__requested = FALSE;
      /*Stm25pSectorC.PowerManagerC.PowerManager*/DeferredPowerManagerP__0__stopping = FALSE;
    }
#line 128
    __nesc_atomic_end(__nesc_atomic); }
}

# 88 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static error_t Stm25pSectorP__SplitControl__start(void )
#line 88
{
  error_t error = Stm25pSectorP__SpiResource__request();

#line 90
  if (error == SUCCESS) {
    Stm25pSectorP__m_power_state = Stm25pSectorP__S_START;
    }
#line 92
  return error;
}

# 77 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 84
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(id);

          {
#line 84
            __nesc_atomic_end(__nesc_atomic); 
#line 84
            return __nesc_temp;
          }
        }
    }
#line 87
    __nesc_atomic_end(__nesc_atomic); }
#line 86
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 136 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static void Stm25pBlockP__ClientResource__granted(uint8_t id)
#line 136
{

  switch (Stm25pBlockP__m_block_state[id].req) {
      case Stm25pBlockP__S_READ: 
        Stm25pBlockP__Sector__read(id, Stm25pBlockP__m_block_state[id].addr, 
        Stm25pBlockP__m_block_state[id].buf, 
        Stm25pBlockP__m_block_state[id].len);
      break;
      case Stm25pBlockP__S_CRC: 
        Stm25pBlockP__Sector__computeCrc(id, (uint16_t )Stm25pBlockP__m_block_state[id].buf, 
        Stm25pBlockP__m_block_state[id].addr, 
        Stm25pBlockP__m_block_state[id].len);
      break;
      case Stm25pBlockP__S_WRITE: 
        Stm25pBlockP__Sector__write(id, Stm25pBlockP__m_block_state[id].addr, 
        Stm25pBlockP__m_block_state[id].buf, 
        Stm25pBlockP__m_block_state[id].len);
      break;
      case Stm25pBlockP__S_ERASE: 
        Stm25pBlockP__Sector__erase(id, 0, Stm25pBlockP__Sector__getNumSectors(id));
      break;
      case Stm25pBlockP__S_SYNC: 
        Stm25pBlockP__signalDone(id, 0, SUCCESS);
      break;
      case Stm25pBlockP__S_IDLE: 
        break;
    }
}

# 171 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static error_t Stm25pSectorP__Sector__read(uint8_t id, stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len)
#line 172
{

  Stm25pSectorP__m_state = Stm25pSectorP__S_READ;
  Stm25pSectorP__m_addr = addr;
  Stm25pSectorP__m_buf = buf;
  Stm25pSectorP__m_len = len;

  return Stm25pSectorP__Spi__read(Stm25pSectorP__physicalAddr(id, addr), buf, len);
}

# 138 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static error_t Stm25pSpiP__Spi__read(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len)
#line 139
{
  Stm25pSpiP__m_cmd[0] = Stm25pSpiP__S_READ;
  Stm25pSpiP__m_addr = addr;
  Stm25pSpiP__m_buf = buf;
  Stm25pSpiP__m_len = len;
  return Stm25pSpiP__newRequest(FALSE, 4);
}

#line 176
static error_t Stm25pSpiP__newRequest(bool write, stm25p_len_t cmd_len)
#line 176
{
  Stm25pSpiP__m_cmd[1] = Stm25pSpiP__m_addr >> 16;
  Stm25pSpiP__m_cmd[2] = Stm25pSpiP__m_addr >> 8;
  Stm25pSpiP__m_cmd[3] = Stm25pSpiP__m_addr;
  if (write) {
    Stm25pSpiP__sendCmd(Stm25pSpiP__S_WRITE_ENABLE, 1);
    }
#line 182
  Stm25pSpiP__CSN__clr();
  Stm25pSpiP__SpiPacket__send(Stm25pSpiP__m_cmd, (void *)0, cmd_len);
  return SUCCESS;
}

# 144 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 146
{

  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client = id;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf = tx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf = rx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len = len;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos = 0;

  if (len) {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

#line 121
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void )
#line 121
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 126
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);

      end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos + /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE;
      if (end > /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
        end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
        }
      while (++/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < end) {
          while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
          tmp = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();
          if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
            /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = tmp;
            }
#line 138
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);
        }
    }
#line 140
    __nesc_atomic_end(__nesc_atomic); }
}

# 188 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static error_t Stm25pSectorP__Sector__write(uint8_t id, stm25p_addr_t addr, 
uint8_t *buf, 
stm25p_len_t len)
#line 190
{

  Stm25pSectorP__m_state = Stm25pSectorP__S_WRITE;
  Stm25pSectorP__m_addr = addr;
  Stm25pSectorP__m_buf = buf;
  Stm25pSectorP__m_len = Stm25pSectorP__m_cur_len = len;

  return Stm25pSectorP__Spi__pageProgram(Stm25pSectorP__physicalAddr(id, addr), buf, 
  Stm25pSectorP__calcWriteLen(addr));
}

# 156 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static error_t Stm25pSpiP__Spi__pageProgram(stm25p_addr_t addr, uint8_t *buf, 
stm25p_len_t len)
#line 157
{
  Stm25pSpiP__m_cmd[0] = Stm25pSpiP__S_PAGE_PROGRAM;
  Stm25pSpiP__m_addr = addr;
  Stm25pSpiP__m_buf = buf;
  Stm25pSpiP__m_len = len;
  return Stm25pSpiP__newRequest(TRUE, 4);
}

# 158 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static stm25p_len_t Stm25pSectorP__calcWriteLen(stm25p_addr_t addr)
#line 158
{
  stm25p_len_t len = STM25P_PAGE_SIZE - (addr & STM25P_PAGE_MASK);

#line 160
  return Stm25pSectorP__m_cur_len < len ? Stm25pSectorP__m_cur_len : len;
}

#line 213
static error_t Stm25pSectorP__Sector__erase(uint8_t id, uint8_t sector, 
uint8_t num_sectors)
#line 214
{

  Stm25pSectorP__m_state = Stm25pSectorP__S_ERASE;
  Stm25pSectorP__m_addr = sector;
  Stm25pSectorP__m_len = num_sectors;
  Stm25pSectorP__m_cur_len = 0;

  return Stm25pSectorP__Spi__sectorErase(STM25P_VMAP[Stm25pSectorP__getVolumeId(id)].base + Stm25pSectorP__m_addr + 
  Stm25pSectorP__m_cur_len);
}

# 165 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static error_t Stm25pSpiP__Spi__sectorErase(uint8_t sector)
#line 165
{
  Stm25pSpiP__m_cmd[0] = Stm25pSpiP__S_SECTOR_ERASE;
  Stm25pSpiP__m_addr = (stm25p_addr_t )sector << STM25P_SECTOR_SIZE_LOG2;
  return Stm25pSpiP__newRequest(TRUE, 4);
}

# 167 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static uint8_t Stm25pSectorP__Sector__getNumSectors(uint8_t id)
#line 167
{
  return STM25P_VMAP[Stm25pSectorP__getVolumeId(id)].size;
}

# 189 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pBlockP.nc"
static void Stm25pBlockP__signalDone(uint8_t id, uint16_t crc, error_t error)
#line 189
{

  Stm25pBlockP__stm25p_block_req_t req = Stm25pBlockP__m_block_state[id].req;

  Stm25pBlockP__ClientResource__release(id);
  Stm25pBlockP__m_block_state[id].req = Stm25pBlockP__S_IDLE;
  switch (req) {
      case Stm25pBlockP__S_READ: 
        Stm25pBlockP__Read__readDone(id, Stm25pBlockP__m_block_state[id].addr, 
        Stm25pBlockP__m_block_state[id].buf, 
        Stm25pBlockP__m_block_state[id].len, error);
      break;
      case Stm25pBlockP__S_CRC: 
        Stm25pBlockP__Read__computeCrcDone(id, Stm25pBlockP__m_block_state[id].addr, 
        Stm25pBlockP__m_block_state[id].len, crc, error);
      break;
      case Stm25pBlockP__S_WRITE: 
        Stm25pBlockP__Write__writeDone(id, Stm25pBlockP__m_block_state[id].addr, 
        Stm25pBlockP__m_block_state[id].buf, 
        Stm25pBlockP__m_block_state[id].len, error);
      break;
      case Stm25pBlockP__S_SYNC: 
        Stm25pBlockP__Write__syncDone(id, error);
      break;
      case Stm25pBlockP__S_ERASE: 
        Stm25pBlockP__Write__eraseDone(id, error);
      break;
      case Stm25pBlockP__S_IDLE: 
        break;
    }
}

# 110 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSectorP.nc"
static error_t Stm25pSectorP__ClientResource__release(uint8_t id)
#line 110
{
  if (Stm25pSectorP__m_client == id) {
      Stm25pSectorP__m_state = Stm25pSectorP__S_IDLE;
      Stm25pSectorP__m_client = Stm25pSectorP__NO_CLIENT;
      Stm25pSectorP__SpiResource__release();
      Stm25pSectorP__Stm25pResource__release(id);
      return SUCCESS;
    }
  return FAIL;
}

# 214 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pLogP.nc"
static void Stm25pLogP__ClientResource__granted(uint8_t id)
#line 214
{


  if (Stm25pLogP__m_log_info[id].read_addr == STM25P_INVALID_ADDRESS && 
  Stm25pLogP__m_log_state[id].req != Stm25pLogP__S_ERASE) {
      Stm25pLogP__m_rw_state = Stm25pLogP__S_SEARCH_BLOCKS;
      Stm25pLogP__Sector__read(id, 0, (uint8_t *)&Stm25pLogP__m_addr, sizeof Stm25pLogP__m_addr);
    }
  else 
    {
      switch (Stm25pLogP__m_log_state[id].req) {
          case Stm25pLogP__S_READ: 
            Stm25pLogP__m_rw_state = Stm25pLogP__m_log_info[id].remaining ? Stm25pLogP__S_DATA : Stm25pLogP__S_HEADER;
          Stm25pLogP__continueReadOp(id);
          break;
          case Stm25pLogP__S_SEEK: 
            {

              uint8_t numSectors = Stm25pLogP__Sector__getNumSectors(id);
              uint8_t readSector = 
              Stm25pLogP__m_log_state[id].cookie >> STM25P_SECTOR_SIZE_LOG2;
              uint8_t writeSector = ((
              Stm25pLogP__m_log_info[id].write_addr - 1) >> STM25P_SECTOR_SIZE_LOG2) + 1;

              if (writeSector - readSector > numSectors) {
                  Stm25pLogP__m_log_state[id].cookie = 
                  (storage_cookie_t )(writeSector - numSectors)
                   << STM25P_SECTOR_SIZE_LOG2;
                }
              Stm25pLogP__m_log_info[id].read_addr = Stm25pLogP__m_log_state[id].cookie & ~Stm25pLogP__BLOCK_MASK;
              Stm25pLogP__m_log_info[id].remaining = 0;
              Stm25pLogP__m_rw_state = Stm25pLogP__S_SEARCH_SEEK;
              if (Stm25pLogP__m_log_info[id].read_addr != Stm25pLogP__m_log_state[id].cookie) {
                  Stm25pLogP__m_log_info[id].read_addr += sizeof Stm25pLogP__m_addr;
                  Stm25pLogP__Sector__read(id, Stm25pLogP__calcAddr(id, Stm25pLogP__m_log_info[id].read_addr), 
                  &Stm25pLogP__m_header, sizeof Stm25pLogP__m_header);
                }
              else {
                Stm25pLogP__signalDone(id, SUCCESS);
                }
            }
#line 254
          break;
          case Stm25pLogP__S_ERASE: 
            Stm25pLogP__Sector__erase(id, 0, Stm25pLogP__Sector__getNumSectors(id));
          break;
          case Stm25pLogP__S_APPEND: 
            Stm25pLogP__m_rw_state = Stm25pLogP__S_HEADER;
          Stm25pLogP__continueAppendOp(id);
          break;
          case Stm25pLogP__S_SYNC: 
            Stm25pLogP__signalDone(id, SUCCESS);
          break;
          case Stm25pLogP__S_IDLE: 
            break;
        }
    }
}


static void Stm25pLogP__continueReadOp(uint8_t client)
#line 272
{

  stm25p_addr_t read_addr = Stm25pLogP__m_log_info[client].read_addr;
  uint8_t *buf;
  uint8_t len;


  if (Stm25pLogP__m_len == 0 || read_addr >= Stm25pLogP__m_log_info[client].write_addr) {
      Stm25pLogP__signalDone(client, SUCCESS);
      return;
    }

  buf = &Stm25pLogP__m_header;
  len = sizeof Stm25pLogP__m_header;

  if (Stm25pLogP__m_rw_state == Stm25pLogP__S_DATA) {

      if (Stm25pLogP__m_header == Stm25pLogP__INVALID_HEADER) {
          Stm25pLogP__m_rw_state = Stm25pLogP__S_HEADER;
          read_addr += Stm25pLogP__BLOCK_SIZE;
          read_addr &= ~Stm25pLogP__BLOCK_MASK;
        }
      else {
          buf = Stm25pLogP__m_log_state[client].buf + Stm25pLogP__m_log_state[client].len - Stm25pLogP__m_len;

          if (Stm25pLogP__m_log_info[client].remaining < Stm25pLogP__m_len) {
            len = Stm25pLogP__m_log_info[client].remaining;
            }
          else {
#line 300
            len = Stm25pLogP__m_len;
            }
        }
    }

  if (!((uint16_t )read_addr & Stm25pLogP__BLOCK_MASK)) {
    read_addr += sizeof Stm25pLogP__m_addr;
    }
  Stm25pLogP__m_log_info[client].read_addr = read_addr;
  Stm25pLogP__Sector__read(client, Stm25pLogP__calcAddr(client, read_addr), buf, len);
}

#line 485
static void Stm25pLogP__signalDone(uint8_t id, error_t error)
#line 485
{

  Stm25pLogP__stm25p_log_req_t req = Stm25pLogP__m_log_state[id].req;
  void *buf = Stm25pLogP__m_log_state[id].buf;
  storage_len_t len = Stm25pLogP__m_log_state[id].len;

  Stm25pLogP__ClientResource__release(id);
  Stm25pLogP__m_log_state[id].req = Stm25pLogP__S_IDLE;
  switch (req) {
      case Stm25pLogP__S_IDLE: 
        break;
      case Stm25pLogP__S_READ: 
        Stm25pLogP__Read__readDone(id, buf, len - Stm25pLogP__m_len, error);
      break;
      case Stm25pLogP__S_SEEK: 
        Stm25pLogP__Read__seekDone(id, error);
      break;
      case Stm25pLogP__S_ERASE: 
        Stm25pLogP__Write__eraseDone(id, error);
      break;
      case Stm25pLogP__S_APPEND: 
        Stm25pLogP__Write__appendDone(id, buf, len, Stm25pLogP__m_records_lost, error);
      break;
      case Stm25pLogP__S_SYNC: 
        Stm25pLogP__Write__syncDone(id, error);
      break;
    }
}

#line 207
static stm25p_addr_t Stm25pLogP__calcAddr(uint8_t client, stm25p_addr_t addr)
#line 207
{
  stm25p_addr_t result = Stm25pLogP__calcSector(client, addr);

#line 209
  result <<= STM25P_SECTOR_SIZE_LOG2;
  result |= addr & STM25P_SECTOR_MASK;
  return result;
}

#line 418
static void Stm25pLogP__continueAppendOp(uint8_t client)
#line 418
{

  stm25p_addr_t write_addr = Stm25pLogP__m_log_info[client].write_addr;
  void *buf;
  uint8_t len;

  if (! (uint16_t )write_addr) {
      Stm25pLogP__m_records_lost = TRUE;
      Stm25pLogP__Sector__erase(client, Stm25pLogP__calcSector(client, write_addr), 1);
    }
  else {
      if (!((uint16_t )write_addr & Stm25pLogP__BLOCK_MASK)) {
          buf = & Stm25pLogP__m_log_info[client].write_addr;
          len = sizeof Stm25pLogP__m_addr;
        }
      else {
#line 433
        if (Stm25pLogP__m_rw_state == Stm25pLogP__S_HEADER) {
            buf = & Stm25pLogP__m_log_state[client].len;
            len = sizeof  Stm25pLogP__m_log_state[client].len;
          }
        else {
            buf = Stm25pLogP__m_log_state[client].buf;
            len = Stm25pLogP__m_log_state[client].len;
          }
        }
#line 441
      Stm25pLogP__Sector__write(client, Stm25pLogP__calcAddr(client, write_addr), buf, len);
    }
}

# 187 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/stm25p/Stm25pSpiP.nc"
static void Stm25pSpiP__releaseAndRequest(void )
#line 187
{
  Stm25pSpiP__SpiResource__release();
  Stm25pSpiP__SpiResource__request();
}

#line 249
static void Stm25pSpiP__signalDone(error_t error)
#line 249
{
  Stm25pSpiP__m_is_writing = FALSE;
  switch (Stm25pSpiP__m_cmd[0]) {
      case Stm25pSpiP__S_READ: 
        if (Stm25pSpiP__m_computing_crc) {
            Stm25pSpiP__m_computing_crc = FALSE;
            Stm25pSpiP__Spi__computeCrcDone(Stm25pSpiP__m_crc, Stm25pSpiP__m_addr, Stm25pSpiP__m_len, error);
          }
        else {
            Stm25pSpiP__Spi__readDone(Stm25pSpiP__m_addr, Stm25pSpiP__m_buf, Stm25pSpiP__m_len, error);
          }
      break;
      case Stm25pSpiP__S_PAGE_PROGRAM: 
        Stm25pSpiP__Spi__pageProgramDone(Stm25pSpiP__m_addr, Stm25pSpiP__m_buf, Stm25pSpiP__m_len, error);
      break;
      case Stm25pSpiP__S_SECTOR_ERASE: 
        Stm25pSpiP__Spi__sectorEraseDone(Stm25pSpiP__m_addr >> STM25P_SECTOR_SIZE_LOG2, error);
      break;
      case Stm25pSpiP__S_BULK_ERASE: 
        Stm25pSpiP__Spi__bulkEraseDone(error);
      break;
    }
}

#line 192
static void Stm25pSpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 193
{

  int i;

  switch (Stm25pSpiP__m_cmd[0]) {

      case Stm25pSpiP__S_READ: 
        if (tx_buf == Stm25pSpiP__m_cmd) {
            Stm25pSpiP__SpiPacket__send((void *)0, Stm25pSpiP__m_buf, Stm25pSpiP__m_len);
            break;
          }
        else {
#line 204
          if (Stm25pSpiP__m_computing_crc) {
              for (i = 0; i < len; i++) 
                Stm25pSpiP__m_crc = crcByte(Stm25pSpiP__m_crc, Stm25pSpiP__m_crc_buf[i]);
              Stm25pSpiP__m_cur_addr += len;
              Stm25pSpiP__m_cur_len -= len;
              if (Stm25pSpiP__m_cur_len) {
                  Stm25pSpiP__SpiPacket__send((void *)0, Stm25pSpiP__m_crc_buf, Stm25pSpiP__calcReadLen());
                  break;
                }
            }
          }
#line 214
      Stm25pSpiP__CSN__set();
      Stm25pSpiP__signalDone(SUCCESS);
      break;

      case Stm25pSpiP__S_PAGE_PROGRAM: 
        if (tx_buf == Stm25pSpiP__m_cmd) {
            Stm25pSpiP__SpiPacket__send(Stm25pSpiP__m_buf, (void *)0, Stm25pSpiP__m_len);
            break;
          }


      case Stm25pSpiP__S_SECTOR_ERASE: case Stm25pSpiP__S_BULK_ERASE: 
          Stm25pSpiP__CSN__set();
      Stm25pSpiP__m_is_writing = TRUE;
      Stm25pSpiP__releaseAndRequest();
      break;

      default: 
        break;
    }
}

# 62 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 46 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )49U &= ~(0x01 << 4);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 136 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
static void HplAdc12P__HplAdc12__stopConversion(void )
#line 105
{

  uint16_t ctl1 = HplAdc12P__ADC12CTL1;

#line 108
  HplAdc12P__ADC12CTL1 &= ~(0x0002 | 0x0004);
  HplAdc12P__ADC12CTL0 &= ~(0x0001 + 0x0002);
  HplAdc12P__ADC12CTL0 &= ~0x0010;
  HplAdc12P__ADC12CTL1 |= ctl1 & (0x0002 | 0x0004);
}

# 96 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(18)))  void sig_UART0RX_VECTOR(void )
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 150 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void )
#line 150
{
  /* atomic removed: atomic calls only */
#line 151
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 153
        FALSE;

#line 153
        return __nesc_temp;
      }
  }
#line 155
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void )
#line 163
{
  /* atomic removed: atomic calls only */
#line 164
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 166
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;

#line 166
        return __nesc_temp;
      }
#line 167
    {
      unsigned char __nesc_temp = 
#line 167
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__1__resId;

#line 167
      return __nesc_temp;
    }
  }
}

# 101 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(16)))  void sig_UART0TX_VECTOR(void )
#line 101
{
  if (HplMsp430Usart0P__HplI2C__isI2C()) {
    HplMsp430Usart0P__I2CInterrupts__fired();
    }
  else {
#line 105
    HplMsp430Usart0P__Interrupts__txDone();
    }
}

# 120 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/HplAdc12P.nc"
__attribute((wakeup)) __attribute((interrupt(14)))  void sig_ADC_VECTOR(void )
#line 120
{
  HplAdc12P__HplAdc12__conversionDone(HplAdc12P__ADC12IV);
}

# 503 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static void Msp430Adc12ImplP__stopConversion(void )
{
  uint8_t i;

  if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__USE_TIMERA) {
    Msp430Adc12ImplP__TimerA__setMode(MSP430TIMER_STOP_MODE);
    }
  Msp430Adc12ImplP__resetAdcPin(Msp430Adc12ImplP__HplAdc12__getMCtl(0).inch);
  if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__MULTI_CHANNEL) {
      for (i = 1; i < Msp430Adc12ImplP__numChannels; i++) 
        Msp430Adc12ImplP__resetAdcPin(Msp430Adc12ImplP__HplAdc12__getMCtl(i).inch);
    }
  /* atomic removed: atomic calls only */
#line 515
  {
    Msp430Adc12ImplP__HplAdc12__stopConversion();
    Msp430Adc12ImplP__HplAdc12__resetIFGs();
    Msp430Adc12ImplP__state &= ~Msp430Adc12ImplP__ADC_BUSY;
  }
}

#line 159
static void Msp430Adc12ImplP__resetAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP__Port60__selectIOFunc();
#line 164
      break;
      case 1: Msp430Adc12ImplP__Port61__selectIOFunc();
#line 165
      break;
      case 2: Msp430Adc12ImplP__Port62__selectIOFunc();
#line 166
      break;
      case 3: Msp430Adc12ImplP__Port63__selectIOFunc();
#line 167
      break;
      case 4: Msp430Adc12ImplP__Port64__selectIOFunc();
#line 168
      break;
      case 5: Msp430Adc12ImplP__Port65__selectIOFunc();
#line 169
      break;
      case 6: Msp430Adc12ImplP__Port66__selectIOFunc();
#line 170
      break;
      case 7: Msp430Adc12ImplP__Port67__selectIOFunc();
#line 171
      break;
    }
}

# 142 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcP.nc"
static error_t AdcP__SingleChannel__singleDataReady(uint8_t client, uint16_t data)
{
  switch (AdcP__state) 
    {
      case AdcP__STATE_READ: 
        AdcP__owner = client;
      AdcP__value = data;
      AdcP__readDone__postTask();
      break;
      case AdcP__STATE_READNOW: 
        AdcP__ReadNow__readDone(client, SUCCESS, data);
      break;
      default: 

        break;
    }
  return SUCCESS;
}

# 242 "/home/ahmaurya/Dropbox/Courses/CS691/brimon/tinyos-2.1.1/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP__SingleChannel__singleDataReady(uint8_t streamClient, uint16_t data)
{
  if (AdcStreamP__client == AdcStreamP__NSTREAM) {
    return FAIL;
    }
  if (AdcStreamP__count == 0) 
    {
      AdcStreamP__now = AdcStreamP__Alarm__getNow();
      AdcStreamP__nextBuffer(TRUE);
    }
  else 
    {
      * AdcStreamP__pos++ = data;
      if (AdcStreamP__pos == AdcStreamP__buffer + AdcStreamP__count) 
        {
          /* atomic removed: atomic calls only */
          {
            if (AdcStreamP__lastBuffer) 
              {

                AdcStreamP__bufferQueueEnd[AdcStreamP__client] = (void *)0;
                AdcStreamP__readStreamFail__postTask();
                {
                  unsigned char __nesc_temp = 
#line 264
                  FAIL;

#line 264
                  return __nesc_temp;
                }
              }
            else {
                AdcStreamP__lastCount = AdcStreamP__count;
                AdcStreamP__lastBuffer = AdcStreamP__buffer;
              }
          }
          AdcStreamP__bufferDone__postTask();
          AdcStreamP__nextBuffer(TRUE);
        }
      else {
        AdcStreamP__nextAlarm();
        }
    }
#line 278
  return FAIL;
}

static uint16_t *AdcStreamP__SingleChannel__multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length)
{
  /* atomic removed: atomic calls only */
  {
    if (AdcStreamP__lastBuffer) 
      {

        AdcStreamP__bufferQueueEnd[AdcStreamP__client] = (void *)0;
        AdcStreamP__readStreamFail__postTask();
        {
          unsigned int *__nesc_temp = 
#line 291
          0;

#line 291
          return __nesc_temp;
        }
      }
    else {
        AdcStreamP__lastBuffer = AdcStreamP__buffer;
        AdcStreamP__lastCount = AdcStreamP__pos - AdcStreamP__buffer;
      }
  }
  AdcStreamP__bufferDone__postTask();
  AdcStreamP__nextMultiple(streamClient);
  return 0;
}

