#ifndef BOX_FILTER_H
#define BOX_FILTER_H
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;
typedef signed int i32;
enum { BOX_SOURCE_VALID=1,BOX_STALE=2,BOX_OVERFLOW=4,BOX_RESULT=8,
       BOX_BUSY_VIOLATION=16,BOX_RESET=32,BOX_WAS_BUSY=64 };
// Existing hard-MCU private RAM accepts aligned full32-bit writes only.
// Keep every persisted field a word; do not generate byte/halfword stores.
typedef struct {u32 frame,flags;u32 xy[16];} BoxInput;
typedef struct {
 u32 history1[16],history2[16],published[16],work[16];
 u32 last_frame,output_frame;
 u32 count[4],seen,aborted,published_valid,commit;
} BoxState;
typedef struct {u32 frame,valid,commit;u32 xy[16];} BoxOutput;
void box_reset(BoxState *s);
void box_process(BoxState *s,const BoxInput *in);
void box_output(const BoxState *s,BoxOutput *out);
#endif
