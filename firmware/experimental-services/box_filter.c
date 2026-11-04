#include "box_filter.h"
#ifndef BOX_ROI_LEFT
#define BOX_ROI_LEFT 0
#endif
#ifndef BOX_ROI_RIGHT
#define BOX_ROI_RIGHT 1279
#endif
#ifndef BOX_HEIGHT
#define BOX_HEIGHT 720
#endif
static void withdraw(BoxState *s) {
 s->published_valid=0;s->seen=0;
 for(u32 k=0;k<4;k++)s->count[k]=0;
}
void box_reset(BoxState *s) {
 u32 *p=(u32*)s;for(u32 k=0;k<sizeof(*s)/4;k++)p[k]=0;
}
static u32 distance(i32 a) {return a<0?(u32)-a:(u32)a;}
static u32 median(u32 a,u32 b,u32 c) {
 if(a>b){u32 t=a;a=b;b=t;}if(b>c)b=c;return a>b?a:b;
}
static int coherent(i32 first,i32 second) {
 return first&&second&&((first<0)==(second<0))&&distance(first-second)<=1;
}
static int geometry(const u32 *b) {
 return b[0]>=BOX_ROI_LEFT&&b[1]<=BOX_ROI_RIGHT&&b[0]<=b[1]&&
 b[2]<=b[3]&&b[3]<BOX_HEIGHT&&!(b[0]&3)&&!(b[2]&3)&&
 (b[1]&3)==3&&(b[3]&3)==3;
}
void box_process(BoxState *s,const BoxInput *in) {
 u32 flags=in->flags,valid=(flags>>8)&15;
 s->commit=0;
 if(flags&BOX_RESET){box_reset(s);return;}
 if(!(flags&BOX_SOURCE_VALID)||(flags&(BOX_STALE|BOX_OVERFLOW))) {
  int notify=!s->aborted||(flags&BOX_WAS_BUSY)||s->published_valid;
  withdraw(s);s->aborted=1;
  if(notify){s->output_frame=in->frame;s->commit=1;}
  return;
 }
 s->aborted=0;
 if(!(flags&BOX_RESULT))return;
 if(flags&BOX_BUSY_VIOLATION) {
  withdraw(s);s->output_frame=in->frame;s->commit=1;return;
 }
 if(!valid) {
  withdraw(s);s->seen=1;s->last_frame=in->frame;
  s->output_frame=in->frame;s->commit=1;return;
 }
 u32 old_valid=s->published_valid;
 s->published_valid&=valid;
 for(u32 k=0;k<4;k++)if(!(valid&(1u<<k)))s->count[k]=0;
 if(s->seen&&in->frame==s->last_frame) {
  if(old_valid&~valid){s->output_frame=in->frame;s->commit=1;}
  return;
 }
 u32 gap=in->frame-s->last_frame;
 int gap_good=s->seen&&gap&&gap<=2;
 s->seen=1;s->last_frame=in->frame;
 u32 accepted_bits=0;
 for(u32 slot=0;slot<4;slot++) {
  u32 bit=1u<<slot,base=slot*4;
  if(!(valid&bit)){s->count[slot]=0;continue;}
  const u32 *box=in->xy+base;
  if(!geometry(box)){withdraw(s);s->output_frame=in->frame;s->commit=1;return;}
  u32 cur[4];i32 delta[4];int matched=s->count[slot]&&gap_good;
  for(u32 e=0;e<4;e++) {
   cur[e]=box[e]>>2;delta[e]=(i32)cur[e]-(i32)s->history1[base+e];
   if(distance(delta[e])>16)matched=0;
  }
  int three=s->count[slot]==2&&matched;
  int moving_x=coherent(delta[0],delta[1]),moving_y=coherent(delta[2],delta[3]);
  for(u32 e=0;e<4;e++) {
   u32 at=base+e;u32 chosen=cur[e];
   if(three&&!(e<2?moving_x:moving_y)) {
    u32 med=median(cur[e],s->history1[at],s->history2[at]);
    chosen=distance((i32)med-(i32)s->published[at])<=1?s->published[at]:med;
   }
   s->work[at]=chosen;s->history2[at]=s->history1[at];s->history1[at]=cur[e];
  }
  if(s->work[base]>s->work[base+1]||s->work[base+2]>s->work[base+3])
   for(u32 e=0;e<4;e++)s->work[base+e]=cur[e];
  s->count[slot]=matched?2:1;
  if(matched)accepted_bits|=bit;
 }
 for(u32 k=0;k<16;k++)s->published[k]=s->work[k];
 s->published_valid=accepted_bits;s->output_frame=in->frame;s->commit=1;
}
void box_output(const BoxState *s,BoxOutput *out) {
 out->frame=s->output_frame;out->valid=s->published_valid;out->commit=s->commit;
 for(u32 k=0;k<16;k++)out->xy[k]=(s->published[k]<<2)|((k&1)?3:0);
}
