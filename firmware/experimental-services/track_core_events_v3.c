/* Single actual red candidate after the existing four-colour filter.
 * All persistent accesses remain aligned32-bit words; no SB/SH RAM writes.
 * This pure reducer has not yet been compared against a real RTL trace.
 */
#include "track_core_events_v3.h"
void track_reset(TrackState *s,tr_u32 zero_identity){
 volatile tr_u32 *p=(volatile tr_u32*)s;
 for(tr_u32 i=zero_identity?0:1;i<8;i++)p[i]=0;
}
void track_copy(TrackState *to,const TrackState *from){
 volatile tr_u32 *d=(volatile tr_u32*)to;
 const volatile tr_u32 *s=(const volatile tr_u32*)from;
 for(tr_u32 i=0;i<8;i++)d[i]=s[i];
}
static tr_u32 distance(tr_u32 a,tr_u32 b){return a>=b?a-b:b-a;}
tr_u32 track_step(TrackState *s,tr_u32 frame,tr_u32 valid,const tr_u32 *b){
 tr_u32 f=s->flags,hits=(f>>4)&3u,misses=(f>>6)&7u;
 tr_u32 force_missing=valid>>31;valid&=1u;
 tr_u32 count=0,x=0,y=0;
 if(valid&&b[0]<=b[1]&&b[1]<320&&b[2]<=b[3]&&b[3]<180){
  count=(b[1]-b[0]+1)*(b[3]-b[2]+1);
  x=((b[0]+b[1])<<1)+1;y=((b[2]+b[3])<<1)+1;
 }else valid=0;
 tr_u32 initially_valid=valid&&count>=16&&count<=57600;
 if(!force_missing&&(f&TR_HAVE_FRAME)&&frame==s->last_frame)return initially_valid;
 s->last_frame=frame;f|=TR_HAVE_FRAME;
 valid=initially_valid&&(!(f&TR_REFERENCE)||
  (distance(x,s->reference_x)+distance(y,s->reference_y)<=160&&
   count<=s->reference_count*4&&count*4>=s->reference_count));
 if(valid){
  s->center_x=(f&TR_REFERENCE)?(3*s->center_x+x+2)>>2:x;
  s->center_y=(f&TR_REFERENCE)?(3*s->center_y+y+2)>>2:y;
  s->reference_x=x;s->reference_y=y;s->reference_count=count;misses=0;
  if(hits>=2){
   if(!(f&TR_CONFIRMED))s->identity=s->identity==65535?1:s->identity+1;
   f|=TR_CONFIRMED|TR_VISIBLE;
  }else f&=~TR_VISIBLE;
  if(hits<3)hits++;f|=TR_REFERENCE;
 }else{
  f&=~TR_VISIBLE;
  if((f&TR_CONFIRMED)&&misses<3)misses++;
  else{f&=~(TR_REFERENCE|TR_CONFIRMED);hits=0;misses=0;s->center_x=0;s->center_y=0;}
 }
 s->flags=(f&15u)|(hits<<4)|(misses<<6);return initially_valid;
}
void track_pack(const TrackState *s,tr_u32 part[3]){
 tr_u32 id=(s->flags&TR_CONFIRMED)?s->identity:0;
 tr_u32 low=s->center_x|(s->center_y<<11)|(id<<21);
 tr_u32 high=(id>>11)|((!!(s->flags&TR_VISIBLE))<<5)|(((s->flags>>6)&3u)<<6);
 part[0]=low&16383;part[1]=(low>>14)&16383;part[2]=(low>>28)|(high<<4);
}
