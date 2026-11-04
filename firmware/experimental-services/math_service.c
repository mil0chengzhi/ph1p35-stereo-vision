/* Isolated single-real-core candidate: AE always has foreground priority.
 * No busy-wait for box completion; one bounded calculation per active token.
 * Status16/token17/frame18/packed bbox20..27/meta28/commit29/drain30.
 */
#include "box_filter.h"
#include "track_core_events_v3.h"
#include "candidate/ae_decision.c"
#include "awb_decision.c"
#define IO ((volatile u32 *)0x80008000u)
static BoxState state;
static BoxInput input;
static u32 previous_ae;
/* Reserved status bits are a PROPOSED held request bundle, not current RTL.
 * The next owned token carries acceptance of the preceding result. Public
 * pixel cancellation is immediate and separately sticky through mode ABA.
 * Busy-withdraw event transport is not yet an integrated RTL contract.
 */
enum { TR_ENABLE=1u<<8,TR_PREVIOUS_ACCEPTED=1u<<9,
       TR_RESET_HISTORY=1u<<10,TR_ZERO_IDENTITY=1u<<11,TR_PREVIOUS_FRESH=1u<<12 };
static TrackState committed_track,pending_track;
static volatile u32 pending_track_valid;
static u32 pending_track_token;
static void reconcile_track(u32 status,u32 token){
 if(pending_track_valid){
  if(token==pending_track_token+1u&&(status&TR_PREVIOUS_ACCEPTED)){
   if(status&TR_PREVIOUS_FRESH){
    track_reset(&committed_track,0);
    track_step(&committed_track,state.output_frame,state.published_valid&1u,state.published);
   }else track_copy(&committed_track,&pending_track);
  }
  pending_track_valid=0;
 }
 if(status&(TR_RESET_HISTORY|TR_ZERO_IDENTITY))
  track_reset(&committed_track,!!(status&TR_ZERO_IDENTITY));
}

static inline void ordered(void){__asm__ volatile("fence iorw,iorw":::"memory");}
static int service_ae(void){
 u32 command=IO[0],ticket=command&65535u;
 if(!(command&0x80000000u)||ticket==previous_ae)return 0;
 u32 exposure=IO[1],sum=IO[2],info=IO[3];
 u32 highlight=info&4095u,mode=(info>>16)&3u,low=110,high=132;
 if(mode==1)high=130;if(mode==2){low=128;high=152;}if(mode==3){low=152;high=180;}
 u32 bright=sum>3600*high||highlight>56,dark=sum<3600*low&&highlight<28;
 IO[5]=ae_next(exposure,sum,highlight,mode)|(bright<<12)|(dark<<13);
 ordered();IO[6]=ticket;ordered();previous_ae=ticket;return 1;
}
static void discard_box(u32 token,u32 frame){
 input.frame=frame;input.flags=BOX_SOURCE_VALID|BOX_RESULT|BOX_BUSY_VIOLATION;
 box_process(&state,&input);ordered();IO[30]=token;ordered();
}
static int service_box(void){
 u32 status=IO[16];if(!(status&0x80000000u))return 0;
 u32 token=IO[17],frame=IO[18];
 reconcile_track(status,token);service_ae();
 if(status&0x60000000u){
  if((status&0x40000000u)&&(status&TR_ENABLE))
   for(u32 n=(status>>13)&7u;n;n--)
    track_step(&committed_track,frame,0x80000000u,state.published);
  discard_box(token,frame);return 1;
 }
 input.frame=frame;input.flags=BOX_SOURCE_VALID|BOX_RESULT|(((status>>4)&15u)<<8);
 for(u32 k=0;k<4;k++){
  u32 x=IO[20+k*2],y=IO[21+k*2];
  input.xy[k*4]=x&65535u;input.xy[k*4+1]=x>>16;
  input.xy[k*4+2]=y&65535u;input.xy[k*4+3]=y>>16;
 }
 ordered();
 if(IO[17]!=token||(IO[16]&0xe0000000u)!=0x80000000u){discard_box(token,frame);return 1;}
 box_process(&state,&input);
 // Bound foreground interference: service an AE that arrived during the
 // <=1470-instruction pure filter before spending time on result transfers.
 service_ae();
 if(IO[17]!=token||(IO[16]&0xe0000000u)!=0x80000000u){discard_box(token,frame);return 1;}
 u32 track_parts[3]={0,0,0};
 if(state.commit){
  track_copy(&pending_track,&committed_track);
  u32 initial=track_step(&pending_track,frame,state.published_valid&1u,state.published);
  pending_track_valid=1;pending_track_token=token;
  track_pack(&pending_track,track_parts);track_parts[2]|=initial<<12;
 }
 service_ae();
 for(u32 k=0;k<8;k++){
  u32 data=state.published[k*2]|(state.published[k*2+1]<<9);
  if(k<3)data|=track_parts[k]<<18;
  IO[20+k]=data;
 }
 service_ae();ordered();
 if(IO[17]!=token||(IO[16]&0xe0000000u)!=0x80000000u){discard_box(token,frame);return 1;}
 IO[28]=state.published_valid|(state.commit<<8);ordered();IO[29]=token;ordered();return 1;
}
void main(void){
 u32 previous_awb=0;previous_ae=0;box_reset(&state);track_reset(&committed_track,1);pending_track_valid=0;pending_track_token=0;
 // The parent gates box worker_ready on this marker; state exists first.
 IO[7]=0x4d435531u;ordered();
 for(;;){
  if(service_ae())continue;
  if(service_box())continue;
  u32 meta=IO[8],sequence=meta&65535u;
  if((meta&0x70000u)!=0x70000u||sequence==previous_awb)continue;
  u32 red=IO[9],green=IO[10],blue=IO[11];ordered();if(IO[8]!=meta)continue;
  IO[12]=(sequence<<16)|awb_classify(red,green,blue);ordered();previous_awb=sequence;
 }
}
