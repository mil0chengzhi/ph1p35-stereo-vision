#ifndef TRACK_CORE_H
#define TRACK_CORE_H
typedef unsigned int tr_u32;
typedef struct {
 tr_u32 identity,last_frame,reference_x,reference_y,reference_count;
 tr_u32 center_x,center_y,flags;
} TrackState;
enum { TR_REFERENCE=1,TR_CONFIRMED=2,TR_HAVE_FRAME=4,TR_VISIBLE=8 };
void track_reset(TrackState *s,tr_u32 zero_identity);
void track_copy(TrackState *to,const TrackState *from);
tr_u32 track_step(TrackState *s,tr_u32 frame,tr_u32 valid,const tr_u32 *grid_box);
void track_pack(const TrackState *s,tr_u32 part[3]);
#endif
