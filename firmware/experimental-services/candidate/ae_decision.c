/* Fixed production profile shared by A/B: 3600 samples, 1200-half-line
 * flicker ladder, 3600 maximum exposure, gain ceiling 0B/80.
 * This is the arithmetic part of the existing auto_exposure.v policy.
 * The I2C owner/group-hold sequencing remains separately verified.
 */
typedef unsigned int u32;
__attribute__((used,noinline))
u32 ae_next(u32 exposure,u32 sum,u32 highlights,u32 mode) {
    u32 half=exposure&4095,coarse=(exposure>>16)&255,fine=exposure>>24;
    u32 low=110,high=132;
    if(mode==1)high=130;
    if(mode==2){low=128;high=152;}
    if(mode==3){low=152;high=180;}
    u32 bright=sum>3600*high||highlights>56;
    u32 dark=sum<3600*low&&highlights<28;
    u32 severe=highlights>225,step=half>>3;
    if(!step)step=1;
    if(bright){
        if(fine>128)fine=severe?(fine>144?fine-16:128):fine-4;
        else if(coarse){
            coarse=coarse==31?15:coarse==15?11:coarse==11?9:coarse==9?8:0;
            fine=severe?192:252;
        }else if(half==3600){half=2400;fine=severe?128:176;}
        else if(half==2400){half=1200;fine=severe?128:224;}
        else half=half-step<32?32:half-step;
    }else if(dark){
        if(half<1200){half=half+step>1200?1200:half+step;}
        else if(half==1200&&!coarse&&fine==252){half=2400;fine=128;}
        else if(half==2400&&!coarse&&fine==188){half=3600;fine=128;}
        else if(coarse!=31){
            if(fine<252)fine+=4;
            else{coarse=coarse==0?8:coarse==8?9:coarse==9?11:coarse==11?15:31;fine=128;}
        }
    }
    coarse&=255;fine&=255;half&=4095;
    if(coarse>11||(coarse==11&&fine>128)){coarse=11;fine=128;}
    return half|(coarse<<16)|(fine<<24);
}
