/* Exactly the original neutral_awb threshold expressions, including integer
 * truncation. Validity/coverage/sample counting remain in source-clock RTL.
 * Packet: flags0 red-low,1 red-high,2 blue-low,3 blue-high.
 */
typedef unsigned int awb_u32;
__attribute__((used,noinline))
awb_u32 awb_classify(awb_u32 red,awb_u32 green,awb_u32 blue) {
    red &= 1048575u; green &= 1048575u; blue &= 1048575u;
    awb_u32 low=green-(green>>5),high=green+(green>>5);
    return (red<low)|((red>high)<<1)|((blue<low)<<2)|((blue>high)<<3);
}
