`timescale 1ns/1ps
// Native TJC page 3 controls for an already captured, immutable DDR model.
// Pose packing matches hd_cloud_projector. Selection never resets model RAM.
module preset_view_control(
    input wire clk, rst_n,
    input wire page_valid,
    input wire [2:0] page,
    input wire touch_valid, touch_down,
    input wire [15:0] touch_x, touch_y,
    input wire show_key, home_key,
    output reg display_requested,
    output wire [47:0] pose_record,
    output wire token_exhausted
);
    localparam SHOW=1, HOME=2, ORBIT=3, LEFT=4, RIGHT=5,
               UP=6, DOWN=7, ZOOM_IN=8, ZOOM_OUT=9;
    reg [2:0] current_page;
    reg pressed;
    reg [3:0] pressed_command;
    reg signed [7:0] yaw, pitch;
    reg [7:0] zoom;
    reg auto_rotate;
    reg [15:0] token;
    reg [3:0] hit, action;
    reg signed [7:0] next_yaw, next_pitch;
    reg [7:0] next_zoom;
    reg next_auto;

    assign pose_record={7'd0,token,auto_rotate,zoom,pitch,yaw};
    assign token_exhausted=token==16'hffff;

    always @* begin
        hit=0;
        if(current_page==3) begin
            if(touch_y>=146 && touch_y<188) begin
                if(touch_x>=68 && touch_x<263) hit=SHOW;
                else if(touch_x>=274 && touch_x<368) hit=HOME;
                else if(touch_x>=379 && touch_x<470) hit=ORBIT;
            end else if(touch_y>=200 && touch_y<243) begin
                if(touch_x>=68 && touch_x<131) hit=LEFT;
                else if(touch_x>=136 && touch_x<199) hit=RIGHT;
                else if(touch_x>=204 && touch_x<267) hit=UP;
                else if(touch_x>=272 && touch_x<335) hit=DOWN;
                else if(touch_x>=340 && touch_x<403) hit=ZOOM_IN;
                else if(touch_x>=408 && touch_x<470) hit=ZOOM_OUT;
            end
        end
        action=0;
        if(touch_valid && !touch_down && pressed && hit==pressed_command)
            action=hit;
        if(home_key) action=HOME;
        next_yaw=yaw;
        next_pitch=pitch;
        next_zoom=zoom;
        next_auto=auto_rotate;
        case(action)
            HOME: begin next_yaw=0; next_pitch=16; next_zoom=16; next_auto=0; end
            ORBIT: next_auto=!auto_rotate;
            LEFT: begin if(yaw>-24) next_yaw=yaw-8'sd4; next_auto=0; end
            RIGHT: begin if(yaw<24) next_yaw=yaw+8'sd4; next_auto=0; end
            UP: begin if(pitch<24) next_pitch=pitch+8'sd4; next_auto=0; end
            DOWN: begin if(pitch>-24) next_pitch=pitch-8'sd4; next_auto=0; end
            ZOOM_IN: begin if(zoom<24) next_zoom=zoom+8'd2; end
            ZOOM_OUT: begin if(zoom>8) next_zoom=zoom-8'd2; end
            default: begin end
        endcase
    end

    always @(posedge clk) begin
        if(!rst_n) begin
            current_page<=0;
            pressed<=0;
            pressed_command<=0;
            display_requested<=0;
            yaw<=0;
            pitch<=16;
            zoom<=16;
            auto_rotate<=0;
            token<=1;
        end else begin
            if(page_valid) begin
                current_page<=page;
                if(page!=current_page) pressed<=0;
                if(page==0 || page==1) display_requested<=0;
            end
            if(touch_valid) begin
                if(touch_down) begin
                    pressed<=hit!=0;
                    pressed_command<=hit;
                end else pressed<=0;
            end
            if(show_key) display_requested<=!display_requested;
            if(action!=0) display_requested<=1;
            if(!token_exhausted &&
               {next_auto,next_zoom,next_pitch,next_yaw}!={auto_rotate,zoom,pitch,yaw}) begin
                yaw<=next_yaw;
                pitch<=next_pitch;
                zoom<=next_zoom;
                auto_rotate<=next_auto;
                token<=token+1'b1;
            end
        end
    end
endmodule
