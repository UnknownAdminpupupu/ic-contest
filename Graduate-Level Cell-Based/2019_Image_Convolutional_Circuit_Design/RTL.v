`timescale 1ns/10ps

module  CONV(
            input		clk,
            input		reset,
            output		busy,
            input		ready,

            output	[11:0]	iaddr,
            input	[19:0]	idata,

            output	 	cwr,
            output	[11:0] 	caddr_wr,
            output	[19:0] 	cdata_wr,

            output	 	crd,
            output	[11:0] 	caddr_rd,
            input	[19:0] 	cdata_rd,

            output	[2:0] 	csel
        );


reg [3:0]cs;
reg [3:0]ns;
reg [2:0]addrcnt[0:1];
reg [11:0]point;
reg [19:0]inpixel[0:2][0:2];
reg kernal;
reg [19:0]max;

wire [19:0]Layer0_out;
wire [5:0]iaddrx;
wire [5:0]iaddry;
wire outside;

parameter IDLE=4'd0;
parameter LAYER0_RECIVE=4'd1;
parameter LAYER0_SEND=4'd2;
parameter LAYER0_KERNAL_CHANGE=4'd3;
parameter LAYER1_RECIVE=4'd4;
parameter LAYER1_SEND=4'd5;
parameter LAYER1_KERNAL_CHANGE=4'd6;
parameter LAYER2_RECIVE=4'd7;
parameter LAYER2_SEND=4'd8;
parameter LAYER0_DELAY=4'd9;

integer i,j;

assign busy=(cs==IDLE)?1'b0:1'b1;
assign cwr=((cs==LAYER0_SEND)|(cs==LAYER1_SEND)|(cs==LAYER2_SEND))?1'b1:1'b0;
assign cdata_wr=(cs[1:0]==2'b10)?Layer0_out:
                (cs[1:0]==2'b01)?max:cdata_rd;
assign caddr_wr=(cs==LAYER0_SEND)?point:
                ((cs==LAYER1_SEND)&(point[11:2]==0))?1023:
                (cs==LAYER1_SEND)?point[11:2]-1:
                (cs==LAYER2_SEND)?point:0;
assign crd=((cs==LAYER1_RECIVE)|(cs==LAYER2_RECIVE))?1'b1:1'b0;
assign caddr_rd=(cs==LAYER1_RECIVE)?{point[11],point[10],point[9],point[8],point[7],point[1],point[6],point[5],point[4],point[3],point[2],point[0]}:point[11:1];
assign csel=(((cs==LAYER0_SEND)|(cs==LAYER1_RECIVE))&(kernal==0))?3'b001:
            (((cs==LAYER0_SEND)|(cs==LAYER1_RECIVE))&(kernal==1))?3'b010:
            (((cs==LAYER1_SEND)|(cs==LAYER2_RECIVE))&(kernal==0))?3'b011:
            (((cs==LAYER1_SEND)|(cs==LAYER2_RECIVE))&(kernal==1))?3'b100:
            (cs==LAYER2_SEND)?3'b101:3'b000;
assign outside=(((point[5:0]+addrcnt[0])==0)|((point[11:6]+addrcnt[1])==0)|((point[5:0]+addrcnt[0])==65)|((point[11:6]+addrcnt[1])==65))?1'b1:1'b0;
assign iaddrx=(outside)?0:(point[5:0]+addrcnt[0]-1);
assign iaddry=(outside)?0:(point[11:6]+addrcnt[1]-1);
assign iaddr={iaddry,iaddrx};

always @(posedge clk or posedge reset)
    if(reset)
        cs<=IDLE;
    else
        cs<=ns;

always @(*)
case(cs)
    IDLE:
        ns<=LAYER0_RECIVE;
    LAYER0_RECIVE:
        ns<=((addrcnt[0]==2)&(addrcnt[1]==2))?LAYER0_DELAY:LAYER0_RECIVE;
    LAYER0_DELAY:
        ns<=LAYER0_SEND;
    LAYER0_SEND:
        ns<=(point==12'hFFF)?LAYER0_KERNAL_CHANGE:LAYER0_RECIVE;
    LAYER0_KERNAL_CHANGE:
        ns<=(kernal)?LAYER1_RECIVE:LAYER0_RECIVE;
    LAYER1_RECIVE:
        ns<=({point[1],point[0]}==2'b11)?LAYER1_SEND:LAYER1_RECIVE;
    LAYER1_SEND:
        ns<=(point[11:0]==0)?LAYER1_KERNAL_CHANGE:LAYER1_RECIVE;
    LAYER1_KERNAL_CHANGE:
        ns<=(kernal)?LAYER2_SEND:LAYER1_RECIVE;
    LAYER2_RECIVE:
        ns<=LAYER2_SEND;
    LAYER2_SEND:
        ns<=(point==12'hFFF)?IDLE:LAYER2_RECIVE;
    default:
        ns<=IDLE;
endcase

always @(posedge clk)
case(cs)
    IDLE:
        {addrcnt[0],addrcnt[1]}<=0;
    LAYER0_RECIVE:
    begin
        if(addrcnt[0]<2)
            addrcnt[0]<=addrcnt[0]+1;
        else if(addrcnt[1]<2)
        begin
            addrcnt[0]<=0;
            addrcnt[1]<=addrcnt[1]+1;
        end
        else
        begin
            addrcnt[0]<=0;
            addrcnt[1]<=0;
        end
    end
endcase

always @(posedge clk)
case(cs)
    IDLE,LAYER0_KERNAL_CHANGE:
        {inpixel[0][0],inpixel[1][0],inpixel[2][0],inpixel[0][1],inpixel[1][1],inpixel[2][1],inpixel[0][2],inpixel[1][2],inpixel[2][2]}<=0;
    LAYER0_RECIVE:
        if(outside)
            inpixel[addrcnt[0]][addrcnt[1]]<=0;
        else
            inpixel[addrcnt[0]][addrcnt[1]]<=idata;
endcase

//overflow reset
always @(posedge clk )
case(cs)
    IDLE,LAYER1_KERNAL_CHANGE:
        point<=0;
    LAYER0_SEND,LAYER1_RECIVE,LAYER2_SEND:
        point<=point+1;
endcase

always @(posedge clk)
case(cs)
    IDLE:
        kernal<=0;
    LAYER0_KERNAL_CHANGE,LAYER1_KERNAL_CHANGE,LAYER2_SEND:
        kernal<=!kernal;
endcase

always @(posedge clk)
case(cs)
    IDLE,LAYER1_SEND:
        max<=0;
    LAYER1_RECIVE:
        max<=(cdata_rd>max)?cdata_rd:max;
endcase

Layer0 Layer0(.clk(clk),.inpixel({inpixel[0][0],inpixel[1][0],inpixel[2][0],inpixel[0][1],inpixel[1][1],inpixel[2][1],inpixel[0][2],inpixel[1][2],inpixel[2][2]}),.kernal(kernal),.outpixel(Layer0_out));

endmodule



module Layer0(
        input clk,
        input [179:0] inpixel,
        input  kernal,
        output [19:0] outpixel
    );

wire signed [36:0]conv;
wire [179:0]kernal_value;
wire signed[19:0]kernal_bias;
reg signed [36:0] pixel_product[0:8];
wire signed [36:0] pixel_add;


assign kernal_value=(kernal)?180'hFDB55_02992_FC994_050FD_02F20_0202D_03BD7_FD369_05E68:
       180'h0A89E_092D5_06D43_01004_F8F71_F6E54_FA6D7_FC834_FAC19;


assign kernal_bias=(kernal)?20'hF7295:20'h01310;

always@(posedge clk)
begin
    pixel_product[0]=inpixel[179:160]*{{19{kernal_value[179]}},kernal_value[179:160]};
    pixel_product[1]=inpixel[159:140]*{{19{kernal_value[159]}},kernal_value[159:140]};
    pixel_product[2]=inpixel[139:120]*{{19{kernal_value[139]}},kernal_value[139:120]};
    pixel_product[3]=inpixel[119:100]*{{19{kernal_value[119]}},kernal_value[119:100]};
    pixel_product[4]=inpixel[99:80]*{{19{kernal_value[99]}},kernal_value[99:80]};
    pixel_product[5]=inpixel[79:60]*{{19{kernal_value[79]}},kernal_value[79:60]};
    pixel_product[6]=inpixel[59:40]*{{19{kernal_value[59]}},kernal_value[59:40]};
    pixel_product[7]=inpixel[39:20]*{{19{kernal_value[39]}},kernal_value[39:20]};
    pixel_product[8]=inpixel[19:0]*{{19{kernal_value[19]}},kernal_value[19:0]};
end


assign pixel_add=pixel_product[0]+pixel_product[1]+pixel_product[2]+pixel_product[3]+pixel_product[4]+pixel_product[5]+pixel_product[6]+pixel_product[7]+pixel_product[8];

assign conv=pixel_add+(kernal_bias<<16);

assign outpixel=(conv[36])?0:conv[35:16]+conv[15];


endmodule
