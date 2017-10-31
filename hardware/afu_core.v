module afu_core (
    input  wire                             Clk_400,
    input  wire                             reset_n,
	input  wire 							re2xy_go,
    input  wire                             spl_reset,
    
    // TX_RD request, afu_core --> afu_io
	input reg ab2l1_stallRd,
    output reg                              l12ab_RdEn,
    output reg  [41:0]                      l12ab_RdAddr,
	output reg  [15:0]                      l12ab_RdTID,   
   
    // TX_WR request, afu_core --> afu_io
	input reg ab2l1_WrAlmFull,
    output reg                              l12ab_WrEn,
	output reg  [41:0]                      l12ab_WrAddr,
	output reg  [15:0]                      l12ab_WrTID,
	output reg  [511:0]                     l12ab_WrDin,
    output reg                              l12ab_WrFence,


    // RX_RD response, afu_io --> afu_core
    input  wire                             ab2l1_RdRspValid_T0,
    input  wire [511:0]                     ab2l1_RdData_T0,    
    input  wire [15:0]						ab2l1_RdRsp_T0,
	
	input									ab2l1_WrRspValid_T0,
    input	[15:0]							ab2l1_WrRsp_T0,         	
	input reg [41:0] 						ctx_src_ptr,
	input reg [41:0] 						ctx_dst_ptr

);
	wire test_Resetb;
	assign test_Resetb = reset_n & ~spl_reset;
    localparam [2:0]
        BWT_STATE__IDLE         = 3'b000,
        BWT_STATE__CNT_polling  = 3'b001,
        BWT_STATE__CNT_read     = 3'b010,
        BWT_STATE__BWT_polling  = 3'b011,
        BWT_STATE__BWT_read     = 3'b100,
        BWT_STATE__BWT_run      = 3'b101,
		BWT_STATE__WRITEFINISH	= 3'b110,
		BWT_STATE__HANDFINISH	= 3'b111;

	logic   [2:0]			BWT_state;
    logic   [1:0]           read_fsm;
    

    logic   [31:0]          Num_Write_req;
    logic   [31:0]          Num_Write_rsp;  
    logic   [31:0]			Total_Num_RdRq;
	logic   [31:0]          Total_Num_RdRsp;

    logic                   ab2l1_WrAlmFull_q, ab2l1_WrAlmFull_qq;
    logic   [1:0]           wrCLnum, wrCLnum_q, wrCLnum_qq, wrCLnum_qqq;
    logic                   ram_rdValid, ram_rdValid_q, ram_rdValid_qq, ram_rdValid_qqq;
    logic                   wrsop, wrsop_q, wrsop_qq, wrsop_qqq;
    logic   [6:0]           WrReq_tid, WrReq_tid_q;
    logic   [6:0]           WrReq_tid_mCL;
    logic   [6:0]           i;
    logic   [2:0]           multiCL_num;  
    logic                   rd_done;
	logic [4:0] Num_Read_count,Num_Read_count_q;

    logic   [4:0]           ram_max_index;
    logic   [7:0]           Num_ram_reads;
    logic   [20:0]          Total_ram_reads;                      
    logic   [15:0]          ab2l1_WrRsp;
    logic   [1:0]           ab2l1_WrRspCLnum;
    logic   [2:0]           pckd_num_wr_rsp;
    logic   [15:0]  ab2l1_WrRspAddr;
    logic                   ab2l1_WrRspFormat;
    logic                   ab2l1_WrRspValid; 
    logic   [15:0]          ab2l1_RdRsp;
    logic   [1:0]           ab2l1_RdRspCLnum;
    logic   [15:0]  ab2l1_RdRspAddr;  
    logic   [511:0]         ab2l1_RdData;       
    logic                   ab2l1_RdRspValid;   

	logic  [41:0]                     BWT_base0,BWT_base1,BWT_base2,BWT_base3;   
    logic  [41:0]                     CNT_base;    
    logic  [41:0]                     input_base0,input_base1,input_base2,input_base3;  
    logic  [41:0]                     output_base; 
    logic  [41:0]                     hand_ptr;   

	//declare variables
	logic  [3:0]      CNT_counter;
	logic  [5:0]		request_tag;
	logic rx_wr_finish;
	logic [3:0]	PE_id;

	reg			Request_start;
	reg			korl; //should be initiated as 0;
	reg [4:0]	PE_num,PE_num_q;

	logic err_Rx_BWT_run;
	
	//Signals to read input
    logic  [9:0]  batch_size, batch_size_temp;
	logic  [9:0]  read_in_num;
    wire [9:0]  read_in_num_next;
	assign read_in_num_next = read_in_num + 1;
	logic  [9:0]	out_read_num; 
	logic out_read_finish;	

	logic			CNT_polling, CNT_read, BWT_polling, BWT_run,BWT_get;
	

	logic          	           CNT_trigger;
    logic                      CNT_get;
    logic    	               CNT_next_read;
    logic [2:0]                CNT_read_number;
    logic                      CNT_flag1,CNT_flag2;
    reg [63:0]                      CNT_in1,CNT_in2;
    reg [7:0]                       CNT_addr1,CNT_addr2;
    logic [511:0]              CNT_data;
    logic                      CNT_done;

	logic			Write_Res,Write_Fence;
	logic			pop_done;
	logic			CNT_polling_received, CNT_read_tag, polling_tag;
	logic           BWT_read_tag_0;
    logic           BWT_read_tag_1;
	logic			reset_read, pop_done_pulse_n, pop_done_q;
	
	logic pop_hold,readout;	
	logic[4:0]	token_id,token_id_q;
	
    //output control
    logic Write_finish;
    
    logic [41:0]		output_addr;

	
    logic [6:0] P_out_count;
    logic P_out_flag;
    
    reg cor_tx_wr_valid_0, cor_tx_wr_valid_1, cor_tx_wr_valid_2, cor_tx_wr_valid_3;
	
    reg [41:0] cor_tx_rd_addr_a0, cor_tx_rd_addr_a1, cor_tx_rd_addr_a2, cor_tx_rd_addr_a3;
    reg [41:0] cor_tx_rd_addr_b0, cor_tx_rd_addr_b1, cor_tx_rd_addr_b2, cor_tx_rd_addr_b3;  
    reg [41:0] cor_tx_wr_addr_0, cor_tx_wr_addr_1, cor_tx_wr_addr_2, cor_tx_wr_addr_3;
    reg [511:0] cor_tx_data_0, cor_tx_data_1, cor_tx_data_2, cor_tx_data_3;
	reg [41:0] R0_CL_addr_k, R1_CL_addr_k, R2_CL_addr_k, R3_CL_addr_k; 
    reg [41:0] R4_CL_addr_k, R5_CL_addr_k, R6_CL_addr_k, R7_CL_addr_k; 
    reg [41:0] R8_CL_addr_k, R9_CL_addr_k, R10_CL_addr_k, R11_CL_addr_k;
    reg [41:0] R12_CL_addr_k, R13_CL_addr_k, R14_CL_addr_k, R15_CL_addr_k;
    reg [41:0] R0_CL_addr_l, R1_CL_addr_l, R2_CL_addr_l, R3_CL_addr_l; 
    reg [41:0] R4_CL_addr_l, R5_CL_addr_l, R6_CL_addr_l, R7_CL_addr_l; 
    reg [41:0] R8_CL_addr_l, R9_CL_addr_l, R10_CL_addr_l, R11_CL_addr_l;
    reg [41:0] R12_CL_addr_l, R13_CL_addr_l, R14_CL_addr_l, R15_CL_addr_l;
    
	reg R0_DRAM_valid_hold, R1_DRAM_valid_hold, R2_DRAM_valid_hold, R3_DRAM_valid_hold,
        R4_DRAM_valid_hold, R5_DRAM_valid_hold, R6_DRAM_valid_hold, R7_DRAM_valid_hold,
        R8_DRAM_valid_hold, R9_DRAM_valid_hold, R10_DRAM_valid_hold, R11_DRAM_valid_hold,
        R12_DRAM_valid_hold, R13_DRAM_valid_hold, R14_DRAM_valid_hold, R15_DRAM_valid_hold;
    reg R0_read_done_hold, R1_read_done_hold, R2_read_done_hold, R3_read_done_hold,
        R4_read_done_hold, R5_read_done_hold, R6_read_done_hold, R7_read_done_hold,
        R8_read_done_hold, R9_read_done_hold, R10_read_done_hold, R11_read_done_hold,
        R12_read_done_hold, R13_read_done_hold, R14_read_done_hold, R15_read_done_hold;
    
	reg R0_BWT_read_req_hold;
    reg R1_BWT_read_req_hold;
    reg R2_BWT_read_req_hold;
    reg R3_BWT_read_req_hold;
    reg R4_BWT_read_req_hold;
    reg R5_BWT_read_req_hold;
    reg R6_BWT_read_req_hold;
    reg R7_BWT_read_req_hold;
    reg R8_BWT_read_req_hold;
    reg R9_BWT_read_req_hold;
    reg R10_BWT_read_req_hold;
    reg R11_BWT_read_req_hold;
    reg R12_BWT_read_req_hold;
    reg R13_BWT_read_req_hold;
    reg R14_BWT_read_req_hold;
    reg R15_BWT_read_req_hold;  
	logic [511:0] R0_BWT_input_dataK,R0_BWT_input_dataL,R1_BWT_input_dataK,R1_BWT_input_dataL,R2_BWT_input_dataK,R2_BWT_input_dataL;
	logic [511:0] R3_BWT_input_dataK,R3_BWT_input_dataL,R4_BWT_input_dataK,R4_BWT_input_dataL,R5_BWT_input_dataK,R5_BWT_input_dataL,
					R6_BWT_input_dataK,R6_BWT_input_dataL,R7_BWT_input_dataK,R7_BWT_input_dataL,R8_BWT_input_dataK,R8_BWT_input_dataL,
					R9_BWT_input_dataK,R9_BWT_input_dataL,R10_BWT_input_dataK,R10_BWT_input_dataL,R11_BWT_input_dataK,R11_BWT_input_dataL,
					R12_BWT_input_dataK,R12_BWT_input_dataL,R13_BWT_input_dataK,R13_BWT_input_dataL,R14_BWT_input_dataK,R14_BWT_input_dataL,
					R15_BWT_input_dataK,R15_BWT_input_dataL;
	logic [1:0]		PE0_rec,PE1_rec,PE2_rec,PE3_rec,PE4_rec,PE5_rec,PE6_rec,
					PE7_rec,PE8_rec,PE9_rec,PE10_rec,PE11_rec,PE12_rec,PE13_rec,PE14_rec,PE15_rec; 
	logic			PE0_start,PE1_start,PE2_start,PE3_start,PE4_start,PE5_start,PE6_start,PE7_start,
					PE8_start,PE9_start,PE10_start,PE11_start,PE12_start,PE13_start,PE14_start,PE15_start;

	logic			cor_tx_rd_valid_a, cor_tx_rd_valid_b;
	
	 // Input of PE_0
    reg R0_BWT_input_valid;
    reg R0_read_ack;
    reg [9:0] R0_read_num;
    reg [511:0] R0_BWT_input; 
    reg R0_match_pop_pulse;
    reg R0_pop_done;

    // Output of PE_0
    wire R0_BWT_read_req;
    wire R0_read_trigger;
    wire R0_DRAM_valid;
    wire [31:0] R0_addr_k, R0_addr_l;
    wire [31:0] R0_ret;
    wire [6:0]  R0_mem_size;
    wire [63:0] R0_mem_out_x0,R0_mem_out_x1, R0_mem_out_x2; 
    wire [63:0] R0_mem_out_info;
    wire R0_read_done_pulse;

	// Input of PE_1
    reg R1_BWT_input_valid;
    reg R1_read_ack;
    reg [9:0] R1_read_num;
    reg [511:0] R1_BWT_input;  
    reg R1_match_pop_pulse;
    reg R1_pop_done;

    // Output of PE_1
    wire R1_BWT_read_req;
    wire R1_read_trigger;
    wire R1_DRAM_valid;
    wire [31:0] R1_addr_k, R1_addr_l;
    wire [31:0] R1_ret;
    wire [6:0]  R1_mem_size;
    wire [63:0] R1_mem_out_x0,R1_mem_out_x1, R1_mem_out_x2; 
    wire [63:0] R1_mem_out_info;
    wire R1_read_done_pulse;
    
    // Input of PE_2
    reg R2_BWT_input_valid;
    reg R2_read_ack;
    reg [9:0] R2_read_num;
    reg [511:0] R2_BWT_input;  
    reg R2_match_pop_pulse;
    reg R2_pop_done;

    // Output of PE_2
    wire R2_BWT_read_req;
    wire R2_read_trigger;
    wire R2_DRAM_valid;
    wire [31:0] R2_addr_k, R2_addr_l;
    wire [31:0] R2_ret;
    wire [6:0]  R2_mem_size;
    wire [63:0] R2_mem_out_x0,R2_mem_out_x1, R2_mem_out_x2; 
    wire [63:0] R2_mem_out_info;
    wire R2_read_done_pulse;
    
    // Input of PE_3
    reg R3_BWT_input_valid;
    reg R3_read_ack;
    reg [9:0] R3_read_num;
    reg [511:0] R3_BWT_input;  
    reg R3_match_pop_pulse;
    reg R3_pop_done;

    // Output of PE_3
    wire R3_BWT_read_req;
    wire R3_read_trigger;
    wire R3_DRAM_valid;
    wire [31:0] R3_addr_k, R3_addr_l;
    wire [31:0] R3_ret;
    wire [6:0]  R3_mem_size;
    wire [63:0] R3_mem_out_x0,R3_mem_out_x1, R3_mem_out_x2; 
    wire [63:0] R3_mem_out_info;
    wire R3_read_done_pulse;
    
    // Input of PE_4
    reg R4_BWT_input_valid;
    reg R4_read_ack;
    reg [9:0] R4_read_num;
    reg [511:0] R4_BWT_input;  
    reg R4_match_pop_pulse;
    reg R4_pop_done;

    // Output of PE_4
    wire R4_BWT_read_req;
    wire R4_read_trigger;
    wire R4_DRAM_valid;
    wire [31:0] R4_addr_k, R4_addr_l;
    wire [31:0] R4_ret;
    wire [6:0]  R4_mem_size;
    wire [63:0] R4_mem_out_x0,R4_mem_out_x1, R4_mem_out_x2; 
    wire [63:0] R4_mem_out_info;
    wire R4_read_done_pulse;
    
    // Input of PE_5
    reg R5_BWT_input_valid;
    reg R5_read_ack;
    reg [9:0] R5_read_num;
    reg [511:0] R5_BWT_input;  
    reg R5_match_pop_pulse;
    reg R5_pop_done;

    // Output of PE_5
    wire R5_BWT_read_req;
    wire R5_read_trigger;
    wire R5_DRAM_valid;
    wire [31:0] R5_addr_k, R5_addr_l;
    wire [31:0] R5_ret;
    wire [6:0]  R5_mem_size;
    wire [63:0] R5_mem_out_x0,R5_mem_out_x1, R5_mem_out_x2; 
    wire [63:0] R5_mem_out_info;
    wire R5_read_done_pulse;
    
    // Input of PE_6
    reg R6_BWT_input_valid;
    reg R6_read_ack;
    reg [9:0] R6_read_num;
    reg [511:0] R6_BWT_input;  
    reg R6_match_pop_pulse;
    reg R6_pop_done;

    // Output of PE_6
    wire R6_BWT_read_req;
    wire R6_read_trigger;
    wire R6_DRAM_valid;
    wire [31:0] R6_addr_k, R6_addr_l;
    wire [31:0] R6_ret;
    wire [6:0]  R6_mem_size;
    wire [63:0] R6_mem_out_x0,R6_mem_out_x1, R6_mem_out_x2; 
    wire [63:0] R6_mem_out_info;
    wire R6_read_done_pulse;
    
    // Input of PE_7
    reg R7_BWT_input_valid;
    reg R7_read_ack;
    reg [9:0] R7_read_num;
    reg [511:0] R7_BWT_input;  
    reg R7_match_pop_pulse;
    reg R7_pop_done;

    // Output of PE_7
    wire R7_BWT_read_req;
    wire R7_read_trigger;
    wire R7_DRAM_valid;
    wire [31:0] R7_addr_k, R7_addr_l;
    wire [31:0] R7_ret;
    wire [6:0]  R7_mem_size;
    wire [63:0] R7_mem_out_x0,R7_mem_out_x1, R7_mem_out_x2; 
    wire [63:0] R7_mem_out_info;
    wire R7_read_done_pulse;
    
    // Input of PE_8
    reg R8_BWT_input_valid;
    reg R8_read_ack;
    reg [9:0] R8_read_num;
    reg [511:0] R8_BWT_input;  
    reg R8_match_pop_pulse;
    reg R8_pop_done;

    // Output of PE_8
    wire R8_BWT_read_req;
    wire R8_read_trigger;
    wire R8_DRAM_valid;
    wire [31:0] R8_addr_k, R8_addr_l;
    wire [31:0] R8_ret;
    wire [6:0]  R8_mem_size;
    wire [63:0] R8_mem_out_x0,R8_mem_out_x1, R8_mem_out_x2; 
    wire [63:0] R8_mem_out_info;
    wire R8_read_done_pulse;
    
    // Input of PE_9
    reg R9_BWT_input_valid;
    reg R9_read_ack;
    reg [9:0] R9_read_num;
    reg [511:0] R9_BWT_input;  
    reg R9_match_pop_pulse;
    reg R9_pop_done;

    // Output of PE_9
    wire R9_BWT_read_req;
    wire R9_read_trigger;
    wire R9_DRAM_valid;
    wire [31:0] R9_addr_k, R9_addr_l;
    wire [31:0] R9_ret;
    wire [6:0]  R9_mem_size;
    wire [63:0] R9_mem_out_x0, R9_mem_out_x1, R9_mem_out_x2; 
    wire [63:0] R9_mem_out_info;
    wire R9_read_done_pulse;
        
    // Input of PE_10
    reg R10_BWT_input_valid;
    reg R10_read_ack;
    reg [9:0] R10_read_num;
    reg [511:0] R10_BWT_input;  
    reg R10_match_pop_pulse;
    reg R10_pop_done;
    
    // Output of PE_10
    wire R10_BWT_read_req;
    wire R10_read_trigger;
    wire R10_DRAM_valid;
    wire [31:0] R10_addr_k, R10_addr_l;
    wire [31:0] R10_ret;
    wire [6:0]  R10_mem_size;
    wire [63:0] R10_mem_out_x0, R10_mem_out_x1, R10_mem_out_x2; 
    wire [63:0] R10_mem_out_info;
    wire R10_read_done_pulse;
    
    // Input of PE_11
    reg R11_BWT_input_valid;
    reg R11_read_ack;
    reg [9:0] R11_read_num;
    reg [511:0] R11_BWT_input;  
    reg R11_match_pop_pulse;
    reg R11_pop_done;
    
    // Output of PE_11
    wire R11_BWT_read_req;
    wire R11_read_trigger;
    wire R11_DRAM_valid;
    wire [31:0] R11_addr_k, R11_addr_l;
    wire [31:0] R11_ret;
    wire [6:0]  R11_mem_size;
    wire [63:0] R11_mem_out_x0, R11_mem_out_x1, R11_mem_out_x2; 
    wire [63:0] R11_mem_out_info;
    wire R11_read_done_pulse;
    
    // Input of PE_12
    reg R12_BWT_input_valid;
    reg R12_read_ack;
    reg [9:0] R12_read_num;
    reg [511:0] R12_BWT_input;  
    reg R12_match_pop_pulse;
    reg R12_pop_done;
    
    // Output of PE_12
    wire R12_BWT_read_req;
    wire R12_read_trigger;
    wire R12_DRAM_valid;
    wire [31:0] R12_addr_k, R12_addr_l;
    wire [31:0] R12_ret;
    wire [6:0]  R12_mem_size;
    wire [63:0] R12_mem_out_x0, R12_mem_out_x1, R12_mem_out_x2; 
    wire [63:0] R12_mem_out_info;
    wire R12_read_done_pulse;
    
    // Input of PE_13
    reg R13_BWT_input_valid;
    reg R13_read_ack;
    reg [9:0] R13_read_num;
    reg [511:0] R13_BWT_input;  
    reg R13_match_pop_pulse;
    reg R13_pop_done;
    
    // Output of PE_13
    wire R13_BWT_read_req;
    wire R13_read_trigger;
    wire R13_DRAM_valid;
    wire [31:0] R13_addr_k, R13_addr_l;
    wire [31:0] R13_ret;
    wire [6:0]  R13_mem_size;
    wire [63:0] R13_mem_out_x0, R13_mem_out_x1, R13_mem_out_x2; 
    wire [63:0] R13_mem_out_info;
    wire R13_read_done_pulse;
    
    // Input of PE_14
    reg R14_BWT_input_valid;
    reg R14_read_ack;
    reg [9:0] R14_read_num;
    reg [511:0] R14_BWT_input;  
    reg R14_match_pop_pulse;
    reg R14_pop_done;
    
    // Output of PE_14
    wire R14_BWT_read_req;
    wire R14_read_trigger;
    wire R14_DRAM_valid;
    wire [31:0] R14_addr_k, R14_addr_l;
    wire [31:0] R14_ret;
    wire [6:0]  R14_mem_size;
    wire [63:0] R14_mem_out_x0, R14_mem_out_x1, R14_mem_out_x2; 
    wire [63:0] R14_mem_out_info;
    wire R14_read_done_pulse;
    
    // Input of PE_15
    reg R15_BWT_input_valid;
    reg R15_read_ack;
    reg [9:0] R15_read_num;
    reg [511:0] R15_BWT_input;  
    reg R15_match_pop_pulse;
    reg R15_pop_done;
    
    // Output of PE_15
    wire R15_BWT_read_req;
    wire R15_read_trigger;
    wire R15_DRAM_valid;
    wire [31:0] R15_addr_k, R15_addr_l;
    wire [31:0] R15_ret;
    wire [6:0]  R15_mem_size;
    wire [63:0] R15_mem_out_x0, R15_mem_out_x1, R15_mem_out_x2; 
    wire [63:0] R15_mem_out_info;
    wire R15_read_done_pulse;


	always@(posedge Clk_400)
    begin
      ab2l1_WrRsp        <= ab2l1_WrRsp_T0;
      ab2l1_WrRspValid   <= ab2l1_WrRspValid_T0;
      ab2l1_RdRsp        <= ab2l1_RdRsp_T0;
      ab2l1_RdData       <= ab2l1_RdData_T0;
      ab2l1_RdRspValid   <= ab2l1_RdRspValid_T0; 
    end 

	always_comb 
    begin
	//relative position initialization
	       BWT_base0    <= ctx_src_ptr;
           BWT_base1    <= ctx_src_ptr;
           BWT_base2    <= ctx_src_ptr;
           BWT_base3    <= ctx_src_ptr;
           CNT_base     <= ctx_src_ptr + 50331648;
           input_base0  <= ctx_src_ptr + 50331648 + 16384;
           input_base1  <= ctx_src_ptr + 50331648 + 16384;
           input_base2  <= ctx_src_ptr + 50331648 + 16384;
           input_base3  <= ctx_src_ptr + 50331648 + 16384;
           output_base  <= ctx_src_ptr + 50331648 + 16384 + 16384;
           hand_ptr     <= ctx_src_ptr + 50331648 + 16383;
	end
	
//////////////////////////////////////////////////////////////
//						BWT STATE CONTROL					//
//////////////////////////////////////////////////////////////
	//BWT_state_control
	always @(posedge Clk_400) 
	begin
		if(!test_Resetb)
		begin
			BWT_state <= BWT_STATE__IDLE;
			CNT_polling <= 0;
			CNT_read	<= 0;
			BWT_polling <= 0;
			BWT_run		<= 0;
		//tx wr control signals
			CNT_done    <= 0;
			Write_Res	<= 0; 
			Write_Fence <= 0;
			pop_done	<= 0;
			Write_finish	<= 0;
		//state control signals	
			polling_tag				<= 0;
			CNT_counter		<= 0;
            batch_size			<= 16;
		end
		else if (!reset_read)
		begin 
			CNT_polling <= 0;
			CNT_read	<= 0;
			BWT_polling <= 1;
			BWT_run		<= 0;
		//tx wr control signals
			CNT_done    <= 0;
			Write_Res	<= 0; 
			Write_Fence <= 0;
			pop_done	<= 0;
			Write_finish	<= 0;
		//state control signals	
			CNT_counter		<=0;
            batch_size			<= 16;
			BWT_state <= BWT_STATE__BWT_polling;
		end
		else
		begin
			case (BWT_state)
			BWT_STATE__IDLE:
			begin
			//tx rd control signals
				
				CNT_read	<= 0;
				BWT_polling <= 0;
				BWT_run		<= 0;
			//tx wr control signals
				CNT_done    <= 0;
				Write_Res	<= 0; 
				Write_Fence <= 0;
				pop_done	<= 0;
			//state control signals	
				polling_tag				<= 0;
				if(re2xy_go)
				begin
					CNT_polling		<= 1;
					BWT_state		<= BWT_STATE__CNT_polling;
				end

			end
			BWT_STATE__CNT_polling:
			begin
				CNT_read	<= 0;
				BWT_polling <= 0;
				BWT_run		<= 0;
				CNT_done    <= 0;
				Write_Res	<= 0;
				if(CNT_polling_received & CNT_read_tag)
				begin
					BWT_state	<= BWT_STATE__CNT_read;
					CNT_read	<= 1;
				end	
				else if(CNT_polling_received & !CNT_read_tag)
				begin
					CNT_polling <= 1;
				end
				else
				begin
					CNT_polling <= 0;
				end
			end
			BWT_STATE__CNT_read:
			begin
				CNT_polling <= 0;
				BWT_run		<= 0;
				Write_Res	<= 0;
				if(CNT_next_read) begin		//after processing all input data, proceed to the next
          			if(CNT_counter == 15) begin
          				BWT_polling		<= 1;
          				CNT_read		<= 0;
          				BWT_state		<= BWT_STATE__BWT_polling; //after transmitting the CNT table, goto BWT polling
          				CNT_counter		<= 0;
          				CNT_done		<= 1;
          			end
          			else begin
          				BWT_polling		<= 0;
          				CNT_read		<= 1;
          				BWT_state		<= BWT_STATE__CNT_read;
          				CNT_counter		<= CNT_counter + 1;
          			end
				end
				else begin
					BWT_polling			<= 0;
            		CNT_read			<= 0;
            		BWT_state			<= BWT_STATE__CNT_read;
            		CNT_counter			<= CNT_counter;
				end
			 end

			BWT_STATE__BWT_polling:
			begin
				CNT_done <= 0;
				Write_Res	<= 0;
				pop_done	<= 0;
				if(BWT_get)
				begin
					if(BWT_read_tag_0 && (polling_tag==0)) 
					begin
        				BWT_state		<= BWT_STATE__BWT_run;
        				BWT_polling		<= 0;
        				polling_tag		<= 1;

        				batch_size <= batch_size_temp;
        			end
        			else if(BWT_read_tag_1 && (polling_tag==1)) 
					begin
        		  		BWT_state		<= BWT_STATE__BWT_run;
        				BWT_polling		<= 0;
        				polling_tag		<= 0;
        				batch_size <= batch_size_temp;
        			end
					else 
					begin
            			BWT_state		<= BWT_STATE__BWT_polling;
        				BWT_polling		<= 1;
        			end
				end
				else
				begin
					BWT_state			<= BWT_STATE__BWT_polling;
					BWT_polling			<= 0;
				end
			end

			BWT_STATE__BWT_run:  //assume first receive k then l in harp1
			begin
				if(out_read_finish) begin
					if(~ab2l1_WrAlmFull) begin
						Write_Fence		<= 1;
						Write_Res		<= 0;
						BWT_run			<= 0;
						BWT_state		<= BWT_STATE__WRITEFINISH; 
					end
				end
				else begin
					BWT_run			<= 1;
					Write_Res		<= 1;
					CNT_polling		<= 0;
					CNT_read		<= 0;
					BWT_polling		<= 0;
					CNT_done		<= 0;
					Write_Fence		<= 0;
				end
			end
			BWT_STATE__WRITEFINISH:
			begin
				BWT_run			<= 0;
				Write_Res		<= 0;
				CNT_polling		<= 0;
				CNT_read		<= 0;
				BWT_polling		<= 0;
				CNT_done		<= 0;
				Write_Fence		<= 0;
				Write_finish	<= 1;
				BWT_state		<= BWT_STATE__HANDFINISH;
			end
			BWT_STATE__HANDFINISH:
			begin
				BWT_run			<= 0;
				Write_Res		<= 0;
				CNT_polling		<= 0;
				CNT_read		<= 0;
				BWT_polling		<= 0;
				CNT_done		<= 0;
				Write_Fence		<= 0;
				Write_finish	<= 0;
				if(rx_wr_finish)
				begin
					pop_done	<= 1; //reseting all PEs (read)
				end

			end
			endcase
		end
	end

//////////////////////////////////////////////////////////////////
//					Read Request Management						//
//////////////////////////////////////////////////////////////////
  
    always @(posedge Clk_400)
    begin
		
		if(!test_Resetb)
		begin
			R0_DRAM_valid_hold	<= 0;
			Request_start		<= 0;
			PE_num				<= 0;
			korl <= 0;
			read_fsm           <= 0;
		end
		else if (!reset_read)begin
			R0_DRAM_valid_hold	<= 0;
			Request_start		<= 0;
			l12ab_RdEn			<= 0;
			korl <= 0;

		end
        case(read_fsm)
        2'h0:   
        begin  // start polling of the hand_ptr. Wait for re2xy_go
            l12ab_RdAddr        <=  hand_ptr;
            l12ab_RdTID			<=	16'h0;
			l12ab_RdEn			<=	1'b0;
            if(re2xy_go)
			begin
                read_fsm          <= 2'h1;
			end
        end
            
        2'h1:					// Send read requests
        begin  
			if(CNT_polling)
			begin
				l12ab_RdAddr    <=  hand_ptr;
				l12ab_RdTID		<=	{BWT_STATE__CNT_polling,6'b0};
				l12ab_RdEn		<=	1'b1; 
			end
			else if (CNT_read) 
			begin
				l12ab_RdAddr    <=  CNT_base + CNT_counter;
				l12ab_RdTID		<=  {BWT_STATE__CNT_read,2'b0,CNT_counter};//8'b0,3'b010,
				l12ab_RdEn		<=	1'b1;
			end
			else if(BWT_polling)
			begin
				l12ab_RdAddr    <=  hand_ptr;
				l12ab_RdTID		<=	{BWT_STATE__BWT_polling,6'b0};
				l12ab_RdEn		<=	1'b1; 
			end
			else if(BWT_run)
			begin
				if(cor_tx_rd_valid_a) begin					//send k request
					l12ab_RdEn		<=	1'b1;
					l12ab_RdTID		<=  {BWT_STATE__BWT_run,request_tag};
					case(request_tag[4:3])
						2'd0: l12ab_RdAddr <= cor_tx_rd_addr_a0;
						2'd1: l12ab_RdAddr <= cor_tx_rd_addr_a1;
						2'd2: l12ab_RdAddr <= cor_tx_rd_addr_a2;
						2'd3: l12ab_RdAddr <= cor_tx_rd_addr_a3;
					endcase
				end
				else if(cor_tx_rd_valid_b) begin			//send l request
					l12ab_RdEn		<=	1'b1;
					l12ab_RdTID		<=  {BWT_STATE__BWT_run,request_tag};
					case(request_tag[4:3])
						2'd0: l12ab_RdAddr <= cor_tx_rd_addr_b0;
						2'd1: l12ab_RdAddr <= cor_tx_rd_addr_b1;
						2'd2: l12ab_RdAddr <= cor_tx_rd_addr_b2;
						2'd3: l12ab_RdAddr <= cor_tx_rd_addr_b3;
					endcase
				end
				else
					l12ab_RdEn		<=	1'b0;
			end
			else
			begin
				l12ab_RdEn		<=	1'b0;
			end

			//assigning address to intermediate wire and then output
			if(~ab2l1_stallRd && Request_start && ~korl) //when it start to process// logic is not correct if for multiple PEs
			begin
				cor_tx_rd_valid_a	<= 1;
				cor_tx_rd_valid_b	<= 0;
				request_tag			<= {PE_num,1'b0};
			    case(PE_num)
					5'd0: cor_tx_rd_addr_a0  <= R0_CL_addr_k;
					5'd1: cor_tx_rd_addr_a0  <= R1_CL_addr_k; 
					5'd2: cor_tx_rd_addr_a0  <= R2_CL_addr_k; 
					5'd3: cor_tx_rd_addr_a0  <= R3_CL_addr_k; 
					5'd4: cor_tx_rd_addr_a1  <= R4_CL_addr_k; 
					5'd5: cor_tx_rd_addr_a1  <= R5_CL_addr_k; 
					5'd6: cor_tx_rd_addr_a1  <= R6_CL_addr_k; 
					5'd7: cor_tx_rd_addr_a1  <= R7_CL_addr_k; 
					5'd8: cor_tx_rd_addr_a2  <= R8_CL_addr_k; 
					5'd9: cor_tx_rd_addr_a2  <= R9_CL_addr_k; 
					5'd10: cor_tx_rd_addr_a2 <= R10_CL_addr_k;
					5'd11: cor_tx_rd_addr_a2 <= R11_CL_addr_k;
					5'd12: cor_tx_rd_addr_a3 <= R12_CL_addr_k;
					5'd13: cor_tx_rd_addr_a3 <= R13_CL_addr_k;
					5'd14: cor_tx_rd_addr_a3 <= R14_CL_addr_k;
					5'd15: cor_tx_rd_addr_a3 <= R15_CL_addr_k;
                endcase
				korl <= 1;
				
			end
			else if(korl)
			begin
				cor_tx_rd_valid_a	<= 0;
				cor_tx_rd_valid_b	<= 1;
				request_tag			<= {PE_num,1'b1};
				case(PE_num)
                    5'd0: cor_tx_rd_addr_b0  <= R0_CL_addr_l; 
                    5'd1: cor_tx_rd_addr_b0  <= R1_CL_addr_l; 
                    5'd2: cor_tx_rd_addr_b0  <= R2_CL_addr_l; 
                    5'd3: cor_tx_rd_addr_b0  <= R3_CL_addr_l; 
                    5'd4: cor_tx_rd_addr_b1  <= R4_CL_addr_l; 
                    5'd5: cor_tx_rd_addr_b1  <= R5_CL_addr_l; 
                    5'd6: cor_tx_rd_addr_b1  <= R6_CL_addr_l; 
                    5'd7: cor_tx_rd_addr_b1  <= R7_CL_addr_l; 
                    5'd8: cor_tx_rd_addr_b2  <= R8_CL_addr_l; 
                    5'd9: cor_tx_rd_addr_b2  <= R9_CL_addr_l; 
                    5'd10: cor_tx_rd_addr_b2 <= R10_CL_addr_l;
                    5'd11: cor_tx_rd_addr_b2 <= R11_CL_addr_l;
                    5'd12: cor_tx_rd_addr_b3 <= R12_CL_addr_l;
                    5'd13: cor_tx_rd_addr_b3 <= R13_CL_addr_l;
                    5'd14: cor_tx_rd_addr_b3 <= R14_CL_addr_l;
                    5'd15: cor_tx_rd_addr_b3 <= R15_CL_addr_l;
                endcase
				korl <= 0;
				Request_start <= 0;
			end
			else
			begin
				cor_tx_rd_valid_a	<= 0;
				cor_tx_rd_valid_b	<= 0;
				korl				<= 0;
			end 
        end
            
        default:              
		begin
			l12ab_RdEn		<=	1'b0; 
		end
        endcase

//////////////////////////////////////////////////////////////////////////	
//				Holding the request valid signal from PE				//
//////////////////////////////////////////////////////////////////////////

		//keeping R0_DRAM_valid signal
		if(R0_DRAM_valid)
		  R0_DRAM_valid_hold	<= 1;
      	if(R1_DRAM_valid)
      		R1_DRAM_valid_hold <= 1;
      	if(R2_DRAM_valid)
      		R2_DRAM_valid_hold <= 1;
      	if(R3_DRAM_valid)
      		R3_DRAM_valid_hold <= 1;
      	if(R4_DRAM_valid)
      		R4_DRAM_valid_hold <= 1;
      	if(R5_DRAM_valid)
      		R5_DRAM_valid_hold <= 1;
      	if(R6_DRAM_valid)
      		R6_DRAM_valid_hold <= 1;
      	if(R7_DRAM_valid)
      		R7_DRAM_valid_hold <= 1;
      	if(R8_DRAM_valid)
      		R8_DRAM_valid_hold <= 1;
      	if(R9_DRAM_valid)
      		R9_DRAM_valid_hold <= 1;
      	if(R10_DRAM_valid)
      		R10_DRAM_valid_hold <= 1;
      	if(R11_DRAM_valid)
      		R11_DRAM_valid_hold <= 1;
      	if(R12_DRAM_valid)
      		R12_DRAM_valid_hold <= 1;
      	if(R13_DRAM_valid)
      		R13_DRAM_valid_hold <= 1;
      	if(R14_DRAM_valid)
      		R14_DRAM_valid_hold <= 1;
      	if(R15_DRAM_valid)
      		R15_DRAM_valid_hold <= 1;


        if(Request_start == 0) begin	//noone is processing then
			if(R0_DRAM_valid_hold) begin		//with DRAM_valid, push the request to Interface with PE_num as tag
      		  R0_DRAM_valid_hold <= 0;
      		  Request_start <= 1;
      		  PE_num <= 0;
      		end
      		else if(R1_DRAM_valid_hold) begin
      			R1_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 1;
      		end
      		else if(R2_DRAM_valid_hold) begin
      			R2_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 2;
      		end
      		else if(R3_DRAM_valid_hold) begin
      			R3_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 3;
      		end
      		else if(R4_DRAM_valid_hold) begin
      			R4_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 4;
      		end
      		else if(R5_DRAM_valid_hold) begin
      			R5_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 5;
      		end
      		else if(R6_DRAM_valid_hold) begin
      			R6_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 6;
      		end
      		else if(R7_DRAM_valid_hold) begin
      			R7_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 7;
      		end
      		else if(R8_DRAM_valid_hold) begin
      			R8_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 8;
      		end
      		else if(R9_DRAM_valid_hold) begin
      			R9_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 9;
      		end
      		else if(R10_DRAM_valid_hold) begin
      			R10_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 10;
      		end
      		else if(R11_DRAM_valid_hold) begin
      			R11_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 11;
      		end
      		else if(R12_DRAM_valid_hold) begin
      			R12_DRAM_valid_hold <= 0;
      			Request_start <= 1;
        		PE_num <= 12;
      		end
      		else if(R13_DRAM_valid_hold) begin
      			R13_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 13;
      		end
      		else if(R14_DRAM_valid_hold) begin
      			R14_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 14;
      		end
      		else if(R15_DRAM_valid_hold) begin
      			R15_DRAM_valid_hold <= 0;
      			Request_start <= 1;
      			PE_num <= 15;
      		end
      		else begin
      		  Request_start <= 0;
      		end
		end
    end

///////////////////////////////////////////////////////////////////////////
//					assigning address to output port					//
//////////////////////////////////////////////////////////////////////////

	always @(posedge Clk_400) 
	begin												// whether it request read or bwt result
		PE_num_q	<= PE_num;	
		token_id_q	<= 	token_id;						// each request is 2 cache line, for k & l 
      if(R0_read_trigger) begin
        R0_CL_addr_k <= BWT_base0 + R0_addr_k[31:4]; //R0_CL_addr_k won't change until it has been processed!
        R0_CL_addr_l <= BWT_base0 + R0_addr_l[31:4];
      end
      else begin
      	R0_CL_addr_k <= input_base0 + R0_addr_k;
        R0_CL_addr_l <= input_base0 + R0_addr_l;
      end
	  if(R1_read_trigger) begin
        R1_CL_addr_k <= BWT_base0 + R1_addr_k[31:4];
        R1_CL_addr_l <= BWT_base0 + R1_addr_l[31:4];
      end
      else begin
      	R1_CL_addr_k <= input_base0 + R1_addr_k;
        R1_CL_addr_l <= input_base0 + R1_addr_l;
      end
      if(R2_read_trigger) begin
        R2_CL_addr_k <= BWT_base0 + R2_addr_k[31:4];
        R2_CL_addr_l <= BWT_base0 + R2_addr_l[31:4];
      end
      else begin
      	R2_CL_addr_k <= input_base0 + R2_addr_k;
        R2_CL_addr_l <= input_base0 + R2_addr_l;
      end
      if(R3_read_trigger) begin
        R3_CL_addr_k <= BWT_base0 + R3_addr_k[31:4];
        R3_CL_addr_l <= BWT_base0 + R3_addr_l[31:4];
      end
      else begin
      	R3_CL_addr_k <= input_base0 + R3_addr_k;
        R3_CL_addr_l <= input_base0 + R3_addr_l;
      end
      if(R4_read_trigger) begin
        R4_CL_addr_k <= BWT_base1 + R4_addr_k[31:4];
        R4_CL_addr_l <= BWT_base1 + R4_addr_l[31:4];
      end
      else begin
      	R4_CL_addr_k <= input_base1 + R4_addr_k;
        R4_CL_addr_l <= input_base1 + R4_addr_l;
      end
      if(R5_read_trigger) begin
        R5_CL_addr_k <= BWT_base1 + R5_addr_k[31:4];
        R5_CL_addr_l <= BWT_base1 + R5_addr_l[31:4];
      end
      else begin
      	R5_CL_addr_k <= input_base1 + R5_addr_k;
        R5_CL_addr_l <= input_base1 + R5_addr_l;
      end
      if(R6_read_trigger) begin
        R6_CL_addr_k <= BWT_base1 + R6_addr_k[31:4];
        R6_CL_addr_l <= BWT_base1 + R6_addr_l[31:4];
      end
      else begin
      	R6_CL_addr_k <= input_base1 + R6_addr_k;
        R6_CL_addr_l <= input_base1 + R6_addr_l;
      end
      if(R7_read_trigger) begin
        R7_CL_addr_k <= BWT_base1 + R7_addr_k[31:4];
        R7_CL_addr_l <= BWT_base1 + R7_addr_l[31:4];
      end
      else begin
      	R7_CL_addr_k <= input_base1 + R7_addr_k;
        R7_CL_addr_l <= input_base1 + R7_addr_l;
      end
      
      if(R8_read_trigger) begin
        R8_CL_addr_k <= BWT_base2 + R8_addr_k[31:4];
        R8_CL_addr_l <= BWT_base2 + R8_addr_l[31:4];
      end
      else begin
      	R8_CL_addr_k <= input_base2 + R8_addr_k;
        R8_CL_addr_l <= input_base2 + R8_addr_l;
      end
      if(R9_read_trigger) begin
        R9_CL_addr_k <= BWT_base2 + R9_addr_k[31:4];
        R9_CL_addr_l <= BWT_base2 + R9_addr_l[31:4];
      end
      else begin
      	R9_CL_addr_k <= input_base2 + R9_addr_k;
        R9_CL_addr_l <= input_base2 + R9_addr_l;
      end
      if(R10_read_trigger) begin
        R10_CL_addr_k <= BWT_base2 + R10_addr_k[31:4];
        R10_CL_addr_l <= BWT_base2 + R10_addr_l[31:4];
      end
      else begin
      	R10_CL_addr_k <= input_base2 + R10_addr_k;
        R10_CL_addr_l <= input_base2 + R10_addr_l;
      end
      if(R11_read_trigger) begin
        R11_CL_addr_k <= BWT_base2 + R11_addr_k[31:4];
        R11_CL_addr_l <= BWT_base2 + R11_addr_l[31:4];
      end
      else begin
      	R11_CL_addr_k <= input_base2 + R11_addr_k;
        R11_CL_addr_l <= input_base2 + R11_addr_l;
      end
      if(R12_read_trigger) begin
        R12_CL_addr_k <= BWT_base3 + R12_addr_k[31:4];
        R12_CL_addr_l <= BWT_base3 + R12_addr_l[31:4];
      end
      else begin
      	R12_CL_addr_k <= input_base3 + R12_addr_k;
        R12_CL_addr_l <= input_base3 + R12_addr_l;
      end
      if(R13_read_trigger) begin
        R13_CL_addr_k <= BWT_base3 + R13_addr_k[31:4]; //normal BWT data request
        R13_CL_addr_l <= BWT_base3 + R13_addr_l[31:4];
      end
      else begin
      	R13_CL_addr_k <= input_base3 + R13_addr_k;	//request for "read" 
        R13_CL_addr_l <= input_base3 + R13_addr_l;
      end
      if(R14_read_trigger) begin
        R14_CL_addr_k <= BWT_base3 + R14_addr_k[31:4];
        R14_CL_addr_l <= BWT_base3 + R14_addr_l[31:4];
      end
      else begin
      	R14_CL_addr_k <= input_base3 + R14_addr_k;
        R14_CL_addr_l <= input_base3 + R14_addr_l;
      end
      if(R15_read_trigger) begin
        R15_CL_addr_k <= BWT_base3 + R15_addr_k[31:4];
        R15_CL_addr_l <= BWT_base3 + R15_addr_l[31:4];
      end
      else begin
      	R15_CL_addr_k <= input_base3 + R15_addr_k;
        R15_CL_addr_l <= input_base3 + R15_addr_l;
      end

	end

//////////////////////////////////////////////////////////////////////
//								RX RD								//
//////////////////////////////////////////////////////////////////////	
	//managing the tag on the receive side
	always @(posedge Clk_400) 
	begin
		if(!test_Resetb)
		begin
			CNT_read_tag	<= 0;
			R0_BWT_input_valid	<= 0;
			PE0_start			<= 0;
			PE0_rec			<= 0;
			PE1_rec			<= 0;
			PE2_rec			<= 0;
			PE3_rec			<= 0;
			PE4_rec			<= 0;
			PE5_rec			<= 0;
			PE6_rec			<= 0;
			PE7_rec			<= 0;
			PE8_rec			<= 0;
			PE9_rec			<= 0;
			PE10_rec        <= 0;
			PE11_rec        <= 0;
			PE12_rec        <= 0;
			PE13_rec        <= 0;
			PE14_rec        <= 0;
			PE15_rec        <= 0;

	    end
		else if (!reset_read)begin
			PE0_rec			<= 0;
			PE1_rec			<= 0;
			PE2_rec			<= 0;
			PE3_rec			<= 0;
			PE4_rec			<= 0;
			PE5_rec			<= 0;
			PE6_rec			<= 0;
			PE7_rec			<= 0;
			PE8_rec			<= 0;
			PE9_rec			<= 0;
			PE10_rec        <= 0;
			PE11_rec        <= 0;
			PE12_rec        <= 0;
			PE13_rec        <= 0;
			PE14_rec        <= 0;
			PE15_rec        <= 0;
			err_Rx_BWT_run		<= 0;
			BWT_read_tag_0		<= 0;
			BWT_read_tag_1		<= 0;
			batch_size_temp		<= 16;
			BWT_get				<= 0;
			CNT_get				<= 0;
			CNT_polling_received	<= 0;
			R0_BWT_input_valid	<= 0;
			PE0_start			<= 0;
        end
		else if(ab2l1_RdRspValid)
		begin
			case(ab2l1_RdRsp[8:6])
			BWT_STATE__CNT_polling:
			begin
				CNT_read_tag			<= ab2l1_RdData[481];
				CNT_polling_received	<= 1;
			end
			BWT_STATE__CNT_read:
			begin
				CNT_data		<= ab2l1_RdData;
				CNT_get			<= 1;
			end
			BWT_STATE__BWT_polling:
			begin
				BWT_read_tag_0	<= ab2l1_RdData[480];
				BWT_read_tag_1	<= ab2l1_RdData[482];
				batch_size_temp <= ab2l1_RdData[457:448];
				BWT_get			<= 1;
			end
			BWT_STATE__BWT_run: 
			begin
				case(ab2l1_RdRsp[5:1])
				5'h0:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R0_BWT_input_dataL <= ab2l1_RdData;
						PE0_rec	<=	PE0_rec + 1;
					end
					else
					begin
						R0_BWT_input_dataK <= ab2l1_RdData;
						PE0_rec	<=	PE0_rec + 1;
					end
				end
				5'h1:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R1_BWT_input_dataL <= ab2l1_RdData;
						PE1_rec	<=	PE1_rec + 1;
					end
					else
					begin
						R1_BWT_input_dataK <= ab2l1_RdData;
						PE1_rec	<=	PE1_rec + 1;
					end
				end
				5'h2:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R2_BWT_input_dataL <= ab2l1_RdData;
						PE2_rec	<=	PE2_rec + 1;
					end
					else
					begin
						R2_BWT_input_dataK <= ab2l1_RdData;
						PE2_rec	<=	PE2_rec + 1;
					end
				end
				5'h3:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R3_BWT_input_dataL <= ab2l1_RdData;
						PE3_rec	<=	PE3_rec + 1;
					end
					else
					begin
						R3_BWT_input_dataK <= ab2l1_RdData;
						PE3_rec	<=	PE3_rec + 1;
					end
				end
				5'h4:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R4_BWT_input_dataL <= ab2l1_RdData;
						PE4_rec	<=	PE4_rec + 1;
					end
					else
					begin
						R4_BWT_input_dataK <= ab2l1_RdData;
						PE4_rec	<=	PE4_rec + 1;
					end
				end
				5'h5:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R5_BWT_input_dataL <= ab2l1_RdData;
						PE5_rec	<=	PE5_rec + 1;
					end
					else
					begin
						R5_BWT_input_dataK <= ab2l1_RdData;
						PE5_rec	<=	PE5_rec + 1;
					end
				end
				5'h6:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R6_BWT_input_dataL <= ab2l1_RdData;
						PE6_rec	<=	PE6_rec + 1;
					end
					else
					begin
						R6_BWT_input_dataK <= ab2l1_RdData;
						PE6_rec	<=	PE6_rec + 1;
					end
				end
				5'h7:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R7_BWT_input_dataL <= ab2l1_RdData;
						PE7_rec	<=	PE7_rec + 1;
					end
					else
					begin
						R7_BWT_input_dataK <= ab2l1_RdData;
						PE7_rec	<=	PE7_rec + 1;
					end
				end
				5'h8:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R8_BWT_input_dataL <= ab2l1_RdData;
						PE8_rec	<=	PE8_rec + 1;
					end
					else
					begin
						R8_BWT_input_dataK <= ab2l1_RdData;
						PE8_rec	<=	PE8_rec + 1;
					end
				end
				5'h9:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R9_BWT_input_dataL <= ab2l1_RdData;
						PE9_rec	<=	PE9_rec + 1;
					end
					else
					begin
						R9_BWT_input_dataK <= ab2l1_RdData;
						PE9_rec	<=	PE9_rec + 1;
					end
				end
				5'ha:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R10_BWT_input_dataL <= ab2l1_RdData;
						PE10_rec	<=	PE10_rec + 1;
					end
					else
					begin
						R10_BWT_input_dataK <= ab2l1_RdData;
						PE10_rec	<=	PE10_rec + 1;
					end
				end
				5'hb:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R11_BWT_input_dataL <= ab2l1_RdData;
						PE11_rec	<=	PE11_rec + 1;
					end
					else
					begin
						R11_BWT_input_dataK <= ab2l1_RdData;
						PE11_rec	<=	PE11_rec + 1;
					end
				end
				5'hc:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R12_BWT_input_dataL <= ab2l1_RdData;
						PE12_rec	<=	PE12_rec + 1;
					end
					else
					begin
						R12_BWT_input_dataK <= ab2l1_RdData;
						PE12_rec	<=	PE12_rec + 1;
					end
				end
				5'hd:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R13_BWT_input_dataL <= ab2l1_RdData;
						PE13_rec	<=	PE13_rec + 1;
					end
					else
					begin
						R13_BWT_input_dataK <= ab2l1_RdData;
						PE13_rec	<=	PE13_rec + 1;
					end
				end
				5'he:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R14_BWT_input_dataL <= ab2l1_RdData;
						PE14_rec	<=	PE14_rec + 1;
					end
					else
					begin
						R14_BWT_input_dataK <= ab2l1_RdData;
						PE14_rec	<=	PE14_rec + 1;
					end
				end
				5'hf:
				begin
					if(ab2l1_RdRsp[0]) begin  //0 is k, 1 is l
						R15_BWT_input_dataL <= ab2l1_RdData;
						PE15_rec	<=	PE15_rec + 1;
					end
					else
					begin
						R15_BWT_input_dataK <= ab2l1_RdData;
						PE15_rec	<=	PE15_rec + 1;
					end
				end
				default:
				begin
					err_Rx_BWT_run <= 1;
				end
				endcase
			end
			default:
			begin
			//report error
			end
			endcase
		end
		else begin
			CNT_polling_received	<= 0;
			CNT_get					<= 0;
			BWT_get					<= 0;
		end

//////////////////////////////////////////////////////////////////////////////
//			after getting full 2 CL, assign the read back to PE				//
//			the Wrapper always assumes the order of k and l.				//
//	also assmues that each PE only send 2 requests and wait for response	//
//////////////////////////////////////////////////////////////////////////////

		if(PE0_rec==2) begin
			PE0_start			<= 1;
			PE0_rec				<= 0;
			R0_BWT_input_valid	<= 1;
			R0_BWT_input		<= R0_BWT_input_dataK;
		end
		else if(PE0_start) begin
			R0_BWT_input_valid	<= 1;
			R0_BWT_input		<= R0_BWT_input_dataL;
			PE0_start			<= 0;
		end	
		else 
		begin
			R0_BWT_input_valid	<= 0;
			PE0_start			<= 0;
		end
		if (PE1_rec == 2) begin
			PE1_start <= 1;
			PE1_rec <= 0;
			R1_BWT_input_valid <= 1; 
			R1_BWT_input <= R1_BWT_input_dataK; 
		end
		else if (PE1_start) begin
			R1_BWT_input_valid <= 1; 
			R1_BWT_input <= R1_BWT_input_dataL; 
			PE1_start <= 0; 
		end
		else
		begin
			R1_BWT_input_valid <= 0; 
			PE1_start <= 0; 
		end
		if (PE2_rec == 2) begin
			PE2_start <= 1;
			PE2_rec <= 0;
			R2_BWT_input_valid <= 1; 
			R2_BWT_input <= R2_BWT_input_dataK; 
		end
		else if (PE2_start) begin
			R2_BWT_input_valid <= 1; 
			R2_BWT_input <= R2_BWT_input_dataL; 
			PE2_start <= 0; 
		end
		else
		begin
			R2_BWT_input_valid <= 0; 
			PE2_start <= 0; 
		end
		if (PE3_rec == 2) begin
			PE3_start <= 1;
			PE3_rec <= 0;
			R3_BWT_input_valid <= 1; 
			R3_BWT_input <= R3_BWT_input_dataK; 
		end
		else if (PE3_start) begin
			R3_BWT_input_valid <= 1; 
			R3_BWT_input <= R3_BWT_input_dataL; 
			PE3_start <= 0; 
		end
		else
		begin
		R3_BWT_input_valid <= 0; 
		PE3_start <= 0; 
		end
		if (PE4_rec == 2) begin
		PE4_start <= 1;
		PE4_rec <= 0;
		R4_BWT_input_valid <= 1; 
		R4_BWT_input <= R4_BWT_input_dataK; 
		end
		else if (PE4_start) begin
		R4_BWT_input_valid <= 1; 
		R4_BWT_input <= R4_BWT_input_dataL; 
		PE4_start <= 0; 
		end
		else
		begin
		R4_BWT_input_valid <= 0; 
		PE4_start <= 0; 
		end
		if (PE5_rec == 2) begin
		PE5_start <= 1;
		PE5_rec <= 0;
		R5_BWT_input_valid <= 1; 
		R5_BWT_input <= R5_BWT_input_dataK; 
		end
		else if (PE5_start) begin
		R5_BWT_input_valid <= 1; 
		R5_BWT_input <= R5_BWT_input_dataL; 
		PE5_start <= 0; 
		end
		else
		begin
		R5_BWT_input_valid <= 0; 
		PE5_start <= 0; 
		end
		if (PE6_rec == 2) begin
		PE6_start <= 1;
		PE6_rec <= 0;
		R6_BWT_input_valid <= 1; 
		R6_BWT_input <= R6_BWT_input_dataK; 
		end
		else if (PE6_start) begin
		R6_BWT_input_valid <= 1; 
		R6_BWT_input <= R6_BWT_input_dataL; 
		PE6_start <= 0; 
		end
		else
		begin
		R6_BWT_input_valid <= 0; 
		PE6_start <= 0; 
		end
		if (PE7_rec == 2) begin
		PE7_start <= 1;
		PE7_rec <= 0;
		R7_BWT_input_valid <= 1; 
		R7_BWT_input <= R7_BWT_input_dataK; 
		end
		else if (PE7_start) begin
		R7_BWT_input_valid <= 1; 
		R7_BWT_input <= R7_BWT_input_dataL; 
		PE7_start <= 0; 
		end
		else
		begin
		R7_BWT_input_valid <= 0; 
		PE7_start <= 0; 
		end
		if (PE8_rec == 2) begin
		PE8_start <= 1;
		PE8_rec <= 0;
		R8_BWT_input_valid <= 1; 
		R8_BWT_input <= R8_BWT_input_dataK; 
		end
		else if (PE8_start) begin
		R8_BWT_input_valid <= 1; 
		R8_BWT_input <= R8_BWT_input_dataL; 
		PE8_start <= 0; 
		end
		else
		begin
		R8_BWT_input_valid <= 0; 
		PE8_start <= 0; 
		end
		if (PE9_rec == 2) begin
		PE9_start <= 1;
		PE9_rec <= 0;
		R9_BWT_input_valid <= 1; 
		R9_BWT_input <= R9_BWT_input_dataK; 
		end
		else if (PE9_start) begin
		R9_BWT_input_valid <= 1; 
		R9_BWT_input <= R9_BWT_input_dataL; 
		PE9_start <= 0; 
		end
		else
		begin
		R9_BWT_input_valid <= 0; 
		PE9_start <= 0; 
		end
		if (PE10_rec == 2) begin
		PE10_start <= 1;
		PE10_rec <= 0;
		R10_BWT_input_valid <= 1; 
		R10_BWT_input <= R10_BWT_input_dataK; 
		end
		else if (PE10_start) begin
		R10_BWT_input_valid <= 1; 
		R10_BWT_input <= R10_BWT_input_dataL; 
		PE10_start <= 0; 
		end
		else
		begin
		R10_BWT_input_valid <= 0; 
		PE10_start <= 0; 
		end
		if (PE11_rec == 2) begin
		PE11_start <= 1;
		PE11_rec <= 0;
		R11_BWT_input_valid <= 1; 
		R11_BWT_input <= R11_BWT_input_dataK; 
		end
		else if (PE11_start) begin
		R11_BWT_input_valid <= 1; 
		R11_BWT_input <= R11_BWT_input_dataL; 
		PE11_start <= 0; 
		end
		else
		begin
		R11_BWT_input_valid <= 0; 
		PE11_start <= 0; 
		end
		if (PE12_rec == 2) begin
		PE12_start <= 1;
		PE12_rec <= 0;
		R12_BWT_input_valid <= 1; 
		R12_BWT_input <= R12_BWT_input_dataK; 
		end
		else if (PE12_start) begin
		R12_BWT_input_valid <= 1; 
		R12_BWT_input <= R12_BWT_input_dataL; 
		PE12_start <= 0; 
		end
		else
		begin
		R12_BWT_input_valid <= 0; 
		PE12_start <= 0; 
		end
		if (PE13_rec == 2) begin
		PE13_start <= 1;
		PE13_rec <= 0;
		R13_BWT_input_valid <= 1; 
		R13_BWT_input <= R13_BWT_input_dataK; 
		end
		else if (PE13_start) begin
		R13_BWT_input_valid <= 1; 
		R13_BWT_input <= R13_BWT_input_dataL; 
		PE13_start <= 0; 
		end
		else
		begin
		R13_BWT_input_valid <= 0; 
		PE13_start <= 0; 
		end
		if (PE14_rec == 2) begin
		PE14_start <= 1;
		PE14_rec <= 0;
		R14_BWT_input_valid <= 1; 
		R14_BWT_input <= R14_BWT_input_dataK; 
		end
		else if (PE14_start) begin
		R14_BWT_input_valid <= 1; 
		R14_BWT_input <= R14_BWT_input_dataL; 
		PE14_start <= 0; 
		end
		else
		begin
		R14_BWT_input_valid <= 0; 
		PE14_start <= 0; 
		end
		if (PE15_rec == 2) begin
		PE15_start <= 1;
		PE15_rec <= 0;
		R15_BWT_input_valid <= 1; 
		R15_BWT_input <= R15_BWT_input_dataK; 
		end
		else if (PE15_start) begin
		R15_BWT_input_valid <= 1; 
		R15_BWT_input <= R15_BWT_input_dataL; 
		PE15_start <= 0; 
		end
		else
		begin
		R15_BWT_input_valid <= 0; 
		PE15_start <= 0; 
		end

	end	

//////////////////////////////////////////////////////////////////
//						CNT data assignment						//
//					assigning CNT data to PEs					//
//////////////////////////////////////////////////////////////////

	always @(posedge Clk_400)  begin		// CNT table transmit, seperate wires
        if (!reset_read)begin
        	CNT_trigger			<= 0;
        	CNT_next_read		<= 0;
		    CNT_read_number		<= 0;
			CNT_flag1			<= 0;
			CNT_flag2			<= 1;
			CNT_in1				<= 0;
			CNT_addr1			<= 0;
			CNT_in2				<= 0;
			CNT_addr2			<= 0;
        end
        else begin
        	if(CNT_get)  begin			//CNT_get == 1 and start assigning
        		CNT_trigger		<= 1;
        		CNT_next_read	<= 0;
        	end
        	else if(CNT_read_number==7) begin
        		CNT_trigger		<= 0;
        		CNT_next_read	<= 1;	//indicating processed one input, asking for a second
        	end
        	else begin
        		CNT_trigger		<= CNT_trigger;
        		CNT_next_read	<= 0;
        	end 

        	if(CNT_trigger) begin
        		CNT_read_number <= CNT_read_number + 1;
        		CNT_flag1		<= 1;
				CNT_flag2		<= 1;
        		case(CNT_read_number)				//
        		3'd0: begin
        			CNT_in1   <= CNT_data[63:0];
        			CNT_addr1 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        			CNT_in2   <= CNT_data[63:0];
        			CNT_addr2 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        		end 
        		3'd1: begin
        			CNT_in1   <= CNT_data[127:64];
        			CNT_addr1 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        			CNT_in2   <= CNT_data[127:64];
        			CNT_addr2 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        		end 
        		3'd2: begin
        			CNT_in1   <= CNT_data[191:128];
        			CNT_addr1 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        			CNT_in2   <= CNT_data[191:128];
        			CNT_addr2 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        		end 
        		3'd3: begin
        			CNT_in1   <= CNT_data[255:192];
        			CNT_addr1 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        			CNT_in2   <= CNT_data[255:192];
        			CNT_addr2 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        		end 
        		3'd4: begin
        			CNT_in1   <= CNT_data[319:256];
        			CNT_addr1 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        			CNT_in2   <= CNT_data[319:256];
        			CNT_addr2 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        		end 
        		3'd5: begin
        			CNT_in1   <= CNT_data[383:320];
        			CNT_addr1 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        			CNT_in2   <= CNT_data[383:320];
        			CNT_addr2 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        		end 
        		3'd6: begin
        			CNT_in1   <= CNT_data[447:384];
        			CNT_addr1 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        			CNT_in2   <= CNT_data[447:384];
        			CNT_addr2 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        		end 
        		3'd7: begin
        			CNT_in1   <= CNT_data[511:448];
        			CNT_addr1 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        			CNT_in2   <= CNT_data[511:448];
        			CNT_addr2 <= {CNT_counter[3:0], CNT_read_number, 1'b0};
        		end 
        		endcase
        	end
        	else begin
        		CNT_flag1	<= 0;
				CNT_flag2	<= 1;
        		CNT_read_number <= 0;
        		CNT_in1 <= CNT_in1;
        		CNT_addr1 <= CNT_addr1;
			    CNT_in2 <= CNT_in2;
        		CNT_addr2 <= CNT_addr2;
        	end
        end
    end


	 
//////////////////////////////////////////////////////////////////
//					TX WR CONTROL								//
////////////////////////////////////////////////////////////////// 

	always @(posedge Clk_400)
    begin
      if (!reset_read)
      begin
		l12ab_WrTID <= 0;
		l12ab_WrDin <= 0;
		l12ab_WrEn	<= 0;
		l12ab_WrAddr<= 0;
		l12ab_WrFence	<= 0;
	  end
	  else
	  begin
		if(CNT_done)
		begin
			l12ab_WrEn			<= 1'b1;
			l12ab_WrAddr		<= hand_ptr;
			l12ab_WrDin[479:0]	<= 0;
			l12ab_WrDin[511:480]<= 8;
			l12ab_WrTID			<= 0;
		end
		else if(Write_Res) begin
			case(token_id_q[3:2])
            2'd0: begin
				l12ab_WrDin		<= cor_tx_data_0;
				l12ab_WrEn		<= cor_tx_wr_valid_0;
				l12ab_WrAddr	<= cor_tx_wr_addr_0;
				l12ab_WrTID		<= 16;
			end
			2'd1: begin
				l12ab_WrDin		<= cor_tx_data_1;
				l12ab_WrEn		<= cor_tx_wr_valid_1;
				l12ab_WrAddr	<= cor_tx_wr_addr_1;
				l12ab_WrTID		<= 32;
			end
			2'd2: begin
				l12ab_WrDin		<= cor_tx_data_2;
				l12ab_WrEn		<= cor_tx_wr_valid_2;
				l12ab_WrAddr	<= cor_tx_wr_addr_2;
				l12ab_WrTID		<= 64;
			end
			2'd3: begin
				l12ab_WrDin		<= cor_tx_data_3;
				l12ab_WrEn		<= cor_tx_wr_valid_3;
				l12ab_WrAddr	<= cor_tx_wr_addr_3;
				l12ab_WrTID		<= 96;
			end
			endcase
		end
		else if(Write_Fence)
		begin
			l12ab_WrFence <= 1;
			l12ab_WrEn	<=1;
		end
		else if(Write_finish)
		begin
			l12ab_WrFence		<= 0;
			
			l12ab_WrEn			<= 1'b1;
			l12ab_WrAddr		<= hand_ptr;
			l12ab_WrDin[511:480] <= 16; //all finish
			l12ab_WrDin[479:0]	<= 0;
			l12ab_WrTID[15:0]	<= 7;
		end
		else
		begin
			l12ab_WrEn <= 1'b0;
		end
	  end    
	end
//////////////////////////////////////////////////////////////////////////////////////
//				receive write finish to control for finish signal					//
//////////////////////////////////////////////////////////////////////////////////////
	
	always @(posedge Clk_400)
    begin
		if (!reset_read)
		begin
			rx_wr_finish <=0;
		end
		if(ab2l1_WrRspValid) begin
			if(ab2l1_WrRsp == 7)begin
				rx_wr_finish <= 1;
			end	
			else begin
				rx_wr_finish <= 0;
			end
		end 
	end
//////////////////////////////////////////////////////////////////////////////////////
//					managing outputing result										//
//////////////////////////////////////////////////////////////////////////////////////

	always @(posedge Clk_400)
	begin
		if(!test_Resetb)
		begin
			out_read_num	<= 0;
			pop_hold			<= 0;
			token_id		<= 0;
		end
	    else if(!reset_read) begin
            R0_match_pop_pulse	<= 0;
			output_addr			<= output_base;
			pop_hold			<= 0;
			cor_tx_wr_addr_0	<= 0;
			cor_tx_wr_valid_0	<= 0;
			P_out_flag			<= 0;
			P_out_count			<= 0;
			readout				<= 0;
			out_read_num	<= 0;	
		end
		if(out_read_num == batch_size) begin
			out_read_finish	<= 1;
		end
		else begin
			out_read_finish	<= 0;
		end

		  
		if(BWT_state == BWT_STATE__BWT_polling)
			output_addr			<= output_base;
		else if(BWT_state == BWT_STATE__BWT_run)
		begin
			cor_tx_wr_valid_0 <= 1'b0;
			cor_tx_wr_valid_1 <= 1'b0;
			cor_tx_wr_valid_2 <= 1'b0;
			cor_tx_wr_valid_3 <= 1'b0;             
			case(token_id)
            5'd0: begin
				if(R0_read_done_hold) begin
				R0_match_pop_pulse <= 1;
				readout <= 1;
				R0_read_done_hold <= 0;
				end
				if(readout) begin	//the long as one R0_read_done_hold comes, write all the things to output.addr. data format
				cor_tx_wr_valid_0 <= 1'b1;
				cor_tx_wr_addr_0  <= output_addr;
				output_addr     <= output_addr + 1;
				cor_tx_data_0[9:0]     <= R0_read_num;
				cor_tx_data_0[63:10]   <= 0;
				cor_tx_data_0[70:64]   <= R0_mem_size;
				cor_tx_data_0[127:71]  <= 0;
				cor_tx_data_0[159:128] <= R0_ret;
				cor_tx_data_0[511:160] <= 0;
				P_out_count <= 0;
				P_out_flag <= 1;
				readout <= 0;
				end
				else if(P_out_flag) begin
					if(P_out_count < R0_mem_size) begin
						P_out_flag <= 1;
						if(P_out_count[0]==0) begin
                    		cor_tx_data_0[255:0] <= {R0_mem_out_info,R0_mem_out_x2,R0_mem_out_x1,R0_mem_out_x0}; //increased payload size
                    		if(P_out_count == (R0_mem_size-1)) begin
                    			R0_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count		  <= P_out_count + 1;
                    			cor_tx_wr_valid_0 <= 1'b1;
                    			cor_tx_wr_addr_0  <= output_addr;
                    			output_addr       <= output_addr + 1;
                    			cor_tx_data_0[511:256] <= 0;
                    			end
                    			else begin
                    			P_out_count <= P_out_count;
								cor_tx_wr_valid_0 <= 1'b0;
								end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R0_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_0 <= 1'b0;
                    		end
						end
						else begin
                    		if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
							R0_match_pop_pulse <= 1;
							cor_tx_wr_valid_0 <= 1'b1;
                    		cor_tx_wr_addr_0  <= output_addr;
                    		output_addr     <= output_addr + 1;
							cor_tx_data_0[255:0] <= cor_tx_data_0[255:0];
							cor_tx_data_0[511:256] <= {R0_mem_out_info,R0_mem_out_x2,R0_mem_out_x1,R0_mem_out_x0};//increased payload size
							end
							else begin
							P_out_count <= P_out_count;
							R0_match_pop_pulse <= 1'b0;
							cor_tx_wr_valid_0 <= 1'b0;
							end
						end
					end //P_out_count < R0_mem_size case
					else if(P_out_count==R0_mem_size) begin
						R0_match_pop_pulse <= 0;
						R0_read_done_hold <= 0;
						pop_hold <= 0;
						P_out_flag <= 0;				//is this P_out_flag possibly stuck there??????????
						P_out_count <= 0;
						cor_tx_wr_valid_0 <= 1'b0;
						readout <= 0;
						out_read_num <= out_read_num + 1;
						R0_pop_done <= 1;
					end
				end
			end
            5'd1: begin
            if(R1_read_done_hold) begin
                R1_match_pop_pulse <= 1;
                readout <= 1;
                R1_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_0 <= 1'b1;
                cor_tx_wr_addr_0  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_0[9:0]     <= R1_read_num;
                cor_tx_data_0[63:10]   <= 0;
                cor_tx_data_0[70:64]   <= R1_mem_size;
                cor_tx_data_0[127:71]  <= 0;
                cor_tx_data_0[159:128] <= R1_ret;
                cor_tx_data_0[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R1_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_0[255:0] <= {R1_mem_out_info,R1_mem_out_x2,R1_mem_out_x1,R1_mem_out_x0};
                    		if(P_out_count == (R1_mem_size-1)) begin
                    			R1_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_0 <= 1'b1;
                    			cor_tx_wr_addr_0  <= output_addr;
                    			output_addr     <= output_addr + 1;
                    			cor_tx_data_0[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
									cor_tx_wr_valid_0 <= 1'b0;
								end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R1_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_0 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R1_match_pop_pulse <= 1;
                    cor_tx_wr_valid_0 <= 1'b1;
                    	cor_tx_wr_addr_0 <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_0[255:0] <= cor_tx_data_0[255:0];
                    cor_tx_data_0[511:256] <= {R1_mem_out_info,R1_mem_out_x2,R1_mem_out_x1,R1_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R1_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_0 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R1_mem_size) begin
                    	R1_match_pop_pulse <= 0;
                    	R1_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_0 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R1_pop_done <= 1;
                    end
                end
            end                    			
            5'd2: begin
            if(R2_read_done_hold) begin
                R2_match_pop_pulse <= 1;
                readout <= 1;
                R2_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_0 <= 1'b1;
                cor_tx_wr_addr_0  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_0[9:0]     <= R2_read_num;
                cor_tx_data_0[63:10]   <= 0;
                cor_tx_data_0[70:64]   <= R2_mem_size;
                cor_tx_data_0[127:71]  <= 0;
                cor_tx_data_0[159:128] <= R2_ret;
                cor_tx_data_0[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R2_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_0[255:0] <= {R2_mem_out_info,R2_mem_out_x2,R2_mem_out_x1,R2_mem_out_x0};
                    		if(P_out_count == (R2_mem_size-1)) begin
                    			R2_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    				P_out_count <= P_out_count + 1;
                    				cor_tx_wr_valid_0 <= 1'b1;
                    				cor_tx_wr_addr_0  <= output_addr;
                    				output_addr     <= output_addr + 1;
                    				cor_tx_data_0[511:256] <= 0;
                    			end
                    			else begin
                    			P_out_count <= P_out_count;
								cor_tx_wr_valid_0 <= 1'b0;
								end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R2_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_0 <= 1'b0;
                    		end
                    	end
						else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
							R2_match_pop_pulse <= 1;
							cor_tx_wr_valid_0 <= 1'b1;
                    		cor_tx_wr_addr_0  <= output_addr;
                    		output_addr     <= output_addr + 1;
							cor_tx_data_0[255:0] <= cor_tx_data_0[255:0];
							cor_tx_data_0[511:256] <= {R2_mem_out_info,R2_mem_out_x2,R2_mem_out_x1,R2_mem_out_x0};
                    end
                    else begin
						P_out_count <= P_out_count;
						R1_match_pop_pulse <= 1'b0;
						cor_tx_wr_valid_0 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R2_mem_size) begin
                    	R2_match_pop_pulse <= 0;
                    	R2_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_0 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R2_pop_done <= 1;
                    end
                end
            end   
            5'd3: begin
            if(R3_read_done_hold) begin
                R3_match_pop_pulse <= 1;
                readout <= 1;
                R3_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_0 <= 1'b1;
                cor_tx_wr_addr_0  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_0[9:0]     <= R3_read_num;
                cor_tx_data_0[63:10]   <= 0;
                cor_tx_data_0[70:64]   <= R3_mem_size;
                cor_tx_data_0[127:71]  <= 0;
                cor_tx_data_0[159:128] <= R3_ret;
                cor_tx_data_0[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R3_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_0[255:0] <= {R3_mem_out_info,R3_mem_out_x2,R3_mem_out_x1,R3_mem_out_x0};
                    		if(P_out_count == (R3_mem_size-1)) begin
                    			R3_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_0 <= 1'b1;
                    	cor_tx_wr_addr_0  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_0[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
									cor_tx_wr_valid_0 <= 1'b0;
								end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R3_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_0 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R3_match_pop_pulse <= 1;
                    cor_tx_wr_valid_0 <= 1'b1;
                    	cor_tx_wr_addr_0  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_0[255:0] <= cor_tx_data_0[255:0];
                    cor_tx_data_0[511:256] <= {R3_mem_out_info,R3_mem_out_x2,R3_mem_out_x1,R3_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R3_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_0 <= 1'b0;
                    end
                    end
                end 
                    else if(P_out_count==R3_mem_size) begin
                    	R3_match_pop_pulse <= 0;
                    	R3_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_0 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R3_pop_done <= 1;
                    end
                end
            end
            5'd4: begin
            if(R4_read_done_hold) begin
                R4_match_pop_pulse <= 1;
                readout <= 1;
                R4_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_1 <= 1'b1;
                cor_tx_wr_addr_1  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_1[9:0]     <= R4_read_num;
                cor_tx_data_1[63:10]   <= 0;
                cor_tx_data_1[70:64]   <= R4_mem_size;
                cor_tx_data_1[127:71]  <= 0;
                cor_tx_data_1[159:128] <= R4_ret;
                cor_tx_data_1[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R4_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_1[255:0] <= {R4_mem_out_info,R4_mem_out_x2,R4_mem_out_x1,R4_mem_out_x0};
                    		if(P_out_count == (R4_mem_size-1)) begin
                    			R4_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_1 <= 1'b1;
                    	cor_tx_wr_addr_1  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_1[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_1 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R4_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_1 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R4_match_pop_pulse <= 1;
                    cor_tx_wr_valid_1 <= 1'b1;
                    	cor_tx_wr_addr_1  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_1[255:0] <= cor_tx_data_1[255:0];
                    cor_tx_data_1[511:256] <= {R4_mem_out_info,R4_mem_out_x2,R4_mem_out_x1,R4_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R4_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_1 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R4_mem_size) begin
                    	R4_match_pop_pulse <= 0;
                    	R4_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_1 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R4_pop_done <= 1;
                    end
                end
            end
            5'd5: begin
            if(R5_read_done_hold) begin
                R5_match_pop_pulse <= 1;
                readout <= 1;
                R5_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_1 <= 1'b1;
                cor_tx_wr_addr_1  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_1[9:0]     <= R5_read_num;
                cor_tx_data_1[63:10]   <= 0;
                cor_tx_data_1[70:64]   <= R5_mem_size;
                cor_tx_data_1[127:71]  <= 0;
                cor_tx_data_1[159:128] <= R5_ret;
                cor_tx_data_1[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R5_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_1[255:0] <= {R5_mem_out_info,R5_mem_out_x2,R5_mem_out_x1,R5_mem_out_x0};
                    		if(P_out_count == (R5_mem_size-1)) begin
                    			R5_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_1 <= 1'b1;
                    	cor_tx_wr_addr_1  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_1[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_1 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R5_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_1 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R5_match_pop_pulse <= 1;
                    cor_tx_wr_valid_1 <= 1'b1;
                    	cor_tx_wr_addr_1  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_1[255:0] <= cor_tx_data_1[255:0];
                    cor_tx_data_1[511:256] <= {R5_mem_out_info,R5_mem_out_x2,R5_mem_out_x1,R5_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R5_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_1 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R5_mem_size) begin
                    	R5_match_pop_pulse <= 0;
                    	R5_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_1 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R5_pop_done <= 1;
                    end
                end
            end
            5'd6: begin
            if(R6_read_done_hold) begin
                R6_match_pop_pulse <= 1;
                readout <= 1;
                R6_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_1 <= 1'b1;
                cor_tx_wr_addr_1  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_1[9:0]     <= R6_read_num;
                cor_tx_data_1[63:10]   <= 0;
                cor_tx_data_1[70:64]   <= R6_mem_size;
                cor_tx_data_1[127:71]  <= 0;
                cor_tx_data_1[159:128] <= R6_ret;
                cor_tx_data_1[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R6_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_1[255:0] <= {R6_mem_out_info,R6_mem_out_x2,R6_mem_out_x1,R6_mem_out_x0};
                    		if(P_out_count == (R6_mem_size-1)) begin
                    			R6_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_1 <= 1'b1;
                    	cor_tx_wr_addr_1  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_1[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_1 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R6_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_1 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R6_match_pop_pulse <= 1;
                    cor_tx_wr_valid_1 <= 1'b1;
                    	cor_tx_wr_addr_1  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_1[255:0] <= cor_tx_data_1[255:0];
                    cor_tx_data_1[511:256] <= {R6_mem_out_info,R6_mem_out_x2,R6_mem_out_x1,R6_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R6_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_1 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R6_mem_size) begin
                    	R6_match_pop_pulse <= 0;
                    	R6_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_1 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R6_pop_done <= 1;
                    end
                end
            end
            5'd7: begin
            if(R7_read_done_hold) begin
                R7_match_pop_pulse <= 1;
                readout <= 1;
                R7_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_1 <= 1'b1;
                cor_tx_wr_addr_1  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_1[9:0]     <= R7_read_num;
                cor_tx_data_1[63:10]   <= 0;
                cor_tx_data_1[70:64]   <= R7_mem_size;
                cor_tx_data_1[127:71]  <= 0;
                cor_tx_data_1[159:128] <= R7_ret;
                cor_tx_data_1[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R7_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_1[255:0] <= {R7_mem_out_info,R7_mem_out_x2,R7_mem_out_x1,R7_mem_out_x0};
                    		if(P_out_count == (R7_mem_size-1)) begin
                    			R7_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_1 <= 1'b1;
                    	cor_tx_wr_addr_1  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_1[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_1 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R7_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_1 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R7_match_pop_pulse <= 1;
                    cor_tx_wr_valid_1 <= 1'b1;
                    	cor_tx_wr_addr_1  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_1[255:0] <= cor_tx_data_1[255:0];
                    cor_tx_data_1[511:256] <= {R7_mem_out_info,R7_mem_out_x2,R7_mem_out_x1,R7_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R7_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_1 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R7_mem_size) begin
                    	R7_match_pop_pulse <= 0;
                    	R7_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_1 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R7_pop_done <= 1;
                    end
                end
            end
            5'd8: begin
            if(R8_read_done_hold) begin
                R8_match_pop_pulse <= 1;
                readout <= 1;
                R8_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_2 <= 1'b1;
                cor_tx_wr_addr_2  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_2[9:0]     <= R8_read_num;
                cor_tx_data_2[63:10]   <= 0;
                cor_tx_data_2[70:64]   <= R8_mem_size;
                cor_tx_data_2[127:71]  <= 0;
                cor_tx_data_2[159:128] <= R8_ret;
                cor_tx_data_2[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R8_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_2[255:0] <= {R8_mem_out_info,R8_mem_out_x2,R8_mem_out_x1,R8_mem_out_x0};
                    		if(P_out_count == (R8_mem_size-1)) begin
                    			R8_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_2 <= 1'b1;
                    	cor_tx_wr_addr_2  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_2[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_2 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R8_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_2 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R8_match_pop_pulse <= 1;
                    cor_tx_wr_valid_2 <= 1'b1;
                    	cor_tx_wr_addr_2  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_2[255:0] <= cor_tx_data_2[255:0];
                    cor_tx_data_2[511:256] <= {R8_mem_out_info,R8_mem_out_x2,R8_mem_out_x1,R8_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R8_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_2 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R8_mem_size) begin
                    	R8_match_pop_pulse <= 0;
                    	R8_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_2 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R8_pop_done <= 1;
                    end
                end
            end
            5'd9: begin
            if(R9_read_done_hold) begin
                R9_match_pop_pulse <= 1;
                readout <= 1;
                R9_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_2 <= 1'b1;
                cor_tx_wr_addr_2  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_2[9:0]     <= R9_read_num;
                cor_tx_data_2[63:10]   <= 0;
                cor_tx_data_2[70:64]   <= R9_mem_size;
                cor_tx_data_2[127:71]  <= 0;
                cor_tx_data_2[159:128] <= R9_ret;
                cor_tx_data_2[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R9_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_2[255:0] <= {R9_mem_out_info,R9_mem_out_x2,R9_mem_out_x1,R9_mem_out_x0};
                    		if(P_out_count == (R9_mem_size-1)) begin
                    			R9_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_2 <= 1'b1;
                    	cor_tx_wr_addr_2  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_2[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_2 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R9_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_2 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R9_match_pop_pulse <= 1;
                    cor_tx_wr_valid_2 <= 1'b1;
                    	cor_tx_wr_addr_2  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_2[255:0] <= cor_tx_data_2[255:0];
                    cor_tx_data_2[511:256] <= {R9_mem_out_info,R9_mem_out_x2,R9_mem_out_x1,R9_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R9_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_2 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R9_mem_size) begin
                    	R9_match_pop_pulse <= 0;
                    	R9_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_2 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R9_pop_done <= 1;
                    end
                end
            end
            5'd10: begin
            if(R10_read_done_hold) begin
                R10_match_pop_pulse <= 1;
                readout <= 1;
                R10_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_2 <= 1'b1;
                cor_tx_wr_addr_2  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_2[9:0]     <= R10_read_num;
                cor_tx_data_2[63:10]   <= 0;
                cor_tx_data_2[70:64]   <= R10_mem_size;
                cor_tx_data_2[127:71]  <= 0;
                cor_tx_data_2[159:128] <= R10_ret;
                cor_tx_data_2[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R10_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_2[255:0] <= {R10_mem_out_info,R10_mem_out_x2,R10_mem_out_x1,R10_mem_out_x0};
                    		if(P_out_count == (R10_mem_size-1)) begin
                    			R10_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_2 <= 1'b1;
                    	cor_tx_wr_addr_2  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_2[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_2 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R10_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_2 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R10_match_pop_pulse <= 1;
                    cor_tx_wr_valid_2 <= 1'b1;
                    	cor_tx_wr_addr_2  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_2[255:0] <= cor_tx_data_2[255:0];
                    cor_tx_data_2[511:256] <= {R10_mem_out_info,R10_mem_out_x2,R10_mem_out_x1,R10_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R10_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_2 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R10_mem_size) begin
                    	R10_match_pop_pulse <= 0;
                    	R10_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_2 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R10_pop_done <= 1;
                    end
                end
            end
            5'd11: begin
            if(R11_read_done_hold) begin
                R11_match_pop_pulse <= 1;
                readout <= 1;
                R11_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_2 <= 1'b1;
                cor_tx_wr_addr_2  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_2[9:0]     <= R11_read_num;
                cor_tx_data_2[63:10]   <= 0;
                cor_tx_data_2[70:64]   <= R11_mem_size;
                cor_tx_data_2[127:71]  <= 0;
                cor_tx_data_2[159:128] <= R11_ret;
                cor_tx_data_2[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R11_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_2[255:0] <= {R11_mem_out_info,R11_mem_out_x2,R11_mem_out_x1,R11_mem_out_x0};
                    		if(P_out_count == (R11_mem_size-1)) begin
                    			R11_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_2 <= 1'b1;
                    	cor_tx_wr_addr_2  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_2[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_2 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R11_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_2 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R11_match_pop_pulse <= 1;
                    cor_tx_wr_valid_2 <= 1'b1;
                    	cor_tx_wr_addr_2  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_2[255:0] <= cor_tx_data_2[255:0];
                    cor_tx_data_2[511:256] <= {R11_mem_out_info,R11_mem_out_x2,R11_mem_out_x1,R11_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R11_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_2 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R11_mem_size) begin
                    	R11_match_pop_pulse <= 0;
                    	R11_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_2 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R11_pop_done <= 1;
                    end
                end
            end
            5'd12: begin
            if(R12_read_done_hold) begin
                R12_match_pop_pulse <= 1;
                readout <= 1;
                R12_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_3 <= 1'b1;
                cor_tx_wr_addr_3  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_3[9:0]     <= R12_read_num;
                cor_tx_data_3[63:10]   <= 0;
                cor_tx_data_3[70:64]   <= R12_mem_size;
                cor_tx_data_3[127:71]  <= 0;
                cor_tx_data_3[159:128] <= R12_ret;
                cor_tx_data_3[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
            end
            else if(P_out_flag) begin
                    if(P_out_count < R12_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_3[255:0] <= {R12_mem_out_info,R12_mem_out_x2,R12_mem_out_x1,R12_mem_out_x0};
                    		if(P_out_count == (R12_mem_size-1)) begin
                    			R12_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_3 <= 1'b1;
                    			cor_tx_wr_addr_3  <= output_addr;
                    			output_addr     <= output_addr + 1;
                    			cor_tx_data_3[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
									cor_tx_wr_valid_3 <= 1'b0;
								end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R12_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_3 <= 1'b0;
                    		end
                    	end
						else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
							R12_match_pop_pulse <= 1;
							cor_tx_wr_valid_3 <= 1'b1;
                    		cor_tx_wr_addr_3  <= output_addr;
                    		output_addr     <= output_addr + 1;
							cor_tx_data_3[255:0] <= cor_tx_data_3[255:0];
							cor_tx_data_3[511:256] <= {R12_mem_out_info,R12_mem_out_x2,R12_mem_out_x1,R12_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R12_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_3 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R12_mem_size) begin
                    	R12_match_pop_pulse <= 0;
                    	R12_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_3 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R12_pop_done <= 1;
                    end
                end
            end
            5'd13: begin
            if(R13_read_done_hold) begin
                R13_match_pop_pulse <= 1;
                readout <= 1;
                R13_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_3 <= 1'b1;
                cor_tx_wr_addr_3  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_3[9:0]     <= R13_read_num;
                cor_tx_data_3[63:10]   <= 0;
                cor_tx_data_3[70:64]   <= R13_mem_size;
                cor_tx_data_3[127:71]  <= 0;
                cor_tx_data_3[159:128] <= R13_ret;
                cor_tx_data_3[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R13_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_3[255:0] <= {R13_mem_out_info,R13_mem_out_x2,R13_mem_out_x1,R13_mem_out_x0};
                    		if(P_out_count == (R13_mem_size-1)) begin
                    			R13_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_3 <= 1'b1;
                    	cor_tx_wr_addr_3  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_3[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_3 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R13_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_3 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R13_match_pop_pulse <= 1;
                    cor_tx_wr_valid_3 <= 1'b1;
                    	cor_tx_wr_addr_3  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_3[255:0] <= cor_tx_data_3[255:0];
                    cor_tx_data_3[511:256] <= {R13_mem_out_info,R13_mem_out_x2,R13_mem_out_x1,R13_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R13_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_3 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R13_mem_size) begin
                    	R13_match_pop_pulse <= 0;
                    	R13_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_3 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R13_pop_done <= 1;
                    end
                end
            end
            5'd14: begin
            if(R14_read_done_hold) begin
                R14_match_pop_pulse <= 1;
                readout <= 1;
                R14_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_3 <= 1'b1;
                cor_tx_wr_addr_3  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_3[9:0]     <= R14_read_num;
                cor_tx_data_3[63:10]   <= 0;
                cor_tx_data_3[70:64]   <= R14_mem_size;
                cor_tx_data_3[127:71]  <= 0;
                cor_tx_data_3[159:128] <= R14_ret;
                cor_tx_data_3[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R14_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_3[255:0] <= {R14_mem_out_info,R14_mem_out_x2,R14_mem_out_x1,R14_mem_out_x0};
                    		if(P_out_count == (R14_mem_size-1)) begin
                    			R14_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_3 <= 1'b1;
                    	cor_tx_wr_addr_3  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_3[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_3 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R14_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_3 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R14_match_pop_pulse <= 1;
                    cor_tx_wr_valid_3 <= 1'b1;
                    	cor_tx_wr_addr_3  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_3[255:0] <= cor_tx_data_3[255:0];
                    cor_tx_data_3[511:256] <= {R14_mem_out_info,R14_mem_out_x2,R14_mem_out_x1,R14_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R14_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_3 <= 1'b0;
                    end
                    end
                end
                    else if(P_out_count==R14_mem_size) begin
                    	R14_match_pop_pulse <= 0;
                    	R14_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_3 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R14_pop_done <= 1;
                    end
                end
            end
            5'd15: begin
            if(R15_read_done_hold) begin
                R15_match_pop_pulse <= 1;
                readout <= 1;
                R15_read_done_hold <= 0;
            end
            if(readout) begin
                cor_tx_wr_valid_3 <= 1'b1;
                cor_tx_wr_addr_3  <= output_addr;
                output_addr     <= output_addr + 1;
                cor_tx_data_3[9:0]     <= R15_read_num;
                cor_tx_data_3[63:10]   <= 0;
                cor_tx_data_3[70:64]   <= R15_mem_size;
                cor_tx_data_3[127:71]  <= 0;
                cor_tx_data_3[159:128] <= R15_ret;
                cor_tx_data_3[511:160] <= 0;
                P_out_count <= 0;
                P_out_flag <= 1;
                readout <= 0;
                end
                else if(P_out_flag) begin
                    if(P_out_count < R15_mem_size) begin
                    	P_out_flag <= 1;
                    	if(P_out_count[0]==0) begin
                    		cor_tx_data_3[255:0] <= {R15_mem_out_info,R15_mem_out_x2,R15_mem_out_x1,R15_mem_out_x0};
                    		if(P_out_count == (R15_mem_size-1)) begin
                    			R15_match_pop_pulse <= 0;
                    			if(~ab2l1_WrAlmFull) begin
                    			P_out_count <= P_out_count + 1;
                    			cor_tx_wr_valid_3 <= 1'b1;
                    	cor_tx_wr_addr_3  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    			cor_tx_data_3[511:256] <= 0;
                    			end
                    			else begin
                    				P_out_count <= P_out_count;
                        cor_tx_wr_valid_3 <= 1'b0;
                    end   
                    		end
                    		else begin
                    			P_out_count <= P_out_count + 1;
                    			R15_match_pop_pulse <= 1;
                    			cor_tx_wr_valid_3 <= 1'b0;
                    		end
                    	end
                    else begin
                    	if(~ab2l1_WrAlmFull) begin
                    		P_out_count <= P_out_count + 1;
                    R15_match_pop_pulse <= 1;
                    cor_tx_wr_valid_3 <= 1'b1;
                    	cor_tx_wr_addr_3  <= output_addr;
                    	output_addr     <= output_addr + 1;
                    cor_tx_data_3[255:0] <= cor_tx_data_3[255:0];
                    cor_tx_data_3[511:256] <= {R15_mem_out_info,R15_mem_out_x2,R15_mem_out_x1,R15_mem_out_x0};
                    end
                    else begin
                    P_out_count <= P_out_count;
                    R15_match_pop_pulse <= 1'b0;
                    cor_tx_wr_valid_3 <= 1'b0;
                    end
                    end
                end
                else if(P_out_count==R15_mem_size) begin
                    	R15_match_pop_pulse <= 0;
                    	R15_read_done_hold <= 0;
                    	pop_hold <= 0;
                    	P_out_flag <= 0;
                    	P_out_count <= 0;
                    	cor_tx_wr_valid_3 <= 1'b0;
                    	readout <= 0;
                    	out_read_num <= out_read_num + 1;
                    	R15_pop_done <= 1;
                    end
                end
            end
            endcase 

			//if PE indicate output availability
			if(R0_read_done_pulse) R0_read_done_hold <= 1;  // each time finish & directly send out data                  	
            if(R1_read_done_pulse) R1_read_done_hold <= 1;
            if(R2_read_done_pulse) R2_read_done_hold <= 1;
            if(R3_read_done_pulse) R3_read_done_hold <= 1;	//reach to the end of the read
            if(R4_read_done_pulse) R4_read_done_hold <= 1;
            if(R5_read_done_pulse) R5_read_done_hold <= 1;
            if(R6_read_done_pulse) R6_read_done_hold <= 1;
            if(R7_read_done_pulse) R7_read_done_hold <= 1;
            if(R8_read_done_pulse) R8_read_done_hold <= 1;
            if(R9_read_done_pulse) R9_read_done_hold <= 1;
            if(R10_read_done_pulse) R10_read_done_hold <= 1;
            if(R11_read_done_pulse) R11_read_done_hold <= 1;
            if(R12_read_done_pulse) R12_read_done_hold <= 1;
            if(R13_read_done_pulse) R13_read_done_hold <= 1;
            if(R14_read_done_pulse) R14_read_done_hold <= 1;
            if(R15_read_done_pulse) R15_read_done_hold <= 1;
                    	
			if(pop_hold==0) begin
            if(R0_read_done_hold) begin
                token_id <= 0;
                pop_hold <= 1;
                R0_pop_done <= 0;
            end
            else if(R1_read_done_hold) begin
                token_id <= 1;
                pop_hold <= 1;
                R1_pop_done <= 0;
            end
            else if(R2_read_done_hold) begin
				token_id <= 2;
				pop_hold <= 1;
				R2_pop_done <= 0;
            end
            else if(R3_read_done_hold) begin
				token_id <= 3;
				pop_hold <= 1;
				R3_pop_done <= 0;
            end
            else if(R4_read_done_hold) begin
				token_id <= 4;
				pop_hold <= 1;
				R4_pop_done <= 0;
            end
            else if(R5_read_done_hold) begin
				token_id <= 5;
				pop_hold <= 1;
				R5_pop_done <= 0;
            end
            else if(R6_read_done_hold) begin
				token_id <= 6;
				pop_hold <= 1;
				R6_pop_done <= 0;
            end
            else if(R7_read_done_hold) begin
				token_id <= 7;
				pop_hold <= 1;
				R7_pop_done <= 0;
            end
            else if(R8_read_done_hold) begin
				token_id <= 8;
				pop_hold <= 1;
				R8_pop_done <= 0;
            end
            else if(R9_read_done_hold) begin
				token_id <= 9;
				pop_hold <= 1;
				R9_pop_done <= 0;
            end
            else if(R10_read_done_hold) begin
				token_id <= 10;
				pop_hold <= 1;
				R10_pop_done <= 0;
            end
            else if(R11_read_done_hold) begin
				token_id <= 11;
				pop_hold <= 1;
				R11_pop_done <= 0;
            end
            else if(R12_read_done_hold) begin
				token_id <= 12;
				pop_hold <= 1;
				R12_pop_done <= 0;
            end
            else if(R13_read_done_hold) begin
				token_id <= 13;
				pop_hold <= 1;
				R13_pop_done <= 0;
            end
            else if(R14_read_done_hold) begin
				token_id <= 14;
				pop_hold <= 1;
				R14_pop_done <= 0;
            end
            else if(R15_read_done_hold) begin
				token_id <= 15;
				pop_hold <= 1;
				R15_pop_done <= 0;
            end
        end
		end
	end

	always @(posedge Clk_400)
    begin
		if (!reset_read)
		  begin
			Num_Write_req    <= 0;
			Num_Write_rsp    <= 0;
			
			Total_Num_RdRsp  <= 0;
			Total_Num_RdRq	 <= 0;
		  end 
		//count total rd requests
		if(l12ab_RdEn)
		begin
			Total_Num_RdRq		<= Total_Num_RdRq + 1'b1;
		end	  
		// Count Total Rd Responses
		if (ab2l1_RdRspValid)
		begin
			Total_Num_RdRsp      <= Total_Num_RdRsp + 1'b1;
		end
		
		// Track Num Write requests
		if (l12ab_WrEn)
		begin
			Num_Write_req        <= Num_Write_req   + 1'b1;
		end  

		// Track Num Write responses
		if (ab2l1_WrRspValid) 
		begin
			Num_Write_rsp        <= Num_Write_rsp + 1'b1;   
		end

	end


//////////////////////////////////////////////////////////////////
//				providing "read" num to PE						//
//////////////////////////////////////////////////////////////////

    always @(posedge Clk_400) begin					
		if (~test_Resetb) begin
			read_in_num <= 0;
			R0_read_ack <= 0;
    		R1_read_ack <= 0;
    		R2_read_ack <= 0;
    		R3_read_ack <= 0;
    		R4_read_ack <= 0;
    		R5_read_ack <= 0;
    		R6_read_ack <= 0;
    		R7_read_ack <= 0;
    		R8_read_ack <= 0;
    		R9_read_ack <= 0;
    		R10_read_ack <= 0;
    		R11_read_ack <= 0;
    		R12_read_ack <= 0;
    		R13_read_ack <= 0;
    		R14_read_ack <= 0;
    		R15_read_ack <= 0;
    		R0_BWT_read_req_hold <= 0;
    		R1_BWT_read_req_hold <= 0;
    		R2_BWT_read_req_hold <= 0;
    		R3_BWT_read_req_hold <= 0;
    		R4_BWT_read_req_hold <= 0;
    		R5_BWT_read_req_hold <= 0;
    		R6_BWT_read_req_hold <= 0;
    		R7_BWT_read_req_hold <= 0;
    		R8_BWT_read_req_hold <= 0;
    		R9_BWT_read_req_hold <= 0;
    		R10_BWT_read_req_hold <= 0;
    		R11_BWT_read_req_hold <= 0;
    		R12_BWT_read_req_hold <= 0;
    		R13_BWT_read_req_hold <= 0;
    		R14_BWT_read_req_hold <= 0;
    		R15_BWT_read_req_hold <= 0;
		end
		if (!reset_read) begin
    		read_in_num <= 0;
			R0_read_ack <= 0;
    		R1_read_ack <= 0;
    		R2_read_ack <= 0;
    		R3_read_ack <= 0;
    		R4_read_ack <= 0;
    		R5_read_ack <= 0;
    		R6_read_ack <= 0;
    		R7_read_ack <= 0;
    		R8_read_ack <= 0;
    		R9_read_ack <= 0;
    		R10_read_ack <= 0;
    		R11_read_ack <= 0;
    		R12_read_ack <= 0;
    		R13_read_ack <= 0;
    		R14_read_ack <= 0;
    		R15_read_ack <= 0;
    		R0_BWT_read_req_hold <= 0;
    		R1_BWT_read_req_hold <= 0;
    		R2_BWT_read_req_hold <= 0;
    		R3_BWT_read_req_hold <= 0;
    		R4_BWT_read_req_hold <= 0;
    		R5_BWT_read_req_hold <= 0;
    		R6_BWT_read_req_hold <= 0;
    		R7_BWT_read_req_hold <= 0;
    		R8_BWT_read_req_hold <= 0;
    		R9_BWT_read_req_hold <= 0;
    		R10_BWT_read_req_hold <= 0;
    		R11_BWT_read_req_hold <= 0;
    		R12_BWT_read_req_hold <= 0;
    		R13_BWT_read_req_hold <= 0;
    		R14_BWT_read_req_hold <= 0;
    		R15_BWT_read_req_hold <= 0;
        end
	    else begin
		
			if(R0_BWT_read_req) R0_BWT_read_req_hold <= 1;
			if(R1_BWT_read_req) R1_BWT_read_req_hold <= 1;
			if(R2_BWT_read_req) R2_BWT_read_req_hold <= 1;
			if(R3_BWT_read_req) R3_BWT_read_req_hold <= 1;
			if(R4_BWT_read_req) R4_BWT_read_req_hold <= 1;
			if(R5_BWT_read_req) R5_BWT_read_req_hold <= 1;
			if(R6_BWT_read_req) R6_BWT_read_req_hold <= 1;
			if(R7_BWT_read_req) R7_BWT_read_req_hold <= 1;
			if(R8_BWT_read_req) R8_BWT_read_req_hold <= 1;
			if(R9_BWT_read_req) R9_BWT_read_req_hold <= 1;
			if(R10_BWT_read_req) R10_BWT_read_req_hold <= 1;
			if(R11_BWT_read_req) R11_BWT_read_req_hold <= 1;
			if(R12_BWT_read_req) R12_BWT_read_req_hold <= 1;
			if(R13_BWT_read_req) R13_BWT_read_req_hold <= 1;
			if(R14_BWT_read_req) R14_BWT_read_req_hold <= 1;
			if(R15_BWT_read_req) R15_BWT_read_req_hold <= 1;
			R0_read_ack <= 0;
    		R1_read_ack <= 0;
    		R2_read_ack <= 0;
    		R3_read_ack <= 0;
    		R4_read_ack <= 0;
    		R5_read_ack <= 0;
    		R6_read_ack <= 0;
    		R7_read_ack <= 0;
    		R8_read_ack <= 0;
    		R9_read_ack <= 0;
    		R10_read_ack <= 0;
    		R11_read_ack <= 0;
    		R12_read_ack <= 0;
    		R13_read_ack <= 0;
    		R14_read_ack <= 0;
    		R15_read_ack <= 0;
			if(read_in_num <= (batch_size-1) && (BWT_state == BWT_STATE__BWT_run)) begin
				if(R0_BWT_read_req_hold) begin
				R0_read_ack <= 1;
				R0_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R0_BWT_read_req_hold <= 0;
				end
				else if(R1_BWT_read_req_hold) begin
				R1_read_ack <= 1;
				R1_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R1_BWT_read_req_hold <= 0;
				end
				else if(R2_BWT_read_req_hold) begin
				R2_read_ack <= 1;
				R2_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R2_BWT_read_req_hold <= 0;
				end
				else if(R3_BWT_read_req_hold) begin
				R3_read_ack <= 1;
				R3_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R3_BWT_read_req_hold <= 0;
				end
				else if(R4_BWT_read_req_hold) begin
				R4_read_ack <= 1;
				R4_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R4_BWT_read_req_hold <= 0;
				end
				else if(R5_BWT_read_req_hold) begin
				R5_read_ack <= 1;
				R5_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R5_BWT_read_req_hold <= 0;
				end
				else if(R6_BWT_read_req_hold) begin
				R6_read_ack <= 1;
				R6_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R6_BWT_read_req_hold <= 0;
				end
				else if(R7_BWT_read_req_hold) begin
				R7_read_ack <= 1;
				R7_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R7_BWT_read_req_hold <= 0;
				end
				else if(R8_BWT_read_req_hold) begin
				R8_read_ack <= 1;
				R8_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R8_BWT_read_req_hold <= 0;
				end
				else if(R9_BWT_read_req_hold) begin
				R9_read_ack <= 1;
				R9_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R9_BWT_read_req_hold <= 0;
				end
				else if(R10_BWT_read_req_hold) begin
				R10_read_ack <= 1;
				R10_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R10_BWT_read_req_hold <= 0;
				end
				else if(R11_BWT_read_req_hold) begin
				R11_read_ack <= 1;
				R11_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R11_BWT_read_req_hold <= 0;
				end
				else if(R12_BWT_read_req_hold) begin
				R12_read_ack <= 1;
				R12_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R12_BWT_read_req_hold <= 0;
				end
				else if(R13_BWT_read_req_hold) begin
				R13_read_ack <= 1;
				R13_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R13_BWT_read_req_hold <= 0;
				end
				else if(R14_BWT_read_req_hold) begin
				R14_read_ack <= 1;
				R14_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R14_BWT_read_req_hold <= 0;
				end
				else if(R15_BWT_read_req_hold) begin
				R15_read_ack <= 1;
				R15_read_num <= read_in_num;
				read_in_num <= read_in_num_next;
				R15_BWT_read_req_hold <= 0;
				end
      		end
		end
		
	end
	wire clk;
	assign clk = Clk_400;
    // reset read
	assign reset_read = test_Resetb & pop_done_pulse_n;
    always @(posedge Clk_400) begin
      pop_done_q <= pop_done;
   	  if((pop_done==1) && (pop_done_q==0)) pop_done_pulse_n <= 0;
      else pop_done_pulse_n <= 1;
    end

    Wrapper R0(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag1),		//ok
      .CNT_in          (CNT_in1),		//ok
      .CNT_addr        (CNT_addr1),		//ok
      .read_ack        (R0_read_ack),	//ok
      .read_num        (R0_read_num),	//ok
      .BWT_req_pulse   (R0_BWT_read_req),	//ok
      .BWT_input_valid (R0_BWT_input_valid),//ok
      .BWT_input       (R0_BWT_input),		//ok
      .read_trigger    (R0_read_trigger),	//ok, defines read/req
      .match_pop_pulse (R0_match_pop_pulse),//input
      .pop_done        (R0_pop_done),		//input ----
      .DRAM_valid      (R0_DRAM_valid),		//ok
      .addr_k          (R0_addr_k),			//ok
      .addr_l          (R0_addr_l),			//ok
      .read_done_pulse (R0_read_done_pulse),//output ----
      .ret             (R0_ret),
      .mem_size        (R0_mem_size),
      .mem_out_info    (R0_mem_out_info),
      .mem_out_x2      (R0_mem_out_x2),
      .mem_out_x1      (R0_mem_out_x1),
      .mem_out_x0      (R0_mem_out_x0)
    );
	Wrapper R1(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag1),
      .CNT_in          (CNT_in1),
      .CNT_addr        (CNT_addr1),
      .read_ack        (R1_read_ack),
      .read_num        (R1_read_num),
      .BWT_req_pulse   (R1_BWT_read_req),
      .BWT_input_valid (R1_BWT_input_valid),
      .BWT_input       (R1_BWT_input),
      .read_trigger    (R1_read_trigger),
      .match_pop_pulse (R1_match_pop_pulse),
      .pop_done        (R1_pop_done),
      .DRAM_valid      (R1_DRAM_valid),
      .addr_k          (R1_addr_k), 
      .addr_l          (R1_addr_l),
      .read_done_pulse (R1_read_done_pulse),
      .ret             (R1_ret),
      .mem_size        (R1_mem_size),
      .mem_out_info    (R1_mem_out_info),
      .mem_out_x2      (R1_mem_out_x2),
      .mem_out_x1      (R1_mem_out_x1),
      .mem_out_x0      (R1_mem_out_x0)
    );                      
    Wrapper R2(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag1),
      .CNT_in          (CNT_in1),
      .CNT_addr        (CNT_addr1),
      .read_ack        (R2_read_ack),
      .read_num        (R2_read_num),
      .BWT_req_pulse   (R2_BWT_read_req),
      .BWT_input_valid (R2_BWT_input_valid),
      .BWT_input       (R2_BWT_input),
      .read_trigger    (R2_read_trigger),  
      .match_pop_pulse (R2_match_pop_pulse),
      .pop_done        (R2_pop_done),
      .DRAM_valid      (R2_DRAM_valid),
      .addr_k          (R2_addr_k), 
      .addr_l          (R2_addr_l),
      .read_done_pulse (R2_read_done_pulse),
      .ret             (R2_ret),
      .mem_size        (R2_mem_size),
      .mem_out_info    (R2_mem_out_info),
      .mem_out_x2      (R2_mem_out_x2),
      .mem_out_x1      (R2_mem_out_x1),
      .mem_out_x0      (R2_mem_out_x0)
    );                      
    Wrapper R3(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag1),
      .CNT_in          (CNT_in1),
      .CNT_addr        (CNT_addr1),
      .read_ack        (R3_read_ack),
      .read_num        (R3_read_num),
      .BWT_req_pulse   (R3_BWT_read_req),
      .BWT_input_valid (R3_BWT_input_valid),
      .BWT_input       (R3_BWT_input),
      .read_trigger    (R3_read_trigger),  
      .match_pop_pulse (R3_match_pop_pulse),
      .pop_done        (R3_pop_done),
      .DRAM_valid      (R3_DRAM_valid),
      .addr_k          (R3_addr_k), 
      .addr_l          (R3_addr_l),
      .read_done_pulse (R3_read_done_pulse),
      .ret             (R3_ret),
      .mem_size        (R3_mem_size),
      .mem_out_info    (R3_mem_out_info),
      .mem_out_x2      (R3_mem_out_x2),
      .mem_out_x1      (R3_mem_out_x1),
      .mem_out_x0      (R3_mem_out_x0)
    );
    Wrapper R4(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag1),
      .CNT_in          (CNT_in1),
      .CNT_addr        (CNT_addr1),
      .read_ack        (R4_read_ack),
      .read_num        (R4_read_num),
      .BWT_req_pulse   (R4_BWT_read_req),
      .BWT_input_valid (R4_BWT_input_valid),
      .BWT_input       (R4_BWT_input),
      .read_trigger    (R4_read_trigger),
      .match_pop_pulse (R4_match_pop_pulse),
      .pop_done        (R4_pop_done),
      .DRAM_valid      (R4_DRAM_valid),
      .addr_k          (R4_addr_k), 
      .addr_l          (R4_addr_l),
      .read_done_pulse (R4_read_done_pulse),
      .ret             (R4_ret),
      .mem_size        (R4_mem_size),
      .mem_out_info    (R4_mem_out_info),
      .mem_out_x2      (R4_mem_out_x2),
      .mem_out_x1      (R4_mem_out_x1),
      .mem_out_x0      (R4_mem_out_x0)
    );
    Wrapper R5(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag1),
      .CNT_in          (CNT_in1),
      .CNT_addr        (CNT_addr1),
      .read_ack        (R5_read_ack),
      .read_num        (R5_read_num),
      .BWT_req_pulse   (R5_BWT_read_req),
      .BWT_input_valid (R5_BWT_input_valid),
      .BWT_input       (R5_BWT_input),
      .read_trigger    (R5_read_trigger),  
      .match_pop_pulse (R5_match_pop_pulse),
      .pop_done        (R5_pop_done),
      .DRAM_valid      (R5_DRAM_valid),
      .addr_k          (R5_addr_k), 
      .addr_l          (R5_addr_l),
      .read_done_pulse (R5_read_done_pulse),
      .ret             (R5_ret),
      .mem_size        (R5_mem_size),
      .mem_out_info    (R5_mem_out_info),
      .mem_out_x2      (R5_mem_out_x2),
      .mem_out_x1      (R5_mem_out_x1),
      .mem_out_x0      (R5_mem_out_x0)
    );
    Wrapper R6(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag1),
      .CNT_in          (CNT_in1),
      .CNT_addr        (CNT_addr1),
      .read_ack        (R6_read_ack),
      .read_num        (R6_read_num),
      .BWT_req_pulse   (R6_BWT_read_req),
      .BWT_input_valid (R6_BWT_input_valid),
      .BWT_input       (R6_BWT_input),
      .read_trigger    (R6_read_trigger),
      .match_pop_pulse (R6_match_pop_pulse),
      .pop_done        (R6_pop_done),
      .DRAM_valid      (R6_DRAM_valid),
      .addr_k          (R6_addr_k), 
      .addr_l          (R6_addr_l),
      .read_done_pulse (R6_read_done_pulse),
      .ret             (R6_ret),
      .mem_size        (R6_mem_size),
      .mem_out_info    (R6_mem_out_info),
      .mem_out_x2      (R6_mem_out_x2),
      .mem_out_x1      (R6_mem_out_x1),
      .mem_out_x0      (R6_mem_out_x0)
    );
    Wrapper R7(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag1),
      .CNT_in          (CNT_in1),
      .CNT_addr        (CNT_addr1),
      .read_ack        (R7_read_ack),
      .read_num        (R7_read_num),
      .BWT_req_pulse   (R7_BWT_read_req),
      .BWT_input_valid (R7_BWT_input_valid),
      .BWT_input       (R7_BWT_input),
      .read_trigger    (R7_read_trigger),  
      .match_pop_pulse (R7_match_pop_pulse),
      .pop_done        (R7_pop_done),
      .DRAM_valid      (R7_DRAM_valid),
      .addr_k          (R7_addr_k), 
      .addr_l          (R7_addr_l),
      .read_done_pulse (R7_read_done_pulse),
      .ret             (R7_ret),
      .mem_size        (R7_mem_size),
      .mem_out_info    (R7_mem_out_info),
      .mem_out_x2      (R7_mem_out_x2),
      .mem_out_x1      (R7_mem_out_x1),
      .mem_out_x0      (R7_mem_out_x0)
    );
    Wrapper R8(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag2),
      .CNT_in          (CNT_in2),
      .CNT_addr        (CNT_addr2),
      .read_ack        (R8_read_ack),
      .read_num        (R8_read_num),
      .BWT_req_pulse   (R8_BWT_read_req),
      .BWT_input_valid (R8_BWT_input_valid),
      .BWT_input       (R8_BWT_input),
      .read_trigger    (R8_read_trigger),
      .match_pop_pulse (R8_match_pop_pulse),
      .pop_done        (R8_pop_done),
      .DRAM_valid      (R8_DRAM_valid),
      .addr_k          (R8_addr_k), 
      .addr_l          (R8_addr_l),
      .read_done_pulse (R8_read_done_pulse),
      .ret             (R8_ret),
      .mem_size        (R8_mem_size),
      .mem_out_info    (R8_mem_out_info),
      .mem_out_x2      (R8_mem_out_x2),
      .mem_out_x1      (R8_mem_out_x1),
      .mem_out_x0      (R8_mem_out_x0)
    );
    Wrapper R9(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag2),
      .CNT_in          (CNT_in2),
      .CNT_addr        (CNT_addr2),
      .read_ack        (R9_read_ack),
      .read_num        (R9_read_num),
      .BWT_req_pulse   (R9_BWT_read_req),
      .BWT_input_valid (R9_BWT_input_valid),
      .BWT_input       (R9_BWT_input), 
      .read_trigger    (R9_read_trigger), 
      .match_pop_pulse (R9_match_pop_pulse),
      .pop_done        (R9_pop_done),
      .DRAM_valid      (R9_DRAM_valid),
      .addr_k          (R9_addr_k), 
      .addr_l          (R9_addr_l),
      .read_done_pulse (R9_read_done_pulse),
      .ret             (R9_ret),
      .mem_size        (R9_mem_size),
      .mem_out_info    (R9_mem_out_info),
      .mem_out_x2      (R9_mem_out_x2),
      .mem_out_x1      (R9_mem_out_x1),
      .mem_out_x0      (R9_mem_out_x0)
    );
    Wrapper R10(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag2),
      .CNT_in          (CNT_in2),
      .CNT_addr        (CNT_addr2),
      .read_ack        (R10_read_ack),
      .read_num        (R10_read_num),
      .BWT_req_pulse   (R10_BWT_read_req),
      .BWT_input_valid (R10_BWT_input_valid),
      .BWT_input       (R10_BWT_input), 
      .read_trigger    (R10_read_trigger), 
      .match_pop_pulse (R10_match_pop_pulse),
      .pop_done        (R10_pop_done),
      .DRAM_valid      (R10_DRAM_valid),
      .addr_k          (R10_addr_k), 
      .addr_l          (R10_addr_l),
      .read_done_pulse (R10_read_done_pulse),
      .ret             (R10_ret),
      .mem_size        (R10_mem_size),
      .mem_out_info    (R10_mem_out_info),
      .mem_out_x2      (R10_mem_out_x2),
      .mem_out_x1      (R10_mem_out_x1),
      .mem_out_x0      (R10_mem_out_x0)
    );
    Wrapper R11(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag2),
      .CNT_in          (CNT_in2),
      .CNT_addr        (CNT_addr2),
      .read_ack        (R11_read_ack),
      .read_num        (R11_read_num),
      .BWT_req_pulse   (R11_BWT_read_req),
      .BWT_input_valid (R11_BWT_input_valid),
      .BWT_input       (R11_BWT_input),  
      .read_trigger    (R11_read_trigger),
      .match_pop_pulse (R11_match_pop_pulse),
      .pop_done        (R11_pop_done),
      .DRAM_valid      (R11_DRAM_valid),
      .addr_k          (R11_addr_k), 
      .addr_l          (R11_addr_l),
      .read_done_pulse (R11_read_done_pulse),
      .ret             (R11_ret),
      .mem_size        (R11_mem_size),
      .mem_out_info    (R11_mem_out_info),
      .mem_out_x2      (R11_mem_out_x2),
      .mem_out_x1      (R11_mem_out_x1),
      .mem_out_x0      (R11_mem_out_x0)
    );
    Wrapper R12(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag2),
      .CNT_in          (CNT_in2),
      .CNT_addr        (CNT_addr2),
      .read_ack        (R12_read_ack),
      .read_num        (R12_read_num),
      .BWT_req_pulse   (R12_BWT_read_req),
      .BWT_input_valid (R12_BWT_input_valid),
      .BWT_input       (R12_BWT_input),
      .read_trigger    (R12_read_trigger),  
      .match_pop_pulse (R12_match_pop_pulse),
      .pop_done        (R12_pop_done),
      .DRAM_valid      (R12_DRAM_valid),
      .addr_k          (R12_addr_k), 
      .addr_l          (R12_addr_l),
      .read_done_pulse (R12_read_done_pulse),
      .ret             (R12_ret),
      .mem_size        (R12_mem_size),
      .mem_out_info    (R12_mem_out_info),
      .mem_out_x2      (R12_mem_out_x2),
      .mem_out_x1      (R12_mem_out_x1),
      .mem_out_x0      (R12_mem_out_x0)
    );
    Wrapper R13(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag2),
      .CNT_in          (CNT_in2),
      .CNT_addr        (CNT_addr2),
      .read_ack        (R13_read_ack),
      .read_num        (R13_read_num),
      .BWT_req_pulse   (R13_BWT_read_req),
      .BWT_input_valid (R13_BWT_input_valid),
      .BWT_input       (R13_BWT_input),
      .read_trigger    (R13_read_trigger),  
      .match_pop_pulse (R13_match_pop_pulse),
      .pop_done        (R13_pop_done),
      .DRAM_valid      (R13_DRAM_valid),
      .addr_k          (R13_addr_k), 
      .addr_l          (R13_addr_l),
      .read_done_pulse (R13_read_done_pulse),
      .ret             (R13_ret),
      .mem_size        (R13_mem_size),
      .mem_out_info    (R13_mem_out_info),
      .mem_out_x2      (R13_mem_out_x2),
      .mem_out_x1      (R13_mem_out_x1),
      .mem_out_x0      (R13_mem_out_x0)
    );
    Wrapper R14(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag2),
      .CNT_in          (CNT_in2),
      .CNT_addr        (CNT_addr2),
      .read_ack        (R14_read_ack),
      .read_num        (R14_read_num),
      .BWT_req_pulse   (R14_BWT_read_req),
      .BWT_input_valid (R14_BWT_input_valid),
      .BWT_input       (R14_BWT_input), 
      .read_trigger    (R14_read_trigger), 
      .match_pop_pulse (R14_match_pop_pulse),
      .pop_done        (R14_pop_done),
      .DRAM_valid      (R14_DRAM_valid),
      .addr_k          (R14_addr_k), 
      .addr_l          (R14_addr_l),
      .read_done_pulse (R14_read_done_pulse),
      .ret             (R14_ret),
      .mem_size        (R14_mem_size),
      .mem_out_info    (R14_mem_out_info),
      .mem_out_x2      (R14_mem_out_x2),
      .mem_out_x1      (R14_mem_out_x1),
      .mem_out_x0      (R14_mem_out_x0)
    );
    Wrapper R15(
      .clk             (clk),
      .reset_read      (reset_read),
      .CNT_flag        (CNT_flag2),
      .CNT_in          (CNT_in2),
      .CNT_addr        (CNT_addr2),
      .read_ack        (R15_read_ack),
      .read_num        (R15_read_num),
      .BWT_req_pulse   (R15_BWT_read_req),
      .BWT_input_valid (R15_BWT_input_valid),
      .BWT_input       (R15_BWT_input),  
      .read_trigger    (R15_read_trigger),
      .match_pop_pulse (R15_match_pop_pulse),
      .pop_done        (R15_pop_done),
      .DRAM_valid      (R15_DRAM_valid),
      .addr_k          (R15_addr_k), 
      .addr_l          (R15_addr_l),
      .read_done_pulse (R15_read_done_pulse),
      .ret             (R15_ret),
      .mem_size        (R15_mem_size),
      .mem_out_info    (R15_mem_out_info),
      .mem_out_x2      (R15_mem_out_x2),
      .mem_out_x1      (R15_mem_out_x1),
      .mem_out_x0      (R15_mem_out_x0)
    );
   // synthesis translate_off
   //logic numCL_error = 0;
   //always @(posedge Clk_400)
   //begin
   //  if( re2xy_go && ())
   //  begin
   //    $display("%m \m ERROR: some read response is not as expected");
       //$display("\m re2xy_NumLines = %d and re2xy_multiCL_len = %d",re2xy_NumLines,re2xy_multiCL_len);
   //    numCL_error <= 1'b1;
  //   end
  
   //  if(numCL_error)
     //$finish();
  // end
   // synthesis translate_on
endmodule



//read_ack --nextcycle--> read_ack_pulse --nextcycle--> BWT_read_req=0, otherwise is always 1, --nextcycle--> send request for read
module Wrapper(
    //Input: Global Control
    input   wire          clk,
    input   wire          reset_read,
    //Input: Load CNT table
    input   wire          CNT_flag,
    input   wire [63:0]   CNT_in,
    input   wire [7:0]    CNT_addr,
    //Input: BWT_query and BWT_ref
    input   wire          read_ack,		//read valid now //1 cycle of high, then store the value inside PE
    input   wire [9:0]    read_num,		//the numth read to be processed
    input   wire          BWT_input_valid, //use read_id to assign BWTinput back to PE
    input   wire [511:0]  BWT_input,  
    //Input: Pop output data //two are complements
    input   wire          match_pop_pulse, // unknown input //pop out match?
    input   wire          pop_done,		//control part '1' indicate finish reading out mem_out then resetting itself 

    output  logic           BWT_req_pulse, //indicate availability, can assign new read
    output  logic           read_trigger,	// is always 0, until the read information are processed
    output  wire          DRAM_valid,	//mem_request valid //requesting 2 CLs each DRAM_valid
    output  wire [31:0]   addr_k,		//request data from read address or bwt address 
    output  wire [31:0]   addr_l,		//request data from read address or bwt address 
    output  logic           read_done_pulse, //reach to the boundary of one read
    output  wire [31:0]   ret,
    output  wire [6:0]    mem_size,
    output  wire [63:0]   mem_out_info,
    output  wire [63:0]   mem_out_x2,
    output  wire [63:0]   mem_out_x1,
    output  wire [63:0]   mem_out_x0
  );
    
    // Input for SMEM 
    logic [2:0] BWT_in_counter;
    logic query_start;
    logic [511:0] query_in1,query_in2;
    logic [7:0] query_in, query_in_q;
    logic [7:0] query_counter;
    logic query_in_valid;
    logic [6:0]  query_in_addr;  
    logic [63:0] ik_x0_in,ik_x1_in,ik_x2_in,ik_info_in;
    logic [63:0] L2_0_in,L2_1_in,L2_2_in,L2_3_in;
    logic [63:0] primary;
    logic [6:0]  forward_x,backward_x;
    logic [6:0]  min_intv;
    // Control for read & DRAM access
    logic read_ack_q,read_ack_pulse;
    logic BWT_read_req,BWT_read_req_q;
    wire BWT_read_in_valid,BWT_ref_valid;
    logic BWT_read_tigger_pulse1,BWT_read_tigger_pulse2;
    logic [9:0] BWT_read_addr_base;
    logic	BWT_read_valid;
    logic [31:0]BWT_read_addr_k, BWT_read_addr_l;
    // DRAM access for cntk,cntl
    logic DRAM_get;
    logic DRAM_number;
    reg [31:0]  cnt_k_a0, cnt_k_a1, cnt_k_a2, cnt_k_a3;
    reg [63:0]  cnt_k_b0, cnt_k_b1, cnt_k_b2, cnt_k_b3;
    reg [31:0]  cnt_l_a0, cnt_l_a1, cnt_l_a2, cnt_l_a3;
    reg [63:0]  cnt_l_b0, cnt_l_b1, cnt_l_b2, cnt_l_b3;
    // Output from 1-read
    wire read_done;
    wire BWT_extend_done_pulse;
    wire DRAM_get_same;
    wire BWT_DRAM_valid;
    wire [31:0] BWT_DRAM_addr_k, BWT_DRAM_addr_l;
    // logicisters for timing adjustment 
    wire   match_pop_pulse_new;
    logic    read_done_pulse_temp;
    logic    read_done_q; 
    // PE_Reset signal
    wire reset_read_PE;
    logic pop_done_q, pop_done_pulse_n;
     
   
    //reset generator for PE
    assign reset_read_PE = reset_read & pop_done_pulse_n;
    always @(posedge clk) begin
      pop_done_q <= pop_done;
   	  if((pop_done==1) && (pop_done_q==0)) pop_done_pulse_n <= 0;
      else pop_done_pulse_n <= 1;
    end
     
    // Pop output control
    assign match_pop_pulse_new = match_pop_pulse | read_done_pulse;

    always @(posedge clk)
    begin
    	if(!reset_read_PE) begin
    		read_done_pulse <= 0;
    		read_done_pulse_temp <= 0;
    	end
    	else begin
   	   read_done_pulse <= read_done_pulse_temp;
   	   read_done_q <= read_done;
   	   if((read_done==1) && (read_done_q==0)) 
   	   begin
   	 	   read_done_pulse_temp <= 1;
   	   end
   	   else
   	   begin
   	 	  read_done_pulse_temp <= 0;
   	   end
   	 end
    end
    
    // Mux for Read_in or BWT_ref  
    assign BWT_read_in_valid = (read_trigger) ? 0 : BWT_input_valid;
    assign BWT_ref_valid     = (read_trigger) ? BWT_input_valid : 0;
    assign DRAM_valid = (read_trigger) ? BWT_DRAM_valid : BWT_read_valid;
    assign addr_k = (read_trigger) ? BWT_DRAM_addr_k : BWT_read_addr_k;
    assign addr_l = (read_trigger) ? BWT_DRAM_addr_l : BWT_read_addr_l;
    
    //read_ack pulse generator   
    always @(posedge clk)
    begin
   	 if(!reset_read_PE) begin
   		 read_ack_q <= 0;
   		 read_ack_pulse <= 0;
   	 end
   	 else begin
   		 read_ack_q <= read_ack;
   	   if(read_ack==1 && read_ack_q==0) read_ack_pulse <= 1;
   	   else                             read_ack_pulse <= 0;
   	 end
    end
    
    //read_req pulse generator   
    always @(posedge clk)
    begin
   	 if(!reset_read_PE) begin
   		 BWT_read_req_q <= 0;
   		 BWT_req_pulse <= 0;
   	 end
   	 else begin
   		 BWT_read_req_q <= BWT_read_req;
   	   if(BWT_read_req==1 && BWT_read_req_q==0) BWT_req_pulse <= 1;
   	   else                                     BWT_req_pulse <= 0;
   	 end
    end

    // Read SMEM Input
    always @(posedge clk) begin
    	if(!reset_read_PE) begin
    		BWT_read_req  <= 1;
    		BWT_read_addr_base <= 0;
    		BWT_read_tigger_pulse1 <= 0;
    		BWT_read_tigger_pulse2 <= 0;
    		BWT_read_valid <= 0;
    		BWT_read_addr_k <= 0;
    		BWT_read_addr_l <= 0;
    	end
    	else begin
    		BWT_read_valid <= 0;
    		if(read_ack_pulse) begin
    			BWT_read_req <= 0;
    			BWT_read_addr_base <= read_num; //the number of read in the read queue
    			BWT_read_tigger_pulse1 <= 1;
    		end
    		if(BWT_read_tigger_pulse1) begin
    			BWT_read_valid <= 1;
    			BWT_read_addr_k <= {20'd0,BWT_read_addr_base,2'd0};
    			BWT_read_addr_l <= {20'd0,BWT_read_addr_base,2'd1};
    			BWT_read_tigger_pulse1 <= 0;
    		end
    		if(BWT_read_in_valid && (BWT_in_counter==1)) begin
    			BWT_read_tigger_pulse2 <= 1;
    		end
    		if(BWT_read_tigger_pulse2) begin
    			BWT_read_valid <= 1;
    			BWT_read_addr_k <= {20'd0,BWT_read_addr_base,2'd2};
    			BWT_read_addr_l <= {20'd0,BWT_read_addr_base,2'd3};
    			BWT_read_tigger_pulse2 <= 0;
    		end
    	end
    end
    		
    
    always @(posedge clk) begin
    	query_in_q <= query_in;
    end 
    
    // Input Control
    always @(posedge clk) begin
    	if(!reset_read_PE) begin
    		BWT_in_counter <= 0;
    		query_start <= 0;
    		read_trigger <= 0;
    		query_counter <= 0;
        query_in_valid <= 0;
        query_in_addr <= 0;
        query_in <= 0;      
    	end 
    	else begin
    		if(BWT_read_in_valid) begin
    			BWT_in_counter <= BWT_in_counter + 1;
    			if(BWT_in_counter==0) query_in1 <= BWT_input;
				if(BWT_in_counter==1) query_in2 <= BWT_input;
				if(BWT_in_counter==2) begin
          			forward_x  <= BWT_input[6:0];
					backward_x <= BWT_input[6:0];
					min_intv   <= BWT_input[70:64];
					primary    <= BWT_input[191:128];      
				end
			  if(BWT_in_counter==3) begin
        		ik_x0_in   <= BWT_input[63:0];
				ik_x1_in   <= BWT_input[127:64];
				ik_x2_in   <= BWT_input[191:128];
				ik_info_in <= BWT_input[255:192];
				L2_0_in    <= BWT_input[319:256];
				L2_1_in    <= BWT_input[383:320];
				L2_2_in    <= BWT_input[447:384];
				L2_3_in    <= BWT_input[511:448];
				query_start <= 1;
			  end
		   end
        if(query_start) begin
          if(query_counter==100) begin
           query_start <= 0;
           read_trigger <= 1;
          end
          query_counter      <= query_counter + 1;
          query_in_valid     <= 1;
          query_in_addr      <= query_counter;
          query_in           <= query_in1[7:0];   
          query_in1[7:0]     <= query_in1[15:8];   
          query_in1[15:8]    <= query_in1[23:16];  
          query_in1[23:16]   <= query_in1[31:24]; 
          query_in1[31:24]   <= query_in1[39:32];  
          query_in1[39:32]   <= query_in1[47:40];  
          query_in1[47:40]   <= query_in1[55:48];  
          query_in1[55:48]   <= query_in1[63:56];  
          query_in1[63:56]   <= query_in1[71:64];  
          query_in1[71:64]   <= query_in1[79:72];  
          query_in1[79:72]   <= query_in1[87:80];  
          query_in1[87:80]   <= query_in1[95:88];  
          query_in1[95:88]   <= query_in1[103:96]; 
          query_in1[103:96]  <= query_in1[111:104];
          query_in1[111:104] <= query_in1[119:112];
          query_in1[119:112] <= query_in1[127:120];
          query_in1[127:120] <= query_in1[135:128];
          query_in1[135:128] <= query_in1[143:136];
          query_in1[143:136] <= query_in1[151:144];
          query_in1[151:144] <= query_in1[159:152];
          query_in1[159:152] <= query_in1[167:160];
          query_in1[167:160] <= query_in1[175:168];
          query_in1[175:168] <= query_in1[183:176];
          query_in1[183:176] <= query_in1[191:184];
          query_in1[191:184] <= query_in1[199:192];
          query_in1[199:192] <= query_in1[207:200];
          query_in1[207:200] <= query_in1[215:208];
          query_in1[215:208] <= query_in1[223:216];
          query_in1[223:216] <= query_in1[231:224];
          query_in1[231:224] <= query_in1[239:232];
          query_in1[239:232] <= query_in1[247:240];
          query_in1[247:240] <= query_in1[255:248];
          query_in1[255:248] <= query_in1[263:256];
          query_in1[263:256] <= query_in1[271:264];
          query_in1[271:264] <= query_in1[279:272];
          query_in1[279:272] <= query_in1[287:280];
          query_in1[287:280] <= query_in1[295:288];
          query_in1[295:288] <= query_in1[303:296];
          query_in1[303:296] <= query_in1[311:304];
          query_in1[311:304] <= query_in1[319:312];
          query_in1[319:312] <= query_in1[327:320];
          query_in1[327:320] <= query_in1[335:328];
          query_in1[335:328] <= query_in1[343:336];
          query_in1[343:336] <= query_in1[351:344];
          query_in1[351:344] <= query_in1[359:352];
          query_in1[359:352] <= query_in1[367:360];
          query_in1[367:360] <= query_in1[375:368];
          query_in1[375:368] <= query_in1[383:376];
          query_in1[383:376] <= query_in1[391:384];
          query_in1[391:384] <= query_in1[399:392];
          query_in1[399:392] <= query_in1[407:400];
          query_in1[407:400] <= query_in1[415:408];
          query_in1[415:408] <= query_in1[423:416];
          query_in1[423:416] <= query_in1[431:424];
          query_in1[431:424] <= query_in1[439:432];
          query_in1[439:432] <= query_in1[447:440];
          query_in1[447:440] <= query_in1[455:448];
          query_in1[455:448] <= query_in1[463:456];
          query_in1[463:456] <= query_in1[471:464];
          query_in1[471:464] <= query_in1[479:472];
          query_in1[479:472] <= query_in1[487:480];
          query_in1[487:480] <= query_in1[495:488];
          query_in1[495:488] <= query_in1[503:496];
          query_in1[503:496] <= query_in1[511:504];
          query_in1[511:504] <= query_in2[7:0];
          query_in2[7:0]     <= query_in2[15:8];   
          query_in2[15:8]    <= query_in2[23:16];  
          query_in2[23:16]   <= query_in2[31:24]; 
          query_in2[31:24]   <= query_in2[39:32];  
          query_in2[39:32]   <= query_in2[47:40];  
          query_in2[47:40]   <= query_in2[55:48];  
          query_in2[55:48]   <= query_in2[63:56];  
          query_in2[63:56]   <= query_in2[71:64];  
          query_in2[71:64]   <= query_in2[79:72];  
          query_in2[79:72]   <= query_in2[87:80];  
          query_in2[87:80]   <= query_in2[95:88];  
          query_in2[95:88]   <= query_in2[103:96]; 
          query_in2[103:96]  <= query_in2[111:104];
          query_in2[111:104] <= query_in2[119:112];
          query_in2[119:112] <= query_in2[127:120];
          query_in2[127:120] <= query_in2[135:128];
          query_in2[135:128] <= query_in2[143:136];
          query_in2[143:136] <= query_in2[151:144];
          query_in2[151:144] <= query_in2[159:152];
          query_in2[159:152] <= query_in2[167:160];
          query_in2[167:160] <= query_in2[175:168];
          query_in2[175:168] <= query_in2[183:176];
          query_in2[183:176] <= query_in2[191:184];
          query_in2[191:184] <= query_in2[199:192];
          query_in2[199:192] <= query_in2[207:200];
          query_in2[207:200] <= query_in2[215:208];
          query_in2[215:208] <= query_in2[223:216];
          query_in2[223:216] <= query_in2[231:224];
          query_in2[231:224] <= query_in2[239:232];
          query_in2[239:232] <= query_in2[247:240];
          query_in2[247:240] <= query_in2[255:248];
          query_in2[255:248] <= query_in2[263:256];
          query_in2[263:256] <= query_in2[271:264];
          query_in2[271:264] <= query_in2[279:272];
          query_in2[279:272] <= query_in2[287:280];
          query_in2[287:280] <= query_in2[295:288];
          query_in2[295:288] <= query_in2[303:296];
          query_in2[303:296] <= query_in2[311:304];
          query_in2[311:304] <= query_in2[319:312];
          query_in2[319:312] <= query_in2[327:320];
          query_in2[327:320] <= query_in2[335:328];
          query_in2[335:328] <= query_in2[343:336];
          query_in2[343:336] <= query_in2[351:344];
          query_in2[351:344] <= query_in2[359:352];
          query_in2[359:352] <= query_in2[367:360];
          query_in2[367:360] <= query_in2[375:368];
          query_in2[375:368] <= query_in2[383:376];
          query_in2[383:376] <= query_in2[391:384];
          query_in2[391:384] <= query_in2[399:392];
          query_in2[399:392] <= query_in2[407:400];
          query_in2[407:400] <= query_in2[415:408];
          query_in2[415:408] <= query_in2[423:416];
          query_in2[423:416] <= query_in2[431:424];
          query_in2[431:424] <= query_in2[439:432];
          query_in2[439:432] <= query_in2[447:440];
          query_in2[447:440] <= query_in2[455:448];
          query_in2[455:448] <= query_in2[463:456];
          query_in2[463:456] <= query_in2[471:464];
          query_in2[471:464] <= query_in2[479:472];
          query_in2[479:472] <= query_in2[487:480];
          query_in2[487:480] <= query_in2[495:488];
          query_in2[495:488] <= query_in2[503:496];
          query_in2[503:496] <= query_in2[511:504];
        end
        else begin
        	query_in_valid     <= 0;
        end    		
      end
    end
      
      
    // Read Reference Geno
    always @(posedge clk) begin
    	if(!reset_read_PE) begin
    		DRAM_number <= 0;
    		DRAM_get <= 0;
    	end 
    	else begin
    		if(BWT_ref_valid) begin
    			if(DRAM_number==0) begin
          		DRAM_get <= 0;
          		DRAM_number <= 1;
          		cnt_k_a0 <= BWT_input[31:0];   
				cnt_k_a1 <= BWT_input[95:64];  
				cnt_k_a2 <= BWT_input[159:128];
				cnt_k_a3 <= BWT_input[223:192];
				cnt_k_b0 <= BWT_input[319:256];
				cnt_k_b1 <= BWT_input[383:320];
				cnt_k_b2 <= BWT_input[447:384];
				cnt_k_b3 <= BWT_input[511:448];
			    end
         		else if(DRAM_number==1) begin
          		DRAM_get <= 1;
          		DRAM_number <= 0;
          		cnt_l_a0 <= BWT_input[31:0];   
				cnt_l_a1 <= BWT_input[95:64];  
				cnt_l_a2 <= BWT_input[159:128];
				cnt_l_a3 <= BWT_input[223:192];
				cnt_l_b0 <= BWT_input[319:256];
				cnt_l_b1 <= BWT_input[383:320];
				cnt_l_b2 <= BWT_input[447:384];
				cnt_l_b3 <= BWT_input[511:448];
			    end
           end
		   else begin
              if(DRAM_get_same)              DRAM_get <= 1;
			  else if(BWT_extend_done_pulse) DRAM_get <= 0;
			  else                           DRAM_get <= DRAM_get;
		   end
      end
    end


  PE_read PE_wrapper_read(
     //input
     .Clk_32UI        (clk),
     .reset_read      (reset_read_PE),
     .CNT_flag        (CNT_flag),
     .CNT_data        (CNT_in),
     .CNT_addr        (CNT_addr),
     .query_in_valid  (query_in_valid),
     .query_in_addr   (query_in_addr),
     .query_in        (query_in_q),
     .ik_info_in      (ik_info_in),
     .ik_x0_in        (ik_x0_in),
     .ik_x1_in        (ik_x1_in),
     .ik_x2_in        (ik_x2_in),
     .L2_0_in         (L2_0_in),
     .L2_1_in         (L2_1_in),
     .L2_2_in         (L2_2_in),
     .L2_3_in         (L2_3_in),
     .forward_x       (forward_x),
     .backward_x      (backward_x),
     .min_intv        (min_intv),
     .primary         (primary),   
     .read_trigger    (read_trigger),
     .cnt_a0          (cnt_k_a0),
     .cnt_a1          (cnt_k_a1),
     .cnt_a2          (cnt_k_a2),
     .cnt_a3          (cnt_k_a3),
     .cnt_b0          (cnt_k_b0),
     .cnt_b1          (cnt_k_b1),
     .cnt_b2          (cnt_k_b2),
     .cnt_b3          (cnt_k_b3),
     .cntl_a0         (cnt_l_a0),
     .cntl_a1         (cnt_l_a1),
     .cntl_a2         (cnt_l_a2),
     .cntl_a3         (cnt_l_a3),
     .cntl_b0         (cnt_l_b0),
     .cntl_b1         (cnt_l_b1),
     .cntl_b2         (cnt_l_b2),
     .cntl_b3         (cnt_l_b3),
     .DRAM_get        (DRAM_get),
     .match_pop_pulse (match_pop_pulse_new),
     //output
     .DRAM_valid      (BWT_DRAM_valid),
     .addr_k          (BWT_DRAM_addr_k), 
     .addr_l          (BWT_DRAM_addr_l), 
     .BWT_extend_done_pulse (BWT_extend_done_pulse),  
     .read_done       (read_done),
     .ret             (ret),
     .mem_size        (mem_size),
     .mem_out_info    (mem_out_info),
     .mem_out_x0      (mem_out_x0),
     .mem_out_x1      (mem_out_x1),
     .mem_out_x2      (mem_out_x2),
     .DRAM_get_same   (DRAM_get_same)
   );   
          
endmodule    

////////////////////////////////////////////////////
// Forward + Backward for 1 Read                  //
////////////////////////////////////////////////////

// presumably the output should be mem_out->x[012]

module PE_read(
   //input
   Clk_32UI,
   reset_read,
   CNT_flag,
   CNT_data,
   CNT_addr,
   query_in_valid,
   query_in_addr,
   query_in,
   ik_info_in,
   ik_x0_in,
   ik_x1_in,
   ik_x2_in,
   L2_0_in,
   L2_1_in,
   L2_2_in,
   L2_3_in,
   forward_x,
   backward_x,
   min_intv,
   primary,   
   read_trigger,		// control which to come in
   cnt_a0,
   cnt_a1,
   cnt_a2,
   cnt_a3,
   cnt_b0,
   cnt_b1,
   cnt_b2,
   cnt_b3,
   cntl_a0,
   cntl_a1,
   cntl_a2,
   cntl_a3,
   cntl_b0,
   cntl_b1,
   cntl_b2,
   cntl_b3,
   DRAM_get,					// trigger bwt_extend
   match_pop_pulse,				//??
   //output
   DRAM_valid,			//read valid
   addr_k,				//outputing the data request of k //the address is bwt_occ_intv(bwt, k)
   addr_l,
   BWT_extend_done_pulse,   //finish extend
   read_done,
   ret,
   mem_size,
   mem_out_info,
   mem_out_x0,
   mem_out_x1,
   mem_out_x2,
   DRAM_get_same		//saying the dram input is the same as before?
);    
   
   input Clk_32UI,reset_read;
   input CNT_flag;
   input [63:0] CNT_data;
   input [7:0] CNT_addr; 
   input       query_in_valid;
   input [6:0] query_in_addr;
   input [7:0] query_in;
   input [63:0] ik_x0_in,ik_x1_in,ik_x2_in,ik_info_in;
   input [63:0] L2_0_in,L2_1_in,L2_2_in,L2_3_in;
   input [6:0] forward_x, backward_x;
   input [6:0] min_intv;
   input [63:0] primary; 
   input read_trigger;
   input [31:0] cnt_a0,cnt_a1,cnt_a2,cnt_a3;
   input [63:0] cnt_b0,cnt_b1,cnt_b2,cnt_b3;
   input [31:0] cntl_a0,cntl_a1,cntl_a2,cntl_a3;
   input [63:0] cntl_b0,cntl_b1,cntl_b2,cntl_b3;
   input DRAM_get;
   input match_pop_pulse;

   wire Clk_32UI,reset_read;
   wire CNT_flag;
   wire [63:0] CNT_data;  
   wire [7:0]  CNT_addr; 
   wire query_in_valid;
   wire [6:0] query_in_addr;   
   wire [7:0] query_in;
   wire [63:0] ik_x0_in,ik_x1_in,ik_x2_in,ik_info_in;
   wire [63:0] L2_0_in,L2_1_in,L2_2_in,L2_3_in;
   wire [6:0] forward_x, backward_x;
   wire [6:0] min_intv; 
   wire [63:0] primary;   
   wire read_trigger;
   wire [31:0] cnt_a0,cnt_a1,cnt_a2,cnt_a3;
   wire [63:0] cnt_b0,cnt_b1,cnt_b2,cnt_b3;
   wire [31:0] cntl_a0,cntl_a1,cntl_a2,cntl_a3;
   wire [63:0] cntl_b0,cntl_b1,cntl_b2,cntl_b3;
   wire DRAM_get;
   wire match_pop_pulse;

   output DRAM_valid;
   output [31:0] addr_k, addr_l;
   output BWT_extend_done_pulse;   
   output read_done;
   output [31:0] ret;

   output [6:0] mem_size;
   output [63:0] mem_out_info,mem_out_x2,mem_out_x1,mem_out_x0; 
   output logic DRAM_get_same;
    
   logic DRAM_valid;
   logic [31:0] addr_k, addr_l;
   logic read_done;
   logic [31:0] ret;
   logic [6:0] mem_size;
   wire [63:0] mem_out_info,mem_out_x2,mem_out_x1,mem_out_x0;
 
   //Control signals for query_queue
   wire [7:0] query_curr;
   logic [6:0] query_addr;
   wire [6:0] BWT_query_addr;
   logic [6:0] forward_counter, backward_counter_i;
   logic query_we;
   logic forward_all_done;
   assign BWT_query_addr=(forward_all_done)?backward_counter_i:forward_counter;
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_read)
   	 begin
   	 	 query_addr <= 0;
   	 	 query_we <= 0;
   	 end
   	 else
   	 begin
   	 	 if(query_in_valid)
   	 	 begin
   	 	   query_we <= 1;
   	 	   query_addr <= query_in_addr;
   	 	 end
   	 	 else
   	 	 begin
   	 	 	 query_we <= 0;
   	 	 	 query_addr <= BWT_query_addr;
   	 	 end
   	 end
   end
   
   //Query_queue
   quary_queue quary_queue_u0(
	   .data (query_in),
	   .addr (query_addr),
	   .we   (query_we), 
	   .clk  (Clk_32UI),
	   .q    (query_curr)
   );
   
   //call BWT_extend module
   wire reset_BWT_extend;
   logic [63:0] BWT_k, BWT_l;
   wire BWT_extend_done;
   wire [63:0] ok0_x0, ok0_x1, ok0_x2;
   wire [63:0] ok1_x0, ok1_x1, ok1_x2;
   wire [63:0] ok2_x0, ok2_x1, ok2_x2;
   wire [63:0] ok3_x0, ok3_x1, ok3_x2;
   logic [63:0] ok0_info, ok1_info, ok2_info, ok3_info;
   logic [63:0] ik_x0,ik_x1,ik_x2,ik_info;
   logic [63:0] p_x0, p_x1, p_x2;   
   wire [63:0] ik_x0_new,ik_x1_new,ik_x2_new;
   wire BWT_extend_trigger;
   logic reset_BWT_extend_forward, reset_BWT_extend_backward;
   
   //p_x0[7] ---> addr_k[31]
   assign ik_x0_new = (forward_all_done) ? p_x0 : ik_x0; 
   assign ik_x1_new = (forward_all_done) ? p_x1 : ik_x1;
   assign ik_x2_new = (forward_all_done) ? p_x2 : ik_x2;
   assign BWT_extend_trigger = DRAM_get;
   assign reset_BWT_extend = reset_BWT_extend_forward & reset_BWT_extend_backward;
   BWT_extend BWT_ext_U0(
     .Clk_32UI         (Clk_32UI),  
     .reset_BWT_extend (reset_BWT_extend),
     .forward_all_done (forward_all_done),
     .CNT_flag         (CNT_flag),
     .CNT_data         (CNT_data),
     .CNT_addr         (CNT_addr),
     .trigger          (BWT_extend_trigger),
     .primary          (primary),
     .k                (BWT_k),
     .l                (BWT_l),
     .cnt_a0           (cnt_a0),
     .cnt_a1           (cnt_a1),
     .cnt_a2           (cnt_a2),
     .cnt_a3           (cnt_a3),
     .cnt_b0           (cnt_b0),
     .cnt_b1           (cnt_b1),
     .cnt_b2           (cnt_b2),
     .cnt_b3           (cnt_b3),
     .cntl_a0          (cntl_a0),         
     .cntl_a1          (cntl_a1),
     .cntl_a2          (cntl_a2),
     .cntl_a3          (cntl_a3),
     .cntl_b0          (cntl_b0),
     .cntl_b1          (cntl_b1),
     .cntl_b2          (cntl_b2),
     .cntl_b3          (cntl_b3),
     .L2_0             (L2_0_in),
     .L2_1             (L2_1_in),
     .L2_2             (L2_2_in),
     .L2_3             (L2_3_in),
     .ik_x0            (ik_x0_new),
     .ik_x1            (ik_x1_new),
     .ik_x2            (ik_x2_new),
     .BWT_extend_done  (BWT_extend_done),
     .ok0_x0           (ok0_x0),
     .ok0_x1           (ok0_x1),
     .ok0_x2           (ok0_x2),     
     .ok1_x0           (ok1_x0),     
     .ok1_x1           (ok1_x1),     
     .ok1_x2           (ok1_x2),     
     .ok2_x0           (ok2_x0),     
     .ok2_x1           (ok2_x1),     
     .ok2_x2           (ok2_x2),     
     .ok3_x0           (ok3_x0),     
     .ok3_x1           (ok3_x1),     
     .ok3_x2           (ok3_x2)    
   );
        
   //forward_initial pulse generator   
   logic read_trigger_q, forward_initial;
   always @(posedge Clk_32UI)
   begin
   	read_trigger_q <= read_trigger;
   	if(read_trigger==1 && read_trigger_q==0) forward_initial <= 1;
   	else forward_initial <= 0;
   end
   
   //(Forward) DRAM offset address calculation for k and l
   wire [63:0] forward_k,forward_k_temp,forward_l,forward_l_temp;
   assign forward_k_temp = ik_x1 -1;
   assign forward_l_temp = forward_k_temp + ik_x2; 
   assign forward_k = (forward_k_temp >= primary) ? forward_k_temp -1 : forward_k_temp;
   assign forward_l = (forward_l_temp >= primary) ? forward_l_temp -1 : forward_l_temp;
   
   //here p_x0
   //(Backward) DRAM offset address calculation for k and l  
   wire [63:0] backward_k,backward_k_temp,backward_l,backward_l_temp;
   assign backward_k_temp = p_x0 - 1;
   assign backward_l_temp = backward_k_temp + p_x2;
   assign backward_k = (backward_k_temp >= primary) ? backward_k_temp -1 : backward_k_temp;
   assign backward_l = (backward_l_temp >= primary) ? backward_l_temp -1 : backward_l_temp;
   
   //Address and valid for DRAM
     
   logic BWT_forward_extend_trigger_pulse, BWT_backward_extend_trigger_pulse;
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_read)
   	 begin
   	 	 DRAM_valid <= 0;
   	 	 addr_k <= 0;
   	 	 addr_l <= 0;
   	 	 DRAM_get_same <= 0;
   	 end
   	 else
   	 begin
       if(BWT_forward_extend_trigger_pulse) //(Forward)
       begin
       	 if((addr_k=={forward_k[34:7],4'b0}) && (addr_l=={forward_l[34:7],4'b0}))
       	 begin
       	   DRAM_valid <= 0;
       	   addr_k <= addr_k; 
     	   addr_l <= addr_l;
     	   BWT_k <= forward_k;
     	   BWT_l <= forward_l;
       	   DRAM_get_same <= 1;
         end
         else
         begin	
     	     DRAM_valid <= 1;
     	     addr_k <= {forward_k[34:7], 4'b0}; 
     	     addr_l <= {forward_l[34:7], 4'b0};
     	     BWT_k <= forward_k;
     	     BWT_l <= forward_l;
     	     DRAM_get_same <= 0;
     	   end
       end
       else if(BWT_backward_extend_trigger_pulse) //(Backward)
       begin
       	 if((addr_k=={backward_k[34:7],4'b0}) && (addr_l=={backward_l[34:7],4'b0}))
       	 begin
       	 	 DRAM_valid <= 0;
       	     addr_k <= addr_k; 
     	     addr_l <= addr_l;
     	     BWT_k <= backward_k;
     	     BWT_l <= backward_l;
       	     DRAM_get_same <= 1;
       	 end
       	 else
       	 begin
       	   DRAM_valid <= 1;
     	     addr_k <= {backward_k[34:7], 4'b0}; 
     	     addr_l <= {backward_l[34:7], 4'b0};
     	     BWT_k <= backward_k;
     	     BWT_l <= backward_l;
     	     DRAM_get_same <= 0;
     	   end
       end
       else
       begin
       	 DRAM_valid <= 0;
       	 addr_k <= addr_k;
       	 addr_l <= addr_l;
       	 BWT_k <= BWT_k;
     	   BWT_l <= BWT_l;
     	   DRAM_get_same <= 0;
       end
     end
   end
 
   wire BWT_extend_done_pulse;
   logic BWT_f_extend_done_pulse, BWT_b_extend_done_pulse, BWT_b_extend_done_pulse_q;
   assign BWT_extend_done_pulse =  BWT_f_extend_done_pulse | BWT_b_extend_done_pulse;

   //(Forward) BWT_forward_extend_trigger pulse generator   
   logic BWT_forward_extend_trigger, BWT_forward_extend_trigger_q;
   always @(posedge Clk_32UI)
   begin
   	BWT_forward_extend_trigger_q <= BWT_forward_extend_trigger;
   	if(BWT_forward_extend_trigger==1 && BWT_forward_extend_trigger_q==0) 
   	     BWT_forward_extend_trigger_pulse <= 1;
   	else BWT_forward_extend_trigger_pulse <= 0;
   end

   //(Forward) BWT_extend_done pulse generator
   logic BWT_f_extend_done_q;
   logic forward_all_done_q;   
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_BWT_extend_forward)
   	 begin
   	 	 BWT_f_extend_done_q <= 0;
   	 	 BWT_f_extend_done_pulse <= 0;
   	 end
   	 else 
   	 begin
   	 	if(!forward_all_done_q)
   	 	begin
   	 		BWT_f_extend_done_q <= BWT_extend_done;
   	 	  if(BWT_extend_done && (!BWT_f_extend_done_q)) BWT_f_extend_done_pulse <= 1;
   	 	  else BWT_f_extend_done_pulse <= 0;
   	 	end
     end
   end   
   
   //(Forward) Generate BWT control signals 	 
   logic BWT_f_extend_next;
   logic [1:0] forward_c;
   logic forward_done_1,forward_done_1_q,forward_done_1_qq;
   logic forward_done_3,forward_done_3_q;
   logic forward_done_4,forward_done_4_q,forward_done_4_pulse;
   logic BWT_f_extend_next_q,BWT_f_extend_next_qq;
   wire forward_done_or;
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_read)
   	 begin
   	 	 reset_BWT_extend_forward <= 0;   	 	 
   	 	 BWT_forward_extend_trigger <= 0;
   	 	 forward_counter <= 0;
   	 	 BWT_f_extend_next <= 0;
   	 	 forward_c <= 0;
   	 	 forward_done_1 <= 0; //A/C/G/T base
   	 	 forward_done_1_q <= 0;
   	 	 forward_done_1_qq <= 0;
   	 	 forward_done_3 <= 0; //101
   	 	 forward_done_3_q <= 0;
   	 	 forward_done_4 <= 0;
   	 	 forward_done_4_q <= 0;
   	 	 forward_done_4_pulse <= 0;
   	 end
   	 else 
   	 begin
   	 	 //forward counter for loop
   	 	 forward_done_1_q <= forward_done_1;
   	 	 forward_done_1_qq <= forward_done_1_q;
   	 	 forward_done_3_q <= forward_done_3;
   	 	 forward_done_4_q <= forward_done_4;
   	 	 if(forward_done_4 && (!forward_done_4_q)) forward_done_4_pulse <= 1;
   	 	 else forward_done_4_pulse <= 0;
   	 	  
   	 	 if(forward_initial)
   	 	 begin 
   	 	   forward_counter <= forward_x + 1;
   	 	   if(forward_x == 100)
   	 	   begin
   	 	   	 BWT_f_extend_next <= 0;
   	 	     forward_done_3 <= 1;
   	 	   end
   	 	   else BWT_f_extend_next <= 1;
   	 	 end
   	 	 else if(BWT_f_extend_done_pulse)
   	 	 begin
   	 	   forward_counter <= forward_counter + 1;           
   	 	   if(forward_counter == 100) 
   	 	   begin
   	 	   	 BWT_f_extend_next <= 0;
   	 	   	 forward_done_4 <= 1;
   	 	   end
   	 	   else
   	 	   begin 
   	 	     BWT_f_extend_next <= 1;
   	 	   end
   	     end
   	 	 else
   	 	 begin
   	 		 forward_counter <= forward_counter;
   	 		 BWT_f_extend_next <= 0;
   	 	 end
   	 	 // (forward) BWT trigger control
   	 	 BWT_f_extend_next_q <= BWT_f_extend_next;
   	 	 BWT_f_extend_next_qq <= BWT_f_extend_next_q;
   	 	 if(BWT_f_extend_next_qq)
   	 	 begin
   	 		 if(query_curr < 4) forward_c <= 3 - query_curr;
   	 	 end
   	 	 if(BWT_f_extend_next_qq && (!forward_done_or)) reset_BWT_extend_forward <= 0;
   	 	 else                    reset_BWT_extend_forward <= 1;
   	 	 
   	 	 if(BWT_f_extend_next_qq && (!forward_done_or))
   	 	 begin
   	 		 //reset_BWT_extend_forward <= 0;		   	 		
   	 		 if(query_curr < 4)
   	 		 begin
   	 			 //forward_c <= 3 - query_curr;
   	 		   BWT_forward_extend_trigger <= 1;
   	 		   forward_done_1 <= 0;
   	 		 end
   	 		 else
   	 		 begin
   	 			 forward_done_1 <= 1;                //not an A/C/G/T base
   	 			 BWT_forward_extend_trigger <= 0;
   	 		 end  	 		
   	 	 end
   	 	 else
   	 	 begin
   	 		 //reset_BWT_extend_forward <= 1;
   	 		 if(BWT_f_extend_done_pulse) BWT_forward_extend_trigger <= 0;
   	 		 else BWT_forward_extend_trigger <= BWT_forward_extend_trigger;
   	 	 end
   	 end
   end

   //(Forward) ok[c].x2  
   logic [63:0] forward_ok_temp;
   always@(*)
   begin
     case(forward_c)
     2'd0: forward_ok_temp <= ok0_x2;
     2'd1: forward_ok_temp <= ok1_x2;
     2'd2: forward_ok_temp <= ok2_x2;
     2'd3: forward_ok_temp <= ok3_x2;
     endcase
   end
   
   //(Forward) Update ik
   logic forward_done_2;
   logic [63:0] forward_ik_x0,forward_ik_x1,forward_ik_x2;
   logic [63:0] forward_ik_info;    	
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_read)
   	 begin
   	 	 forward_done_2 <= 0;
   	 end
   	 else
   	 begin  
   	 	 if(BWT_forward_extend_trigger && BWT_f_extend_done_pulse)
   	 	 begin
   	 		 if((forward_ok_temp!=ik_x2) && (forward_ok_temp < min_intv))
   	 		 begin
   	 			 forward_done_2 <= 1; // the interval size is too small to be extended further
   	 		 end
   	 		 else
   	 		 begin
   	 		   forward_ik_info <= forward_counter + 1;
   	 		   case(forward_c)
           2'd0: begin
        	   forward_ik_x0 <= ok0_x0;
   	         forward_ik_x1 <= ok0_x1;
   	         forward_ik_x2 <= ok0_x2;
           end
           2'd1: begin
   	         forward_ik_x0 <= ok1_x0;
   	         forward_ik_x1 <= ok1_x1;
   	         forward_ik_x2 <= ok1_x2;
           end
           2'd2: begin
   	         forward_ik_x0 <= ok2_x0;
   	         forward_ik_x1 <= ok2_x1;
   	         forward_ik_x2 <= ok2_x2;
           end
           2'd3: begin
   	         forward_ik_x0 <= ok3_x0;
   	         forward_ik_x1 <= ok3_x1;
   	         forward_ik_x2 <= ok3_x2;
           end
           endcase
         end 
       end
     end
   end
      	 		
   //(Forward) KV_push ik to curr queue
   logic forward_push_curr, forward_push_curr_q; 
   logic forward_push_curr_pulse, forward_push_curr_pulse_q;
   logic ik_wb,ik_wb_q,ik_wb_pulse,ik_wb_pulse_q;
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_BWT_extend_forward)
   	 begin
   	 	 forward_push_curr <= 0;
   	 	 forward_push_curr_q <= 0;
   	 	 forward_push_curr_pulse <= 0;
   	 	 forward_push_curr_pulse_q <= 0;
   	 	 ik_wb <= 0;
   	 	 ik_wb_q <= 0;
   	 end
   	 else
   	 begin
   	 	 forward_push_curr_q <= forward_push_curr;
   	 	 forward_push_curr_pulse_q <= forward_push_curr_pulse;
   	 	 if(BWT_forward_extend_trigger && BWT_extend_done && (forward_ok_temp!=ik_x2)) forward_push_curr <= 1;
       if(forward_done_1 || forward_done_3) forward_push_curr <= 1;  
       if((forward_push_curr==1) && (forward_push_curr_q==0)) forward_push_curr_pulse <= 1;
       else if((!forward_done_2) && (forward_done_4_pulse)) forward_push_curr_pulse <= 1;
       else forward_push_curr_pulse <= 0;
       
       ik_wb_q <= ik_wb;
       if(forward_done_or==0) ik_wb_pulse_q <= ik_wb_pulse;
       if(BWT_forward_extend_trigger && BWT_extend_done) ik_wb <= 1;
   	 	 if((ik_wb==1) && (ik_wb_q==0)) ik_wb_pulse <= 1;
       else ik_wb_pulse <= 0;

   	 end
   end
   
   //(Forward) Generate forward_done signals 
   logic forward_all_done_pulse,forward_all_done_pulse_q;
   assign forward_done_or = forward_done_1_qq | forward_done_2 | forward_done_3_q | forward_done_4_q;
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_read) 
   	 begin 
   	 	 forward_all_done <= 0;
   	 	 forward_all_done_q <= 0;
   	 	 forward_all_done_pulse <= 0;
   	 	 forward_all_done_pulse_q <= 0;
   	 end
   	 else
   	 begin
   	 	 forward_all_done_pulse_q <= forward_all_done_pulse;
   	 	 forward_all_done <= forward_done_or;
   	   forward_all_done_q <= forward_all_done;
   	   if(forward_all_done==1 && forward_all_done_q==0) forward_all_done_pulse <= 1;
   	   else forward_all_done_pulse <= 0;
   	 end
   end


   //(Backward) BWT_backward_extend_trigger pulse generator   
   logic BWT_backward_extend_trigger, BWT_backward_extend_trigger_q; 
   logic BWT_backward_extend_trigger_pulse_q;
   always @(posedge Clk_32UI)
   begin
   	BWT_backward_extend_trigger_q <= BWT_backward_extend_trigger;
   	if(BWT_backward_extend_trigger==1 && BWT_backward_extend_trigger_q==0) 
   	     BWT_backward_extend_trigger_pulse <= 1;
   	else BWT_backward_extend_trigger_pulse <= 0;
   end

   //(Backward) BWT_extend_done pulse generator
   logic BWT_b_extend_done, BWT_b_extend_done_q;
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_BWT_extend_backward)
   	 begin
   	 	 BWT_b_extend_done <= 0;
   	 	 BWT_b_extend_done_q <= 0;
   	 	 BWT_b_extend_done_pulse <= 0;
   	 	 BWT_b_extend_done_pulse_q <= 0;
   	 end
   	 else 
   	 begin
   	 	 BWT_b_extend_done_pulse_q <= BWT_b_extend_done_pulse;
   	 	 if(BWT_backward_extend_trigger) 
   	 	 begin
   	 		 BWT_b_extend_done <= BWT_extend_done;
   	 	   BWT_b_extend_done_q <= BWT_b_extend_done;
   	 	   if(BWT_b_extend_done && (!BWT_b_extend_done_q)) BWT_b_extend_done_pulse <= 1;
   	 	   else BWT_b_extend_done_pulse <= 0;
   	 	 end
     end
   end
   
   //(Backward) Generate BWT control signals 
   logic [6:0] backward_counter_j;
   logic BWT_b_iteration_i_next,BWT_b_iteration_i_next_q,BWT_b_iteration_i_next_qq;
   logic iteration_boundary,ambiguous;
   logic BWT_b_extend_next,BWT_b_extend_next_q,BWT_b_extend_next_qq;	 
   logic backward_j_done,backward_j_done_q,backward_j_done_qq;
   logic [7:0] backward_c;
   logic [6:0] last_size;
   wire [6:0] new_last_size;
   assign new_last_size = (last_size==0) ? 0 : last_size-1;
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_read)
   	 begin
   	 	 backward_counter_i <= 0;
   	 	 backward_counter_j <= 0;
  	 	 reset_BWT_extend_backward <= 0;   	 	 
   	 	 BWT_backward_extend_trigger <= 0;
   	 	 BWT_b_iteration_i_next <= 0;
   	 	 backward_c <= 0;
   	 	 iteration_boundary <= 0;
   	 	 ambiguous <= 0;
  	 	 BWT_b_extend_next <= 0;
       backward_j_done <= 0;
       read_done <= 0;
   	 end
   	 else
   	 begin
   	 	 // control signals for loop-i
   	 	 if(forward_all_done_pulse_q) 
   	 	 begin
   	 	 	 if(backward_x > 0)
   	 	 	 begin
   	 	 	 	 backward_counter_i <= backward_x - 1;
   	 	 	 	 BWT_b_iteration_i_next <= 1;
   	 	 	 	 iteration_boundary <= 0;
   	 	 	 end	 
   	 	 	 else
   	 	 	 begin
   	 	 	 	 backward_counter_i <= 0;
   	 	 	 	 BWT_b_iteration_i_next <= 1;
   	 	 	 	 iteration_boundary <= 1; //i = -1
   	 	 	 end
   	 	 end
   	 	 else if(backward_j_done_qq && (!read_done))
   	 	 begin
   	 	 	 if(last_size == 0)
   	 	 	 begin
   	 	 	 	read_done <= 1;
			    backward_counter_i <= backward_counter_i;
       		    BWT_b_iteration_i_next <= 0;
       		    iteration_boundary <= iteration_boundary;
       		 end 	 	 	 	 
   	 	 	 else if(backward_counter_i > 0) 
   	 	 	 begin
   	 	 	 	 backward_counter_i <= backward_counter_i - 1;
   	 	 	 	 BWT_b_iteration_i_next <= 1;
   	 	 	 	 iteration_boundary <= 0;
   	 	 	 end
   	 	 	 else 
   	 	 	 begin
   	 	 	 	 backward_counter_i <= backward_counter_i; // backward_counter_i =0 for i = -1
   	 	 	 	 BWT_b_iteration_i_next <= 1;
   	 	 	 	 iteration_boundary <= 1; //i = -1
   	 	 	 end   	 	 	 
       end
       else
       begin
       	 backward_counter_i <= backward_counter_i;
       	 BWT_b_iteration_i_next <= 0;
       	 iteration_boundary <= iteration_boundary;
       end
       
       //Control singals for loop-j
       backward_j_done_q <= backward_j_done;
       backward_j_done_qq <= backward_j_done_q;
       BWT_b_extend_next_q <= BWT_b_extend_next;
       BWT_b_extend_next_qq <= BWT_b_extend_next_q;
       BWT_b_iteration_i_next_q <= BWT_b_iteration_i_next;
       BWT_b_iteration_i_next_qq <= BWT_b_iteration_i_next_q;
       
       if(BWT_b_iteration_i_next_qq) //First iteration of loop-j  
       begin
       	 backward_counter_j <= 0;
       	 if(iteration_boundary)
       	 begin
       	 	 BWT_b_extend_next <= 1;
       	 end
       	 else
       	 begin
       	 	 if(query_curr < 4) 
       	 	 begin 
       	 	   ambiguous <= 0;
       	 	   BWT_b_extend_next <= 1;
       	 	   backward_c <= query_curr; //backward_c is the current character
       	 	 end
       	 	 else
       	 	 begin 
       	 	   ambiguous <= 1; // q[i] is an ambiguous base
       	 	   BWT_b_extend_next <= 1;
       	 	 end
       	 end
       end
       else if((BWT_b_extend_done_pulse_q) &&  (backward_counter_j <= new_last_size))
       begin
       	 if(backward_counter_j==(new_last_size))
       	 begin
       	 	 backward_j_done <= 1;
       	 	 BWT_b_extend_next <= 0;
       	 	 if(iteration_boundary) read_done <= 1;
       	 end
       	 else
       	 begin
       	 	 backward_j_done <= 0;
       	   BWT_b_extend_next <= 1;
         end
       	 backward_counter_j <= backward_counter_j + 1;
       end
       else
       begin
       	backward_counter_j <= backward_counter_j;
       	backward_j_done <= 0;
       	BWT_b_extend_next <= 0;
       	ambiguous <= ambiguous;
       end
       //control signals for BWT_extend
       if(BWT_b_extend_next_q)
       begin
       	 reset_BWT_extend_backward <= 0;
       	 BWT_backward_extend_trigger <= 1;
       end
       else
       begin
       	 reset_BWT_extend_backward <= 1;
       	 if(BWT_extend_done_pulse) BWT_backward_extend_trigger <= 0;
       	 else BWT_backward_extend_trigger <= BWT_backward_extend_trigger;
       end
     end
   end
   
   //(Backward) ok[c]  
   logic [63:0] ok_b_temp_x0,ok_b_temp_x1,ok_b_temp_x2;
   always@(*)
   begin
     case(backward_c)
     2'd0: 
     begin 
     	 ok_b_temp_x0 <= ok0_x0;
     	 ok_b_temp_x1 <= ok0_x1;
     	 ok_b_temp_x2 <= ok0_x2;
     end     	 
     2'd1:
     begin 
     	 ok_b_temp_x0 <= ok1_x0;
     	 ok_b_temp_x1 <= ok1_x1;
     	 ok_b_temp_x2 <= ok1_x2;
     end
     2'd2:
     begin 
     	 ok_b_temp_x0 <= ok2_x0;
     	 ok_b_temp_x1 <= ok2_x1;
     	 ok_b_temp_x2 <= ok2_x2;
     end
     2'd3:
     begin 
     	 ok_b_temp_x0 <= ok3_x0;
     	 ok_b_temp_x1 <= ok3_x1;
     	 ok_b_temp_x2 <= ok3_x2;
     end
     endcase
   end
   
   //(Backward) new_i = i+1
   wire [6:0] new_i;
   assign new_i = (iteration_boundary)? 0 : (backward_counter_i+1); 
   
   //(backward) update and push control
   logic mem_push, backward_push;
   logic [6:0] new_size;
   logic [31:0] last_mem_info;
   logic [63:0] backward_ik_x0,backward_ik_x1,backward_ik_x2;
   logic [63:0] backward_ik_info;
   logic [63:0] last_token_x2;
   logic [63:0] p_info;	
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_BWT_extend_backward)
   	 begin
   	 	 mem_push <= 0;
   	 	 backward_push <= 0;
   	 end
   	 else
   	 begin  
   	 	 if(BWT_backward_extend_trigger && BWT_b_extend_done_pulse)
   	 	 begin
   	 	 	 if((ambiguous==1) || (iteration_boundary==1) || (ok_b_temp_x2 < min_intv))
   	 	 	 begin
   	 	 	 	 if(new_size==0)
   	 	 	   begin
   	 	 	 	   if((mem_size == 0) || (new_i < last_mem_info))
   	 	 	 	   begin
   	 	 	 	 	   backward_ik_x2 <= p_x2;
   	 	 	 	 	   backward_ik_x1 <= p_x1;
   	 	 	 	 	   backward_ik_x0 <= p_x0;
   	 	 	 	 	   backward_ik_info[31:0] <= p_info[31:0];
   	 	 	 	 	   backward_ik_info[63:32] <= new_i;
   	 	 	 	 	   mem_push <= 1;
   	 	 	 	   end
   	 	 	   end
   	 	 	 end
   	 	 	 else if((new_size==0) || (ok_b_temp_x2 != last_token_x2))
   	 	   begin
   	 	 	   case(backward_c)
   	 	 	   2'd0: ok0_info <= p_info;
   	 	 	   2'd1: ok1_info <= p_info;
   	 	 	   2'd2: ok2_info <= p_info;
   	 	 	   2'd3: ok3_info <= p_info;
   	 	     endcase
   	 	     backward_push <= 1;
   	 	   end
   	 	 end
   	 	 else
   	 	 begin
   	 	 	 mem_push <= 0;
   	 	 	 backward_push <= 0;
   	 	 end
   	 end
   end
   	 	 	 	 
   //(Backward) curr Push pulse generator   
   logic backward_push_q;
   logic backward_push_pulse,backward_push_pulse_q;
   always @(posedge Clk_32UI)
   begin
   	backward_push_q <= backward_push;
   	backward_push_pulse_q <= backward_push_pulse;
   	if(backward_push==1 && backward_push_q==0) backward_push_pulse <= 1;
   	else backward_push_pulse <= 0;
   end
   
   //(Backward) mem Push pulse generator   
   logic mem_push_q,mem_push_pulse,mem_push_pulse_q;
   always @(posedge Clk_32UI)
   begin
   	mem_push_q <= mem_push;
   	mem_push_pulse_q <= mem_push_pulse;
   	if(mem_push==1 && mem_push_q==0) mem_push_pulse <= 1;
   	else mem_push_pulse <= 0;
   end

//below is the curr and mem_out control.
//store the result into curr and mem_out according to the control signal
		
////////////////////////////////////////////////////
// KV_push Memory                                 //
////////////////////////////////////////////////////
   
   //control for curr queue
   logic curr_we;
   logic [6:0] curr_addr;
   logic [6:0] forward_size_n;
   logic [6:0] backward_addr_w, backward_addr_r;
   logic [63:0] curr_in_x0, curr_in_x1, curr_in_x2;
   logic [63:0] curr_in_info;
   wire [63:0] curr_out_x0, curr_out_x1, curr_out_x2;
   wire [63:0] curr_out_info;
   always @(posedge Clk_32UI)
   begin
  	 if(!reset_read)
   	 begin
   	 	 curr_we <= 0;
   	 	 curr_addr <= 0;
   	 	 forward_size_n <= 0;
   	 	 last_size <= 0;
   	 	 new_size <= 0;
   	 	 backward_addr_w <= 0;
   	 	 backward_addr_r <= 0;
   	 	 p_info <= 0;
   	 	 p_x0 <= 0;
   	 	 p_x1 <= 0;
   	 	 p_x2 <= 0;
   	 	 last_token_x2 <= 0;
   	 end
   	 else
   	 begin
   	 	 // (Forward) KV_push to curr queue
   	 	 if(forward_push_curr_pulse)
   	 	 begin
   	 	 	 curr_we <= 1;
   	 	 	 curr_addr <= forward_size_n;
   	 	 	 curr_in_info <= ik_info;
   	 	 	 curr_in_x0 <= ik_x0;
   	 	 	 curr_in_x1 <= ik_x1;
   	 	 	 curr_in_x2 <= ik_x2;
   	 	 	 forward_size_n <= forward_size_n + 1;
   	 	 end
   	 	 if(forward_initial)
   	 	 begin
   	 	 	 ik_info <= ik_info_in;
   	 	 	 ik_x0 <= ik_x0_in;
   	 	 	 ik_x1 <= ik_x1_in;
   	 	 	 ik_x2 <= ik_x2_in;
   	 	 end
   	 	 if(forward_push_curr_pulse_q)
   	 	 begin
   	 	 	 curr_we <= 0; 
   	 	 end
   	 	 if(ik_wb_pulse_q)
   	 	 begin
   	 	 	 ik_info <= forward_ik_info;
   	 	 	 ik_x0 <= forward_ik_x0;
   	 	 	 ik_x1 <= forward_ik_x1;
   	 	 	 ik_x2 <= forward_ik_x2; 
   	 	 end
		 if(mem_push)    // (Backward) update ik with mem_push
   	 	 begin
   	 	 	ik_info <= backward_ik_info;
   	 	 	ik_x0 <= backward_ik_x0;
   	 	 	ik_x1 <= backward_ik_x1;
   	 	 	ik_x2 <= backward_ik_x2;
   	 	 end    
       // (Backward) KV_push & Pop of curr queue
   	 	 if(forward_all_done_pulse)
   	 	 begin
   	 	 	 new_size <= 0;
   	 	 	 last_size <= forward_size_n;
   	 	 	 backward_addr_w <= forward_size_n-1;
   	 	 	 backward_addr_r <= forward_size_n-1;
   	 	 	 curr_we <= 0;
   	 	 	 ret <= curr_in_info[31:0];
   	 	 end
   	 	 if(backward_j_done_q)
   	 	 begin
   	 	 	 new_size <= 0;
   	 	 	 last_size <= new_size;
   	 	 	 backward_addr_w <= forward_size_n-1;
   	 	 	 backward_addr_r <= forward_size_n-1;
   	 	 end
   	 	 if(BWT_b_extend_next) // (Backward) Pop for *p
   	 	 begin
   	 	 	 curr_we <= 0;
   	 	 	 curr_addr <= backward_addr_r;
   	 	 	 backward_addr_r <= backward_addr_r -1;
   	 	 end
   	 	 if(BWT_b_extend_next_qq)
   	 	 begin
   	 	 	 p_x0 <= curr_out_x0;
   	 	 	 p_x1 <= curr_out_x1;
   	 	 	 p_x2 <= curr_out_x2;
   	 	 	 p_info <= curr_out_info;
   	 	 end
   	 	 if(backward_push) // (Backward) push for intv
   	 	 begin
   	 	 	 curr_we <= 1;
   	 	 	 curr_addr <= backward_addr_w;
   	 	 	 curr_in_info <= p_info;
   	 	 	 curr_in_x0 <= ok_b_temp_x0;
   	 	 	 curr_in_x1 <= ok_b_temp_x1;
   	 	 	 curr_in_x2 <= ok_b_temp_x2;
   	 	 	 last_token_x2 <= ok_b_temp_x2;
   	 	 	 new_size <= new_size +1;
   	 	 end
   	 	 if(backward_push_q)
   	 	 begin
   	 	 	 curr_we <= 0;
   	 	 	 backward_addr_w <= backward_addr_w - 1;
   	 	 end
   	 end
   end
   
   // curr queue   
   push_mem curr(
	   .data ({curr_in_info,curr_in_x2,curr_in_x1,curr_in_x0}),
	   .addr (curr_addr),
	   .we   (curr_we), 
	   .clk  (Clk_32UI),
	   .q    ({curr_out_info,curr_out_x2,curr_out_x1,curr_out_x0})
   );

   //control for mem queue
   logic mem_we;
   logic [6:0] mem_addr;
   logic [63:0] mem_in_x0,mem_in_x1,mem_in_x2;
   logic [63:0] mem_in_info;
   always @(posedge Clk_32UI)
   begin
  	 if(!reset_read)
   	 begin
   	 	 mem_addr <= 0;
   	 	 mem_we <= 0;
   	 	 mem_size <= 0;
   	 	 last_mem_info <= 0;
   	 end
   	 else
   	 begin
   	 	 if(mem_push_pulse)
   	 	 begin
   	 	 	 mem_we <= 1;
   	 	 	 mem_in_info <= backward_ik_info;
   	 	 	 mem_in_x0 <= backward_ik_x0;
   	 	 	 mem_in_x1 <= backward_ik_x1;
   	 	 	 mem_in_x2 <= backward_ik_x2;
   	 	 	 last_mem_info <= backward_ik_info[63:32];
   	 	 end
   	 	 else if(mem_push_pulse_q)
   	 	 begin
   	 	 	 mem_we <= 0;
   	 	   mem_addr <= mem_addr + 1;
   	 	   mem_size <= mem_size + 1;
   	 	 end
   	 	 else if(match_pop_pulse)
   	 	 begin
   	 	 	 mem_we <= 0;
         mem_addr <= mem_addr - 1;   	 	 	 
   	 	 end
   	 	 else
   	 	 begin
   	 	 	 mem_we <= 0;
   	 	 	 mem_addr <= mem_addr;
   	 	 end
   	 end
   end
   
   // mem queue  
   push_mem prev(
	   .data ({mem_in_info,mem_in_x2,mem_in_x1,mem_in_x0}),
	   .addr (mem_addr),
	   .we   (mem_we), 
	   .clk  (Clk_32UI),
	   .q    ({mem_out_info,mem_out_x2,mem_out_x1,mem_out_x0})
   );   

endmodule

////////////////////////////////////////////////////
// BWT_extend                                     //
////////////////////////////////////////////////////

// input cnt+cntl(cntk & cntl), cntk is the 4 consecutive value from p : p = bwt_occ_intv(bwt, k);

// cntb0,1,2,3 are 4 consecutive p value request from bwt. calculating b from bwt
// for (x = 0; p < end; ++p) x += __occ_aux4(bwt, *p)

// ik the values and boundarys: ik_x0/ik_x1(start), + ik_x2 (end). also used when calculting ok
//k and l are boundary, should just be ik_x0 & ik_x0+ik_x2

//the request should just be the position of p = bwt_occ_intv(bwt, k);
// each calculation calculates the position of bwt's k and bwt's l, 
//and request the value following 4*64(4*cnt_a0)  +  use another 8 consecutive 32 BITS value. so in total 1CLs for 1 request.
//memcpy(cnt, p, 4 * sizeof(bwtint_t)); 
//	p +=  8
//	end = p + 4;
//L2:cumulative count
//request k and l ????????/

//output: ok

module BWT_extend(
   Clk_32UI,  
   reset_BWT_extend,
   forward_all_done,
   CNT_flag,
   CNT_data,
   CNT_addr,
   trigger,
   primary,
   k,
   l,
   cnt_a0,
   cnt_a1,
   cnt_a2,
   cnt_a3,
   cnt_b0,
   cnt_b1,
   cnt_b2,
   cnt_b3,
   cntl_a0,
   cntl_a1,
   cntl_a2,
   cntl_a3,
   cntl_b0,
   cntl_b1,
   cntl_b2,
   cntl_b3,
   L2_0,
   L2_1,
   L2_2,
   L2_3,
   ik_x0,
   ik_x1,
   ik_x2,
   BWT_extend_done,
   ok0_x0,
   ok0_x1,
   ok0_x2,     
   ok1_x0,     
   ok1_x1,     
   ok1_x2,     
   ok2_x0,     
   ok2_x1,     
   ok2_x2,     
   ok3_x0,     
   ok3_x1,     
   ok3_x2    
);

   input Clk_32UI;
   input reset_BWT_extend;
   input forward_all_done; //wether it is forward or backward
   input CNT_flag;
   input [63:0] CNT_data;
   input [7:0]  CNT_addr;
   input trigger;
   input [63:0] primary;
   input [63:0] k,l;
   input [31:0] cnt_a0,cnt_a1,cnt_a2,cnt_a3;
   input [63:0] cnt_b0,cnt_b1,cnt_b2,cnt_b3;
   input [31:0] cntl_a0,cntl_a1,cntl_a2,cntl_a3;
   input [63:0] cntl_b0,cntl_b1,cntl_b2,cntl_b3;
   input [63:0] L2_0, L2_1, L2_2, L2_3;
   input [63:0] ik_x0, ik_x1, ik_x2;
   
   output BWT_extend_done;
   output [63:0] ok0_x0, ok0_x1, ok0_x2;
   output [63:0] ok1_x0, ok1_x1, ok1_x2;
   output [63:0] ok2_x0, ok2_x1, ok2_x2;
   output [63:0] ok3_x0, ok3_x1, ok3_x2;
   
   wire Clk_32UI;
   wire reset_BWT_extend;
   wire forward_all_done;
   wire CNT_flag;
   wire [63:0] CNT_data;
   wire [7:0]  CNT_addr;
   wire trigger;
   wire [63:0] primary;
   wire [63:0] k,l;
   wire [31:0] cnt_a0,cnt_a1,cnt_a2,cnt_a3;
   wire [63:0] cnt_b0,cnt_b1,cnt_b2,cnt_b3;
   wire [31:0] cntl_a0,cntl_a1,cntl_a2,cntl_a3;
   wire [63:0] cntl_b0,cntl_b1,cntl_b2,cntl_b3;
   wire [63:0] L2_0, L2_1, L2_2, L2_3;
   wire [63:0] ik_x0, ik_x1, ik_x2;
      
   wire finish_k, finish_l, BWT_2occ4_finish;
   wire [31:0] cnt_tk_out0,cnt_tk_out1,cnt_tk_out2,cnt_tk_out3;
   wire [31:0] cnt_tl_out0,cnt_tl_out1,cnt_tl_out2,cnt_tl_out3;

   logic BWT_extend_done;
   logic [63:0] ok0_x0, ok0_x1, ok0_x2;
   logic [63:0] ok1_x0, ok1_x1, ok1_x2;
   logic [63:0] ok2_x0, ok2_x1, ok2_x2;
   logic [63:0] ok3_x0, ok3_x1, ok3_x2;   
   
   BWT_OCC4 BWT_OCC4_k(
     .Clk_32UI       (Clk_32UI),
     .reset_BWT_occ4 (reset_BWT_extend),
     .CNT_flag       (CNT_flag),
     .CNT_data       (CNT_data),
     .CNT_addr       (CNT_addr),
     .k              (k),
     .cnt_a0         (cnt_a0),
     .cnt_a1         (cnt_a1),
     .cnt_a2         (cnt_a2),
     .cnt_a3         (cnt_a3),
     .cnt_b0         (cnt_b0),
     .cnt_b1         (cnt_b1),
     .cnt_b2         (cnt_b2),
     .cnt_b3         (cnt_b3),
     .trigger        (trigger),
     .cnt_out0       (cnt_tk_out0),
     .cnt_out1       (cnt_tk_out1),
     .cnt_out2       (cnt_tk_out2),
     .cnt_out3       (cnt_tk_out3),
     .x_new_finish   (finish_k) 
   );

   BWT_OCC4 BWT_OCC4_l(
     .Clk_32UI       (Clk_32UI),
     .reset_BWT_occ4 (reset_BWT_extend),
     .CNT_flag       (CNT_flag),
     .CNT_data       (CNT_data),
     .CNT_addr       (CNT_addr),
     .k              (l),
     .cnt_a0         (cntl_a0),
     .cnt_a1         (cntl_a1),
     .cnt_a2         (cntl_a2),
     .cnt_a3         (cntl_a3),
     .cnt_b0         (cntl_b0),
     .cnt_b1         (cntl_b1),
     .cnt_b2         (cntl_b2),
     .cnt_b3         (cntl_b3),
     .trigger        (trigger),
     .cnt_out0       (cnt_tl_out0),
     .cnt_out1       (cnt_tl_out1),
     .cnt_out2       (cnt_tl_out2),
     .cnt_out3       (cnt_tl_out3),
     .x_new_finish   (finish_l)
   );

   assign BWT_2occ4_finish = finish_k & finish_l;
   logic [2:0] BWT_extend_counter;
   
   always @(posedge Clk_32UI)
   begin
   	 if(!reset_BWT_extend)
   	 begin
   	   BWT_extend_counter <= 0;
   	   BWT_extend_done <= 0;
   	 end
   	 else
   	 begin
   	 	 if(BWT_2occ4_finish && (BWT_extend_counter!=5)) BWT_extend_counter <= BWT_extend_counter + 1;
   	 	 else BWT_extend_counter <= BWT_extend_counter;
   	 	 if(BWT_extend_counter==4) BWT_extend_done <= 1;
   	 end
   end
		 
	 wire [63:0] ok0_temp,ok1_temp,ok2_temp,ok3_temp;
	 assign ok0_temp = L2_0 + cnt_tk_out0 + 1; 
	 assign	ok1_temp = L2_1 + cnt_tk_out1 + 1;
	 assign	ok2_temp = L2_2 + cnt_tk_out2 + 1;
	 assign	ok3_temp = L2_3 + cnt_tk_out3 + 1;
	 
	 
	 //input wire ik_x0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 wire [63:0] ik_x1_x0,ik_temp;
	 assign ik_x1_x0 = (forward_all_done) ? ik_x0 : ik_x1;
	 assign ik_temp = ik_x1_x0 + ik_x2 - 1;
	 
	 wire [63:0] x2_new, x1_new, x0_new;
	 assign x2_new = ok3_x0 + ok3_x2;
	 assign x1_new = ok2_x0 + ok2_x2;
	 assign x0_new = ok1_x0 + ok1_x2;
	 
   always @(posedge Clk_32UI)
   begin
   	 if(BWT_2occ4_finish)
   	 begin
   	   ok0_x2 <= cnt_tl_out0 - cnt_tk_out0;
   	   ok1_x2 <= cnt_tl_out1 - cnt_tk_out1;
   	   ok2_x2 <= cnt_tl_out2 - cnt_tk_out2;
   	   ok3_x2 <= cnt_tl_out3 - cnt_tk_out3;   	
   	   if(forward_all_done)
   	   begin
   	   	 ok0_x0 <= ok0_temp;
   	     ok1_x0 <= ok1_temp;
   	     ok2_x0 <= ok2_temp;
   	     ok3_x0 <= ok3_temp;
   	     if((ik_x0<=primary) && (ik_temp >= primary)) ok3_x1 <= ik_x1 + 1;
   	     else                                         ok3_x1 <= ik_x1; 
   	     ok2_x1 <= x2_new;
   	     ok1_x1 <= x1_new;
   	     ok0_x1 <= x0_new;
   	   end
   	   else
   	   begin 
   	     ok0_x1 <= ok0_temp;
   	     ok1_x1 <= ok1_temp;
   	     ok2_x1 <= ok2_temp;
   	     ok3_x1 <= ok3_temp;
   	     if((ik_x1<=primary) && (ik_temp >= primary)) ok3_x0 <= ik_x0 + 1;
   	     else                                         ok3_x0 <= ik_x0; 
   	     ok2_x0 <= x2_new;
   	     ok1_x0 <= x1_new;
   	     ok0_x0 <= x0_new;
   	   end
   	 end
   end
endmodule

///////////////////////////////////////////////////////////////////////////////////
//true_dpram_sclk is for CNT table
//push_mem is to store result, mem_out
//quary queue should be the read?
///////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////
// BWT_OCC4                                       //
////////////////////////////////////////////////////

//output is the the output of bwt_occ4: cnt[4]

module BWT_OCC4(
   Clk_32UI,
   reset_BWT_occ4,
   CNT_flag,
   CNT_data,
   CNT_addr,
   k,
   cnt_a0, //	all these are addresses
   cnt_a1, // 
   cnt_a2, //		
   cnt_a3, //	should be the 4 consecutive value from p=(bwt+ k>>7<<4)
   cnt_b0, //
   cnt_b1, //
   cnt_b2, //
   cnt_b3, //	these values are p+1 +2 +3 +4blabla
   trigger,
   cnt_out0,
   cnt_out1,
   cnt_out2,
   cnt_out3,
   x_new_finish
);

   input Clk_32UI;
   input reset_BWT_occ4;
   input CNT_flag;
   input [63:0] CNT_data;
   inout [7:0] CNT_addr;
   input [63:0] k;
   input [31:0] cnt_a0,cnt_a1,cnt_a2,cnt_a3;
   input [63:0] cnt_b0,cnt_b1,cnt_b2,cnt_b3;
   input trigger;
   output [31:0] cnt_out0,cnt_out1,cnt_out2,cnt_out3;
   output x_new_finish;
   
   wire Clk_32UI;
   wire reset_BWT_occ4;
   wire CNT_flag;
   wire [63:0] CNT_data;
   wire [7:0]  CNT_addr;
   wire [63:0] k;
   logic [31:0] cnt_a0,cnt_a1,cnt_a2,cnt_a3;
   wire [63:0] cnt_b0,cnt_b1,cnt_b2,cnt_b3;

   wire [63:0] x_new_logic;
   logic [31:0] cnt_out0,cnt_out1,cnt_out2,cnt_out3;
   logic trigger,x_new_finish;
   wire [2:0] loop_times;
   wire [31:0] mask;
   logic[31:0] tmp;

   assign loop_times = k[6:4];
   assign mask = ~((1<<((~k & 15)<<1)) - 1);
     
   always@( *)
   begin
   	case(loop_times)
   	  3'd0: tmp = cnt_b0[31:0] & mask;
   	  3'd1: tmp = cnt_b0[63:32] & mask;
   	  3'd2: tmp = cnt_b1[31:0] & mask;
   	  3'd3: tmp = cnt_b1[63:32] & mask;
   	  3'd4: tmp = cnt_b2[31:0] & mask;
   	  3'd5: tmp = cnt_b2[63:32] & mask;
   	  3'd6: tmp = cnt_b3[31:0] & mask;
   	  3'd7: tmp = cnt_b3[63:32] & mask;
   	  default: tmp = 0;
    endcase
   end

  logic [2:0] mem_counter;

   always @(posedge Clk_32UI)
   begin
   	if(!reset_BWT_occ4)
   	begin
   		mem_counter <= 0;
   	end
   	else 
   	begin
   	  if(trigger && (mem_counter!=7)) mem_counter <= mem_counter + 1;
   	end    		
   end
   

   logic a_wr_0,b_wr_0;
   logic [7:0] a_addr_0,b_addr_0;
   logic [31:0] a_din_0,b_din_0;
   wire [31:0] a_dout_0,b_dout_0;

   true_dpram_sclk mem_P0(
   .clk(Clk_32UI),
   .we_a(a_wr_0),
   .addr_a(a_addr_0),
   .data_a(a_din_0),
   .q_a(a_dout_0),     
   .we_b(b_wr_0),
   .addr_b(b_addr_0),
   .data_b(b_din_0),
   .q_b(b_dout_0)
   );
   
   logic a_wr_1,b_wr_1;
   logic [7:0] a_addr_1,b_addr_1;
   logic [31:0] a_din_1,b_din_1;
   wire [31:0] a_dout_1,b_dout_1;

   true_dpram_sclk mem_P1(
   .clk(Clk_32UI),
   .we_a(a_wr_1),
   .addr_a(a_addr_1),
   .data_a(a_din_1),
   .q_a(a_dout_1),     
   .we_b(b_wr_1),
   .addr_b(b_addr_1),
   .data_b(b_din_1),
   .q_b(b_dout_1)
   );
   
   always @(posedge Clk_32UI)
   begin
   	if(CNT_flag)
   	begin
   		a_wr_0 <= 1;
   		b_wr_0 <= 1;
   		a_addr_0 <= CNT_addr;
   		b_addr_0 <= CNT_addr+1;
   		a_din_0 <= CNT_data[31:0];
   		b_din_0 <= CNT_data[63:32];
   		a_wr_1 <= 1;
   		b_wr_1 <= 1;
   		a_addr_1 <= CNT_addr;
   		b_addr_1 <= CNT_addr+1;
   		a_din_1 <= CNT_data[31:0];
   		b_din_1 <= CNT_data[63:32];
   	end
   	else
   	begin
   		a_wr_0 <= 0;
   		b_wr_0 <= 0;
   		a_wr_1 <= 0;
   		b_wr_1 <= 0;  		
   		case(mem_counter)
   		4'd0:begin
   			a_addr_0 <= tmp[7:0];
   			b_addr_0 <= tmp[15:8];
   			a_addr_1 <= tmp[23:16];
   			b_addr_1 <= tmp[31:24];
   		end  
   		4'd1:begin
   			a_addr_0 <= cnt_b0[7:0];
   			b_addr_0 <= cnt_b0[15:8];
   			a_addr_1 <= cnt_b0[23:16];
   			b_addr_1 <= cnt_b0[31:24];
   		end
   		4'd2:begin
   			a_addr_0 <= cnt_b0[39:32];
   			b_addr_0 <= cnt_b0[47:40];
   			a_addr_1 <= cnt_b0[55:48];
   			b_addr_1 <= cnt_b0[63:56];
   		end
   		4'd3:begin
   			a_addr_0 <= cnt_b1[7:0];
   			b_addr_0 <= cnt_b1[15:8];
   			a_addr_1 <= cnt_b1[23:16];
   			b_addr_1 <= cnt_b1[31:24];
   		end
   		4'd4:begin
   			a_addr_0 <= cnt_b1[39:32];
   			b_addr_0 <= cnt_b1[47:40];
   			a_addr_1 <= cnt_b1[55:48];
   			b_addr_1 <= cnt_b1[63:56];
   		end   		
   		4'd5:begin
   			a_addr_0 <= cnt_b2[7:0];
   			b_addr_0 <= cnt_b2[15:8];
   			a_addr_1 <= cnt_b2[23:16];
   			b_addr_1 <= cnt_b2[31:24];
   		end
   		4'd6:begin
   			a_addr_0 <= cnt_b2[39:32];
   			b_addr_0 <= cnt_b2[47:40];
   			a_addr_1 <= cnt_b2[55:48];
   			b_addr_1 <= cnt_b2[63:56];
   		end  
   		4'd7:begin
   			a_addr_0 <= cnt_b3[7:0];
   			b_addr_0 <= cnt_b3[15:8];
   			a_addr_1 <= cnt_b3[23:16];
   			b_addr_1 <= cnt_b3[31:24];
   		end
    	endcase
   	end
   end
 
   logic[31:0] x_logic;
   logic x_new_finish; 
   logic [2:0] mem_counter_q,mem_counter_q1,mem_counter_q2;  

   
   always@(posedge Clk_32UI)
   begin
   	if(!reset_BWT_occ4) begin
   		mem_counter_q <= 0;
   		mem_counter_q1 <=0;
   	end
   	else begin
   		mem_counter_q <= mem_counter;
   		mem_counter_q1 <= mem_counter_q;
   		mem_counter_q2 <= mem_counter_q1;
   	end
   end
    
   always @(posedge Clk_32UI)
   begin
   	if(!reset_BWT_occ4) begin
   		x_logic <= 0;
   		x_new_finish <= 0;
   	end
   	else
   	begin
   		//if(mem_counter_q1==7) x_new_finish <= 1;
   		if((mem_counter_q != 0) && ((loop_times > mem_counter_q2) || (mem_counter_q1==0)))
   		begin
   		  x_logic <= x_logic + a_dout_0 + b_dout_0 + a_dout_1 + b_dout_1; 
		  // in the loop, each time add 4 values out.
   		  if(loop_times==mem_counter_q1) x_new_finish <= 1;
   		end
   	  else 
   	  begin
   	    x_logic <= x_logic;
   	  end
   	end
   end

   assign x_new_logic = x_logic - (~k & 15); 
   
   always @(posedge Clk_32UI)
   begin
   		cnt_out0 <= cnt_a0 + x_new_logic[7:0];
   		cnt_out1 <= cnt_a1 + x_new_logic[15:8];
   		cnt_out2 <= cnt_a2 + x_new_logic[23:16];
   		cnt_out3 <= cnt_a3 + x_new_logic[31:24];    			
   end  
      
endmodule

////////////////////////////////////////////////////
// 256x32 Dual-Port Block RAM                     //
////////////////////////////////////////////////////

module true_dpram_sclk
(
	input reg [31:0] data_a, data_b,
	input reg [7:0] addr_a, addr_b,
	input reg we_a, we_b, clk,
	output logic [31:0] q_a, q_b
);
	// Declare the RAM variable
	logic [31:0] ram[255:0];
	
	// Port A
	always @ (posedge clk)
	begin
		if (we_a) 
		begin
			ram[addr_a] <= data_a;
			q_a <= data_a;
		end
		else 
		begin
			q_a <= ram[addr_a];
		end
	end
	
	// Port B
	always @ (posedge clk)
	begin
		if (we_b)
		begin
			ram[addr_b] <= data_b;
			q_b <= data_b;
		end
		else
		begin
			q_b <= ram[addr_b];
		end
	end
endmodule

////////////////////////////////////////////////////
// 101 x 256 Block RAM                            //
////////////////////////////////////////////////////

module push_mem
(
	input reg [255:0] data,
	input reg [6:0] addr,
	input reg we, clk,
	output logic [255:0] q
);
	// Declare the RAM variable
	logic [255:0] ram[127:0];
	
	always @ (posedge clk)
	begin
		if (we) 
		begin
			ram[addr] <= data;
			q <= data;
		end
		else 
		begin
			q <= ram[addr];
		end
	end
	
endmodule

////////////////////////////////////////////////////
// 101 x 8bit Block RAM                           //
////////////////////////////////////////////////////

module quary_queue
(
	input reg [7:0] data,
	input reg [6:0] addr,
	input reg we, clk,
	output logic [7:0] q
);
	// Declare the RAM variable
	logic [7:0] ram[127:0];
	
	always @ (posedge clk)
	begin
		if (we) 
		begin
			ram[addr] <= data;
			q <= data;
		end
		else 
		begin
			q <= ram[addr];
		end
	end
endmodule
