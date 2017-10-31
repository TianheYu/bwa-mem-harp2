import ccip_if_pkg::*;
import ccip_feature_list_pkg::*;

module afu_io#(
  parameter NEXT_DFH_BYTE_OFFSET = 0
)
(
	input  wire                             clk,
	input  wire                             spl_reset,

	// AFU TX read request
	output reg                              afu_tx_rd_valid,
	output t_ccip_c0_ReqMemHdr              afu_tx_rd_hdr,

	// AFU TX write request
	output reg                              afu_tx_wr_valid,
	output t_ccip_c1_ReqMemHdr              afu_tx_wr_hdr,
	output reg [511:0]						afu_tx_data,

	// AFU TX MMIO read response
	output t_if_ccip_c2_Tx                  afu_tx_mmio,

	// AFU RX read response, MMIO request
	input  wire                             spl_rx_rd_valid,
	input  wire                             spl_mmio_rd_valid,
	input  wire                             spl_mmio_wr_valid,
	input  t_ccip_c0_RspMemHdr              spl_rx_rd_hdr,
	input  wire [511:0]     spl_rx_data,

	// AFU RX write response
	input  wire                             spl_rx_wr_valid,
	input  t_ccip_c1_RspMemHdr              spl_rx_wr_hdr,
       
	//============================================================
	
    
    // RX_RD response, afu_io --> afu_core
    output  reg                             io_rx_rd_valid,
    output  reg  [511:0]                    io_rx_data,    
    output	reg  [15:0]						io_rx_tag, 
    // TX_RD request, afu_core --> afu_io
    input  wire                             cor_tx_rd_valid,
    input  wire [41:0]                      cor_tx_rd_addr,
    
    // TX_WR request, afu_core --> afu_io
    input  wire                             cor_tx_wr_valid,
    input  wire                             cor_tx_fence_valid,
      
    input  wire [41:0]                      cor_tx_wr_addr, 
    input  wire [511:0]                     cor_tx_data,     

	// afu_io --> afu_core
	output reg [41:0] 						io_src_ptr,
	output reg [41:0] 						io_dst_ptr,
	
	output  reg						core_reset_n,
	output  reg						core_start,
	input	reg  [15:0]             tx_wr_tag,
    input	reg  [15:0]             tx_rd_tag,
	output reg						wr_rx_valid,
	output reg   [15:0]				wr_rx_tag
);
    reg                             tx_wr_block;
    reg  [5:0]                      tx_wr_block_cnt;
	
	
	//-------------------------------------------------------            
    // TX_WR, drive afu_tx_wr port
    //-------------------------------------------------------	
	always @(posedge clk) begin
		if (spl_reset) begin
			afu_tx_data <= 0;
			afu_tx_wr_hdr <= 0;
			afu_tx_wr_valid <= 1'b0;
			tx_wr_block <= 1'b0;
		end
		
		else begin
			
			if (cor_tx_wr_valid) begin
				afu_tx_data <= cor_tx_data;
				afu_tx_wr_valid <= 1'b1;			
				if(cor_tx_fence_valid)begin
                    afu_tx_wr_hdr.vc_sel   <= eVC_VL0;
					afu_tx_wr_hdr.req_type <= eREQ_WRFENCE;
					afu_tx_wr_hdr.address  <= 42'h0;
					afu_tx_wr_hdr.mdata    <= 16'h0;
					afu_tx_wr_hdr.sop      <= 1'b0;        
					afu_tx_wr_hdr.cl_len   <= eCL_LEN_1;   
                end
                else begin     // mem_wr
					afu_tx_wr_hdr.vc_sel   <= eVC_VL0;
					afu_tx_wr_hdr.req_type <= eREQ_WRLINE_I;
					afu_tx_wr_hdr.address  <= cor_tx_wr_addr[41:0];
					afu_tx_wr_hdr.mdata    <= tx_wr_tag;
					afu_tx_wr_hdr.sop      <= 1'b1;        // TODO: multi-CL
					afu_tx_wr_hdr.cl_len   <= eCL_LEN_1;   // TODO: multi-CL
                end			
			end
			else begin
				afu_tx_wr_valid <= 1'b0;
				afu_tx_data <= afu_tx_data;
				afu_tx_wr_hdr <= afu_tx_wr_hdr;
			end			
		end
	end

	//-------------------------------------------------------            
    // TX_RD, drive afu_tx_rd port
    //-------------------------------------------------------
    always @(posedge clk) begin
        if (spl_reset) begin
            afu_tx_rd_valid <= 1'b0;
			afu_tx_rd_hdr <= 10;
        end

        else begin
            afu_tx_rd_valid <= 1'b0;
            if (cor_tx_rd_valid) begin
                afu_tx_rd_valid <= 1'b1;
				afu_tx_rd_hdr.vc_sel   <= eVC_VL0;
				afu_tx_rd_hdr.req_type <= eREQ_RDLINE_I;
				afu_tx_rd_hdr.address  <= cor_tx_rd_addr[41:0];
				afu_tx_rd_hdr.mdata    <= tx_rd_tag;
				afu_tx_rd_hdr.cl_len   <= eCL_LEN_1;
		
            end
        end
    end 
	
	//-------------------------------------------------------            
    // RX, forward data to afu_core
    //-------------------------------------------------------
    always @(posedge clk) begin
        io_rx_rd_valid <= spl_rx_rd_valid;
        io_rx_data <= spl_rx_data;  
		io_rx_tag <=  spl_rx_rd_hdr.mdata[15:0];    
    end

	//wr rx
	always @(posedge clk) begin
		wr_rx_valid <= spl_rx_wr_valid;
		wr_rx_tag	<= spl_rx_wr_hdr.mdata[15:0];
	end
	
	//-------------------------------------------------------
	// CSR Address Map (byte address)
	//-------------------------------------------------------

	localparam  CSR_AFH_DFH_BASE  = 16'h000;     // RO - Start for the DFH info for this AFU
	localparam  CSR_AFH_ID_L      = 16'h008;     // RO - Lower 64 bits of the AFU ID
	localparam  CSR_AFH_ID_H      = 16'h010;     // RO - Upper 64 bits of the AFU ID
	localparam  CSR_DFH_RSVD0     = 16'h018;     // RO - Offset to next AFU
	localparam  CSR_DFH_RSVD1     = 16'h020;     // RO - Reserved space for DFH managment

	//-------------------------------------------------------

	t_ccip_c0_ReqMmioHdr mmio_req_hdr;
	t_ccip_mmioData      mmio_req_data;

	// RX.c0 interleaves memory rd responses and MMIO rd/wr requests
	always @(*) begin
		mmio_req_hdr = t_ccip_c0_ReqMmioHdr'(spl_rx_rd_hdr);
		mmio_req_data = spl_rx_data[CCIP_MMIODATA_WIDTH-1:0];
	end

	// AFU discovery - SW reads DFH and AFU ID
	always @(posedge clk) begin
	
		if (spl_mmio_rd_valid) begin
			case ({mmio_req_hdr.address[13:0], 2'b0})  // use byte address
			
				CSR_AFH_DFH_BASE: begin
					t_ccip_dfh afu_dfh;
					afu_dfh = ccip_dfh_defaultDFH();
					afu_dfh.f_type = eFTYP_AFU;
					afu_dfh.nextFeature = NEXT_DFH_BYTE_OFFSET;

					afu_tx_mmio.data        <= afu_dfh;
					afu_tx_mmio.mmioRdValid <= 1'b1;
					afu_tx_mmio.hdr         <= mmio_req_hdr.tid;
				end
				
				CSR_AFH_ID_L   : begin
					afu_tx_mmio.data        <= 64'hdead_beef_0123_4567;
					afu_tx_mmio.mmioRdValid <= 1'b1;
					afu_tx_mmio.hdr         <= mmio_req_hdr.tid;
				end
				
				CSR_AFH_ID_H   : begin
					afu_tx_mmio.data        <= 64'h0424_2017_dead_beef;
					afu_tx_mmio.mmioRdValid <= 1'b1;
					afu_tx_mmio.hdr         <= mmio_req_hdr.tid;
				end
				
				CSR_DFH_RSVD0  : begin
					afu_tx_mmio.data        <= 64'h0;
					afu_tx_mmio.mmioRdValid <= 1'b1;
					afu_tx_mmio.hdr         <= mmio_req_hdr.tid;
				end
				
				CSR_DFH_RSVD1  : begin
					afu_tx_mmio.data        <= 64'h0;
					afu_tx_mmio.mmioRdValid <= 1'b1;
					afu_tx_mmio.hdr         <= mmio_req_hdr.tid;
				end
				
				default: begin
					afu_tx_mmio.data        <= 64'h0;
					afu_tx_mmio.mmioRdValid <= 1'b0;
					afu_tx_mmio.hdr         <= 0;
				end
			endcase
		end 
		
		else begin
			afu_tx_mmio.mmioRdValid <= 1'b0;
			afu_tx_mmio.hdr         <= 0;
			afu_tx_mmio.data        <= 64'h0;
		end
	end
	
	//------------------------------------------------------
	
	localparam  CSR_AFU_DSM_BASEL = 16'h110;     // 32b RW - Lower 32-bits of AFU DSM base address. The lower 6-bbits are 4x00 since the address is cache aligned.
	localparam  CSR_AFU_DSM_BASEH = 16'h114;     // 32b RW - Upper 32-bits of AFU DSM base address.
	
	localparam CSR_SRC_ADDR = 16'h120; //64b RW - Source buffer address
	localparam CSR_DST_ADDR = 16'h128; //64b RW - Destination buffer address

	localparam  CSR_CTL           = 16'h138;     // 32b RW   Control CSR to start n stop the test
	reg  [31:0]  csr_ctl;
	
	//-------------------------------------------------------
	// DSM Address Map (byte address)
	//-------------------------------------------------------
	//localparam  DSM_STATUS        = 32'h40;      // 512b RO  Ttest status and error info
	
	//reg  [63:0]  dsm_base_addr;
	//reg  [41:0]  dsm_stat_address;
	
	//-------------------------------------------------------
	
	// SW writes CSR
	always @(posedge clk) begin
		if (spl_reset) begin
			//dsm_base_addr           <= 64'b0;
			csr_ctl                 <= 32'b0;
			
		end
		else begin

			if (spl_mmio_wr_valid ) begin
				case ({mmio_req_hdr.address[13:0], 2'b0})  // use byte address
					
					CSR_SRC_ADDR : begin
						io_src_ptr <= mmio_req_data;	// source pointer
					end
					
					CSR_DST_ADDR : begin
						io_dst_ptr <= mmio_req_data;	// destination pointer
					end
					
					CSR_CTL      : csr_ctl    <= mmio_req_data[31:0];
					
					default:;
				endcase
			end
		end
	end
	
	//---------------------------------------------------------------------

	// afu states
	always @(posedge clk) begin
		if (spl_reset) begin
			core_start        <= 0;
			core_reset_n      <= 0;
		end 

		else begin
			core_reset_n      <= csr_ctl[0];
			core_start        <= csr_ctl[1]; 
		end
	end

endmodule
	
	
	
	
	
	
	
	
	
	
  
