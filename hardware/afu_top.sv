import ccip_if_pkg::*;
module afu_top #(
    parameter NEXT_DFH_BYTE_OFFSET = 0
) (
    input  wire                             clk,
    input  wire                             spl_reset,

    // AFU TX read request
	input  wire				spl_tx_rd_almostfull,
    output wire                             afu_tx_rd_valid,
    output t_ccip_c0_ReqMemHdr              afu_tx_rd_hdr,

    // AFU TX write request
    input  wire                             spl_tx_wr_almostfull,
    output wire                             afu_tx_wr_valid,
    output t_ccip_c1_ReqMemHdr              afu_tx_wr_hdr,
    output wire [511:0]     afu_tx_data,

    // AFU TX MMIO read response
    output t_if_ccip_c2_Tx                  afu_tx_mmio,

    // AFU RX read response
    input  wire                             spl_rx_rd_valid,
    input  wire                             spl_mmio_rd_valid,
    input  wire                             spl_mmio_wr_valid,
    input  t_ccip_c0_RspMemHdr              spl_rx_rd_hdr,
    input  wire [511:0]     spl_rx_data,

    // AFU RX write response
    input  wire                             spl_rx_wr_valid,
    input  t_ccip_c1_RspMemHdr              spl_rx_wr_hdr
);

	//-------------------------------------------------
	
	wire                             io_rx_rd_valid;
    wire  [511:0]                    io_rx_data; 
	wire [15:0]	io_rx_tag;
	wire                             cor_tx_rd_valid;
    wire [41:0]                      cor_tx_rd_addr;

	
	wire                             cor_tx_wr_valid;
    wire                             cor_tx_fence_valid;
       
    wire [41:0]                      cor_tx_wr_addr; 

    wire [511:0]                     cor_tx_data;

	
	wire            				 core_reset_n;
	wire            				 core_start;
	
	wire [41:0] 					 io_src_ptr;
	wire [41:0] 					 io_dst_ptr;
	wire [15:0]		tx_wr_tag;
	wire [15:0]		tx_rd_tag;
	wire [15:0]		wr_rx_tag;
	wire			wr_rx_valid;

	afu_core afu_core(
		.Clk_400(clk),
		.reset_n(core_reset_n),
		.re2xy_go(core_start),
		.spl_reset(spl_reset),
		
		// TX_RD request, afu_core --> afu_io
		.ab2l1_stallRd(spl_tx_rd_almostfull),
		.l12ab_RdEn(cor_tx_rd_valid),
		.l12ab_RdAddr(cor_tx_rd_addr),
		.l12ab_RdTID(tx_rd_tag),
		
		.ab2l1_WrAlmFull(spl_tx_wr_almostfull),
		.l12ab_WrEn(cor_tx_wr_valid),
		.l12ab_WrAddr(cor_tx_wr_addr),
		.l12ab_WrTID(tx_wr_tag), 
		.l12ab_WrDin(cor_tx_data),
		.l12ab_WrFence(cor_tx_fence_valid),	
			 
		// RX_RD response, afu_io --> afu_core
		.ab2l1_RdRspValid_T0(io_rx_rd_valid),
		.ab2l1_RdData_T0(io_rx_data),    
		.ab2l1_RdRsp_T0(io_rx_tag),
		.ab2l1_WrRspValid_T0(wr_rx_valid),
		.ab2l1_WrRsp_T0(wr_rx_tag),

		.ctx_src_ptr(io_src_ptr),
		.ctx_dst_ptr(io_dst_ptr)
	);
	
	 afu_io #(
        .NEXT_DFH_BYTE_OFFSET(NEXT_DFH_BYTE_OFFSET)
    ) afu_io (
	
		.clk(clk),
		.spl_reset(spl_reset),

		// AFU TX read request
		.afu_tx_rd_valid(afu_tx_rd_valid),
		.afu_tx_rd_hdr(afu_tx_rd_hdr),

		// AFU TX write request
		
		.afu_tx_wr_valid(afu_tx_wr_valid),
		.afu_tx_wr_hdr(afu_tx_wr_hdr),
		.afu_tx_data(afu_tx_data),

		// AFU TX MMIO read response
		.afu_tx_mmio(afu_tx_mmio),

		// AFU RX read response, MMIO request
		.spl_rx_rd_valid(spl_rx_rd_valid),
		.spl_mmio_rd_valid(spl_mmio_rd_valid),
		.spl_mmio_wr_valid(spl_mmio_wr_valid),
		.spl_rx_rd_hdr(spl_rx_rd_hdr),
		.spl_rx_data(spl_rx_data),

		// AFU RX write response
		.spl_rx_wr_valid(spl_rx_wr_valid),
		.spl_rx_wr_hdr(spl_rx_wr_hdr),

		// RX_RD response, afu_io --> afu_core
		.io_rx_rd_valid(io_rx_rd_valid),
		.io_rx_data(io_rx_data),    
		.io_rx_tag(io_rx_tag),	
		// TX_RD request(), afu_core --> afu_io
		.cor_tx_rd_valid(cor_tx_rd_valid),
		.cor_tx_rd_addr(cor_tx_rd_addr),
		
		// TX_WR request, afu_core --> afu_io
		.cor_tx_wr_valid(cor_tx_wr_valid),
		.cor_tx_fence_valid(cor_tx_fence_valid),
       
		.cor_tx_wr_addr(cor_tx_wr_addr), 
		.cor_tx_data(cor_tx_data),     

		// afu_io --> afu_core
		.io_src_ptr(io_src_ptr),
		.io_dst_ptr(io_dst_ptr),
		
		.core_reset_n(core_reset_n),
		.core_start(core_start),
		.tx_wr_tag(tx_wr_tag),
		.tx_rd_tag(tx_rd_tag),
		.wr_rx_valid(wr_rx_valid),
		.wr_rx_tag(wr_rx_tag)
	);
	
endmodule
