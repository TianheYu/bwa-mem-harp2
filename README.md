#**Project: SMEM Accelerator on Hardware Accelerator Research Program (HARPv2) platform**



## 1.Introduction
The project's goal is to accelerate the SMEM kernel on HARP system, this is the original code that is designed for HARPv2 platform. 
Our code is based on Heng Li's bwa-mem project (https://github.com/lh3/bwa/releases/tag/0.7.8). Please refer to [1] for code design 
details.


## 2. Quick Startup
One should first intall HARP2 system and follow the instructions to synthesis the hardware code or Do the simulation with VCS.

To run the software, use the run.sh script:

(./helloALInlb mem -t 4 -b 64 -Ma -R '@RG       ID:HCC1954      LB:HCC1954      SM:HCC1954' /path_to_fasta_file/human_g1k_v37.fasta 
/path_to_fq_file/HCC1954_1_1Mreads.fq /path_to_fq_file/HCC1954_2_1Mreads.fq | tee out.txt) 3>&1 1>&2 2>&3 | tee stderr.log

This is to run 4 cpu thread, with a batch size of 64 (reads) each time to be assigned to hardware. The alignment result will be in out.txt.

**[1] Chang, Mau-Chung Frank, Yu-Ting Chen, Jason Cong, Po-Tsang Huang, Chun-Liang Kuo, and Cody Hao Yu. "The SMEM Seeding 
Acceleration for DNA Sequence Alignment." In Field-Programmable Custom Computing Machines (FCCM), 2016 IEEE 24th Annual 
International Symposium on, pp. 32-39. IEEE, 2016.**
