(./helloALInlb mem -t 4 -b 64 -Ma -R '@RG	ID:HCC1954	LB:HCC1954	SM:HCC1954' /path_to_fasta_file/human_g1k_v37.fasta /path_to_fq_file/HCC1954_1_1Mreads.fq /path_to_fq_file/HCC1954_2_1Mreads.fq | tee out.txt) 3>&1 1>&2 2>&3 | tee stderr.log

