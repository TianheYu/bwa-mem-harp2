(bwa mem -t 4 -Ma -R '@RG	ID:HCC1954	LB:HCC1954	SM:HCC1954' /curr/pengwei/archive/genomics/ReferenceMetadata/human_g1k_v37.fasta ../HCC1945_1_256.fq ../HCC1945_2_256.fq | tee out.txt) 3>&1 1>&2 2>&3 | tee stderr.log

#echo "=============== BWA-MEM done, verifying result ==============="
#diff out.txt /curr/cody/genomics/bwa-mem-quickassist/golden_1read_fast.txt
