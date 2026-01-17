#!/usr/bin/env bash

start=$(date +%s%N)
start_ms=${start:0:16}


thread_num=2
#list=(311 767 318 768 769 317 770 771 307 763 305 306 762 312 764 313 765323 327 777 322 776 320 774 319 775 314 773 316 772)


#------------------------------------
tmpfifo=$$.fifo
trap "exec 1000>&-;exec 1000<&-;exit 0" 2
mkfifo $tmpfifo
exec 1000<>$tmpfifo
rm -rf $tmpfifo

for ((i=1;i<=$thread_num;i++))
do
	echo >&1000
done

#for i in ${list[@]}
for i in $(seq 3109 3140) #(453 530), (748 809)
#for i in $(seq 302 777)
do
	read -u1000
	{
		echo begin $i
			root "kine_ana_transfer.C($i)"
		echo >&1000
	} &
done

wait

end=$(date +%s%N)
end_ms=${end:0:16}
echo "==============================="
echo " cost time(s):"
echo "scale=3;($end_ms-$start_ms)/1000000" | bc



#for i in ${list[@]}
#do
#	./telescope.exe $i
#done
