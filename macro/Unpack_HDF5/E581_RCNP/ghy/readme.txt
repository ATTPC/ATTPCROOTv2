Analysis process: 

#AGET data:
1. copy h5 file from mnt/merger/ to /data/tempMergedData/
2. unpack the h5 file using: 
	in Unpacker_HDF5/E581: 
	>>./unpack.sh run_number
3. extract variables(such as thetaLab, kineE, dedx ,range...) from the unpacker root file and store in new root file
	in Unpacker_HDF5/E581:
	>>root "kine_ana_transfer_reunpack.C(run_number)"


#FRIBDAQ data:
1. copy evt file from attpc-frib to /mnt/merger/E581/evt/
2. decode evt file into root file which store the wave information:
	>>/data/sustech/user/ghy/frib-decode/decode run_number
3. extract the ADCMax from wave information:
	>>/data/sustech/user/ghy/frib-decode/fill_hist run_number


#Endata:
1. copy enroot file from quser@nihonium:~/exp/attpc/enana/root to /mnt/merger/

#Merge:

merge three daq data using:
	in Unpacker_HDF5/E581/ghy/merge/
	>>root "merger_data.C(run_num)"
it will store three TTree in one root file, and using AddFriend to combined them together.


