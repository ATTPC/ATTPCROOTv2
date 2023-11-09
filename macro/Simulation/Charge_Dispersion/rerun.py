import subprocess

#run simulation
#subprocess.run(['root', '-l', 'Mg20_test_sim_pag.C'], input=".q\n".encode(), check=True)

# Execute the 'root -x rundigi_sim_Ari.C' command
subprocess.run(['root', '-l', 'rundigi_sim_CD.C'], input=".q\n".encode(), check=True)

# Execute the 'root -l run_eve_new.C' command
subprocess.run(['root', '-l', 'run_eve_new.C'],)
