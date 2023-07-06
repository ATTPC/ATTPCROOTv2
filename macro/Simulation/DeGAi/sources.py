import subprocess

root_files = ['DeGAi_60Co.C', 'DeGAi_137Cs.C', 'DeGAi_22Na.C']

for file in root_files:
    command = f'root -q -l {file}'
    process = subprocess.Popen(command, shell=True)
    process.wait()
    process.communicate(input='.q\n'.encode())
    