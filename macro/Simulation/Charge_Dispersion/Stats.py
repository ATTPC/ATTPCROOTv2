import fileinput
import subprocess

#compile code
def run_make_install(folder_path):
    try:
        result = subprocess.run(['make', 'install', '-j', '8'], cwd=folder_path, check=True)
    except subprocess.CalledProcessError as e:
        print("Command failed with return code:", e.returncode)
 

# Specify the folder path where the make command should be executed
folder_path = '../../../build'

def run_root_script(script_path,input_data):
    try:
        instring= ".q"
        result = subprocess.run(['root', '-l', script_path], input=instring.encode(), check=True)
    except subprocess.CalledProcessError as e:
        print("Command failed with return code:", e.returncode)

# Specify the script path to be executed
script_path = './rundigi_sim_Ari.C'

# Increment value in cpp file
def update_adjpads_value(file_path, new_value):
    for line in fileinput.input(file_path, inplace=True):
        if 'Int_t AdjecentPads =' in line:
            line = line.replace(line.split('=')[1].strip(), str(new_value)+';')
        print(line, end='')

def run_python_script():
    try:
        script_path = 'numerical-analysis.py'
        subprocess.run(['python3', script_path], check=True)
    except subprocess.CalledProcessError as e:
        print("Command failed with return code:", e.returncode)
        print("Error message:")
        print(e.stderr.decode())

def update_file(file_path,new_value):
    for line in fileinput.input(file_path, inplace=True):
        if 'TString filename = ' in line:
            line = line.replace(line.split('=')[1].strip(), str(new_value)+';')
        print(line, end='')






cpp_header_file_path = '../../../AtDigitization/AtPulseTaskGADGET.h'
new_values = [0, 1]# 3, 5, 10]

for value in new_values:
    print("Updating AdjecentPads value to", value)
    update_adjpads_value(cpp_header_file_path, value)
    print("running make install")
    run_make_install(folder_path)
    print("running root script")
    update_file(script_path,str(value))
    run_root_script(script_path, value)

print("running python script")  
run_python_script()
