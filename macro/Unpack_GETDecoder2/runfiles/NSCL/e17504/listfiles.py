import os
import re
from glob import glob


def find_files(root, run_num):
    # root/mmX/run_XXXX/*.graw
    files = glob(os.path.join(root, 'mm*/run_%04d/*.graw' % run_num))
    matches = [re.search(r'mm(\d+).*AsAd(\d+).*_(\d\d\d\d)\.graw', s) for s in files]
    cobos = [int(m.group(1)) for m in matches]
    asads = [int(m.group(2)) for m in matches]
    index = [int(m.group(3)) for m in matches]
    keydict = {k: v for k, v in zip(files, zip(cobos, asads, index))}
    files = sorted(files, key=keydict.get)
    return files

first_run = 110
last_run = 120
full_file_paths = [find_files("/mnt/analysis/attpc/e17504", x) for x in xrange(first_run, last_run)]

new_filename = ['e17504_run_%04d.txt' % runnum for runnum in xrange(first_run, last_run)]
for runfiles, output_name in zip(full_file_paths, new_filename):
    with open(output_name, 'w') as write_file:
        for name in runfiles:
            write_file.write(name + '\n')
