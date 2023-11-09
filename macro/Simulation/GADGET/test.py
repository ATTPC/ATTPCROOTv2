import h5py
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
def analyze_h5_files():
    h5_file1 = "./data/CD_digi.h5"
    h5_file2 = "./data/diff_digi.h5"

    df1 = pd.read_hdf(h5_file1)
    print(df1.head())
    df2 = pd.read_hdf(h5_file2)
    print(df2.head())

analyze_h5_files()