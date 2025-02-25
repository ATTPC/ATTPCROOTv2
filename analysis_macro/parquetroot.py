from parquet_to_root import parquet_to_root
import os

directory = "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Estimation"
output_directory = "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Estimation/"

if not os.path.isdir(output_directory):
    os.makedirs(output_directory)
    print(f"{output_directory} created successfully!")

for file in os.listdir(directory):
    print(file)
    filename, extension = os.path.splitext(file)
    if extension == ".parquet":
      try:
        parquet_to_root(os.path.join(directory, file), os.path.join(output_directory, filename + ".root"))
      except Exception:
         pass
    else:
        print(f"{file} is not a .parquet file.")