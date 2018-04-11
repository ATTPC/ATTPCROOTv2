# trackfinder - Program for finding particle tracks in point clouds

## About

trackfinder is a program that clusters points from a point cloud into
tracks, which can overlap at vertex points. It assigns each point a track
label (0, 1, ...) or the label -1 for noise. Points belonging to more than
one track obtain a combined label with teh individual labels separated
by semikolons.


## Compilation

Prerequisites:

 - Point Cloud Library (PCL)
 - C++11 capable compiler, e.g. gcc 5.x

Compilation ($ is the shell prompt):

    $ mkdir build
	$ cd build
	$ cmake ..
	$ make


## Usage

When called without any arguments, `trackfinder` gives a usage message.

The input file must be in PCL data format (this will possibly be changed in
future versions). To convert from ATTPC data file to PCL file format, there
is an external python script (not included in these source package):

    $ python3 dat2pcl.py infile outfile

The output is one of the following:

 - CSV file (default) with two lines header (starts with #)
   followed by one point per line with its track label
   ("-1" for noise, "L1;L2" for points belonging to L1 and L2)
   field separator is a comma
 - gnuplot command for drawing the result with option `-gnuplot`
 - screen display of result with option `-x11`

Example:

    $ trackfinder /path/to/infile -r 5 -gnuplot | gnuplot -persist


## Authors and copyright

Lukas Aymans, Jens Wilberg, Christoph Dalitz, 2017-2018
Hochschule Niederrhein, Institute for Pattern Recognition

The software uses the fastcluster library by Daniel Müllner,
avalable from http://danifold.net. See the directory src/hclust for details.