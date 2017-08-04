# Mesh Slicer

A simple user interface with which one can slice any mesh (with single or multiple components) in arbitarary directions and get either a rasterized image of the resultant contour or a mesh file (PLY) containing the vertices and edges of the contour.  

## Getting Started

The below instructions will help you install the mesh slicer on your local linux machine and give details on the user interface.

### Prerequisites

Incomplete

```
Give examples
```

### Installing

Incomplete
```
git clone https://github.com/nitinagarwal/mesh_slicer.git
mkdir build
cmake ..
make
```

After successful installation, you can import any meshfile having [OFF](http://segeval.cs.princeton.edu/public/off_format.html) format from the terminal. For example:

```
./mesh_slicer mouseBrain.off ./dirs
```

where:
* the first argument is the mesh file
* the second argument is the output directory for contour mesh or image file(s).

### User Interface






## Contact

Mesh Slicer was created by [Nitin Agarwal](http://www.ics.uci.edu/~agarwal/) to help neuroscience researchers to
generate 2D atlas images by slicing a Virtual 3D Mouse Brain Atlas Model in any arbitarary direction.


