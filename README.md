# Complete, Decomposition-Free Coverage Path Planning

This repository contains code for the paper [Complete, Decomposition-Free Coverage Path Planning](https://www.ri.cmu.edu/app/uploads/2022/12/Complete_Decomposition-Free_Coverage_Path_Planning.pdf).

## Dependencies

- Qt5 (`Qt5::Core`, `Qt5::Widgets`)
- SMPL (github.com/aurone/smpl). SMPL can be built standalone as a CMake project and installed system-wide:
	- Go to smpl/smpl/ and build as a CMake project (see instructions at github.com/aurone/smpl)
	- Install system-wide (`make install`) to run cdf_cpp as is, because it includes SMPL with abosolute paths)

## Building

Build as a regular CMake project (Release mode set by default):

```bash
mkdir build
cd build
cmake ..
make [-j$(nproc)]
```

## Running

### Generate rectangular Boustrophedon coverage patterns

To generate coverage patterns that include all rectangles within a 10 cells x 10 cells area, run:

```
./gen_pattern_no_sampling 10 10
```

This generates the file `patterns_10_10.txt` in the `patterns/` directory.

### Run the planner with these patterns and Qt visualization

```
./persistent ../maps/test/frontier_test_1.map ../patterns/patterns_30_30.txt 2.0
```

## Contact
If you are using this repo ion your research, you may reach me at kusnur.tushar@gmail.com for urgent questions.
