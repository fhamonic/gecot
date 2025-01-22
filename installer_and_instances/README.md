## Installation

Unzip the archive and run the "Gecot-1.0-win64.msi" installer.

## Generate the test instance

Ensure the required python packages are installed with the following command line :
```
pip install pillow csv json matplotlib numpy
```
then run :
```
python generate_instance.py
```

This will create a folder "instance_40x40" containing a cropped version of the 70x70 instance present in the paper.
Parameters for generating the instance can be modified in the "input_data.py" file.


## Solve the test instance

The following command will find the optimal solution for the 40x40 cells instance with a budget of 14 and print it in the "solution.json" file (about 1 minute of computation time):

```
gecot_optimize prep_mip instance_40x40/instance.json 14 -v -p -o solution.json
```

## Visualize the solution

The solution can be visualized with the "display_solution.py" python script :
```
python display_solution.py solution.json
```
