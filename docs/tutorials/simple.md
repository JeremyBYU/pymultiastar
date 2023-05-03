# Starting Tutorial - First Steps

The MultiGoal Planner has several parameters that determine its goal and path finding behavior. Lets begin by discussing the parameters passed into PyMultiAStar constructor.


## PyMultiAStar

::: pymultiastar._core.PyMultiAStar
    handler: python
    options:
      members:
        - __init__.py


## Explanation

PyMultiAStar first requires a  **map**, which is a 3D NumPy array through which it searches through. We call each value in this array a Cell. To start soem