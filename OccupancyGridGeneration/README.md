### How to run

Launch ```bash commands.bash``` to start all the nodes.

### Summary

Creates node that:

1) Removes outliers which are determined based on their neighbour points.

2) Generates occupancy grid for points from step 1. Marks cells from (0,0) to point coords as free when iterating through the points.

![Gif of result](/OccupancyGridGeneration/OccupancyGrid.gif)
