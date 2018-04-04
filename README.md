# FCND - 3D Motion Planning
![Quad Image](./misc/enroute.png)



In this project I integrate the techniques that I have learned throughout the last several lessons to plan a path through an urban environment.



### Step 1: Download the Simulator
This is a new simulator environment!  

Download the Motion-Planning simulator for this project that's appropriate for your operating system from the [simulator releases respository](https://github.com/udacity/FCND-Simulator-Releases/releases).

### Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

### Step 3: base on this Repository
```sh
git clone https://github.com/udacity/FCND-Motion-Planning
```

### Step 5: Inspect the main files

- `motion_planning.py` contains the class MotionPlanning and is the main file.
- `planning_utils.py` contains utils for `motion_planning.py`
- `colliders.csv` contains the 2.5D map of the simulator environment.

### Step 6: Explain what's going on in  `motion_planning.py` and `planning_utils.py`


- what's different about `motion_planning.py` from the `backyard_flyer_solution.py` script?

  - the main differnce between two files is the generation of waypoints
    - in `backyard_flyer_solution.py`, use fixed waypoints.The following are the method of generate waypoints and its core code:
      - `calculate_box`
      - `local_waypoints = [[10.0, 0.0, 3.0], [10.0, 10.0, 3.0], [0.0, 10.0, 3.0], [0.0, 0.0, 3.0]]`
    - in `motion_planning.py`, use fixed goal and A* to find waypoints to reach the goal.The following are the method of generate waypoints and its core code:
      - `plan_path`
      - `path, _ = a_star(grid, heuristic, grid_start, grid_goal)`
- how the functions provided in  `planning_utils.py` work. (todo)
  - `create_grid`
    - based on given obstacle data, drone altitude and safety distance arguments
    - Returns a grid representation of a 2D configuration space
    - the grid representation in fact is matrix where ostacle has value 1 and others have value 0.
  - `Action`
    - An action is represented by a 3 element tuple. The first 2 values are the delta of the action relative to the current grid position. The third value is the cost of performing the action.
    - there are 8 kinds of actions with 2 kinds of cost: 1 or sqrt(2).

  - `valid_actions`
    - used in a_star
    - if one action will lead to meet obstacle, then we treat this action as invalid action.Then we will get valid actions.
  - `a_star`
    - based a grid and heuristic function
    - returns the lowest cost path from start to goal
    - use `PriorityQueue` datas sturcture, which is a convenient way of maintaining a sorted queue, allows us to quickly and efficiently select the lowest cost partial plan from your queue of all partial plans.
    - we will consider 3 kinds of cost
      - current cost
      - cost of action
      - cost based on the environment, i.e., the distance to the goal.
      ```
      new_cost = current_cost + a.cost + h(next_node, goal)
      ```
  - `heuristic`
    - euclidean distance between two position
    - use `np.linalg.norm` to implement
    - used in a_star
  - `prune_path`
    - use `collinearity_check` to prune the path (decrease the number of waypoints)
    - If three points are collinear, then we remove the middle point from the path. Remove repeatly untill no point can be removed.
    ```
    if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
    ```
  - `collinearity_check`
    - use the determinant of the matrix to determine whether or not three points are collinear.

### Step 7: Write planner

planning algorithm is going to look something like the following:

- Load the 2.5D map in the `colliders.csv` file describing the environment.
- Discretize the environment into a grid or graph representation.
- Define the start and goal locations. You can determine your home location from `self._latitude` and `self._longitude`.
  - `different goal loacations` are needed for testing
- Perform a search using A* or other search algorithm.
- Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
- Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0]).


### Step 8: Implementing Path Planning Algorithm

- Set your global home position
  - read the first line of the csv file
  - extract lat0 and lon0 as floating point values
  - and use the `self.set_home_position()` method to set global home.
  ```   
        with open('colliders.csv', 'r') as f:
            latlon = f.readline()
        ll = latlon.strip().replace(',', '').split(' ')
        lat0, lon0 = float(ll[1]), float(ll[3])
        self.set_home_position(lon0, lat0, 0)
  ```
- Set your current local position
  - first, convert global position to current local position using `global_to_local`
  - then update `self._north, self._east, self._down`
  ```
        global_position = self.global_position
        current_local_position = global_to_local(global_position, self.global_home)
        self._north, self._east, self._down = current_local_position
  ```
- Set grid start position from local position
  - use `create_grid` and `colliders.csv` to create grid representation.
  ```
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
  ```
  - we will get grid representation and start offset `(north_offset, east_offset)`
  - add local position to start offset from the process of creating grid
  ```
        grid_start_off = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start = tuple(map(sum, zip(map(int, current_local_position), grid_start_off)))
  ```
- Set grid goal position from geodetic coords
  - get local goal position using `global_to_local` from geodetic coords
  - add goal local position to start offset from the process of creating grid
  ```
        arbitrary_local_position = global_to_local(arbitrary_global_position, self.global_home)
        grid_goal = tuple(map(sum, zip(grid_start_off, arbitrary_local_position)))
  ```
- Modify A* to include diagonal motion
  - add following snippet into classs `Action`
  ```
  # diagonal motions with a cost of sqrt(2)
  NORTH_WEST = (-1, -1, np.sqrt(2))
  NORTH_EAST = (-1, 1, np.sqrt(2))
  SOUTH_WEST = (1, -1, np.sqrt(2))
  SOUTH_EAST = (1, 1, np.sqrt(2))
  ```
  - add following snippet into `valid_actions` to remove invalid action
  ```
  if (x - 1 < 0 and y - 1 < 0) or grid[x - 1, y - 1] == 1:
      valid_actions.remove(Action.NORTH_WEST)
  if (x - 1 < 0 and y + 1 > m) or grid[x - 1, y + 1] == 1:
      valid_actions.remove(Action.NORTH_EAST)
  if (x + 1 > n and y - 1 < 0) or grid[x + 1, y - 1] == 1:
      valid_actions.remove(Action.SOUTH_WEST)
  if (x + 1 > n and y + 1 > m) or grid[x + 1, y + 1] == 1:
      valid_actions.remove(Action.SOUTH_EAST)
  ```

- Cull waypoints
  - use collinearity to prune path of unnecessary waypoints so that the drone flight will be much smoother.
  ```
  path = prune_path(path)
  ```
  - a picture of waypoints before and after pruning to show why pruning was needed
  ![pruned_path](images\pruned_path.jpg)


### Step 9: run the motion_planning
  - run the simulator
  - in env `fcnd`, run `python motion_planning.py` (use the default goal)
  - or run `python motion_planning.py --goal 200 200` (set loacl goal position (200, 200))
