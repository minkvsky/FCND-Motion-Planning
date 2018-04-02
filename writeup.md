### Step 5: Inspect the relevant files

- `motion_planning.py`
- `planning_utils.py`
- `colliders.csv` contains the 2.5D map of the simulator environment.

### Step 6: Explain what's going on in  `motion_planning.py` and `planning_utils.py`


- what's different about `motion_planning.py` from the `backyard_flyer_solution.py` script,
  - in `state_callback` : add `plan_path` in `motion_planning.py`
  - `plan_path` in `motion_planning.py`:
    - generate waypoints, replace `calculate_box` in `backyard_flyer_solution.py`
    - set home position, removed from `arming_transition`
    - add `send_waypoints`
- and how the functions provided in `planning_utils.py` work. (todo)
  - `create_grid`
    - Returns a grid representation of a 2D configuration space
    - based on given obstacle data, drone altitude and safety distance arguments
  - `valid_actions`
    - used in a_star
  - `a_star`
    - Given a grid and heuristic function
    - returns the lowest cost path from start to goal.
  - `heuristic`
    - `np.linalg.norm`

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
- Set your current local position
  - update `self._north, self._east, self._down`
- Set grid start position from local position
  - use `create_grid` to create grid representation.
  - we will get grid representation and start offset `(north_offset, east_offset)`
  - add local position to start offset from the process of creating grid
- Set grid goal position from geodetic coords
  - get local goal position using `global_to_local` from geodetic coords
  - add goal local position to start offset from the process of creating grid
- Modify A* to include diagonal motion (or replace A* altogether)
  - add diagonal motions with a cost of sqrt(2) in `Action` and `Action.valid_actions`
- Cull waypoints
  - use collinearity to prune path of unnecessary waypoints
