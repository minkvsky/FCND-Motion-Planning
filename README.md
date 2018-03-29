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

### Step 5: Inspect the relevant files

- `motion_planning.py`
- `planning_utils.py`.
- `colliders.csv` contains the 2.5D map of the simulator environment.

### Step 6: Explain what's going on in  `motion_planning.py` and `planning_utils.py` 


` Your first task` in this project is to explain
- what's different about `motion_planning.py` from the `backyard_flyer_solution.py` script,
  - in `state_callback` : add `plan_path` in `motion_planning.py`
  - `plan_path` in `motion_planning.py`:
    - generate waypoints, replace `calculate_box` in `backyard_flyer_solution.py`
    - set home position, removed from `arming_transition`
    - add `send_waypoints`
- and how the functions provided in `planning_utils.py` work. (todo)
  - `create_grid`
  - `valid_actions`
  - `a_star`
  - `heuristic`

### Step 7: Write your planner

Your planning algorithm is going to look something like the following:

- Load the 2.5D map in the `colliders.csv` file describing the environment.
- Discretize the environment into a grid or graph representation.
- Define the start and goal locations. You can determine your home location from `self._latitude` and `self._longitude`.
  - `different goal loacations` are needed for testing
- Perform a search using A* or other search algorithm.
- Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
- Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0]).

Some of these steps are already implemented for you and some you need to modify or implement yourself.  See the [rubric](https://review.udacity.com/#!/rubrics/1534/view) for specifics on what you need to modify or implement.

### Step 8 :Need todo

- Set your global home position
- Set your current local position
- Set grid start position from local position
- Set grid goal position from geodetic coords
- Modify A* to include diagonal motion (or replace A* altogether)
- Cull waypoints (prune path of unnecessary waypoints)


### Try flying more complex trajectories
In this project, things are set up nicely to fly right-angled trajectories, where you ascend to a particular altitude, fly a path at that fixed altitude, then land vertically. However, you have the capability to send 3D waypoints and in principle you could fly any trajectory you like. Rather than simply setting a target altitude, try sending altitude with each waypoint and set your goal location on top of a building!

### Adjust your deadbands
Adjust the size of the deadbands around your waypoints, and even try making deadbands a function of velocity. To do this, you can simply modify the logic in the `local_position_callback()` function.

### Add heading commands to your waypoints
This is a recent update! Make sure you have the [latest version of the simulator](https://github.com/udacity/FCND-Simulator-Releases/releases). In the default setup, you're sending waypoints made up of NED position and heading with heading set to 0 in the default setup. Try passing a unique heading with each waypoint. If, for example, you want to send a heading to point to the next waypoint, it might look like this:

```python
# Define two waypoints with heading = 0 for both
wp1 = [n1, e1, a1, 0]
wp2 = [n2, e2, a2, 0]
# Set heading of wp2 based on relative position to wp1
wp2[3] = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
```

This may not be completely intuitive, but this will yield a yaw angle that is positive counterclockwise about a z-axis (down) axis that points downward.

Put all of these together and make up your own crazy paths to fly! Can you fly a double helix??
![Double Helix](./misc/double_helix.gif)

Ok flying a double helix might seem like a silly idea, but imagine you are an autonomous first responder vehicle. You need to first fly to a particular building or location, then fly a reconnaissance pattern to survey the scene! Give it a try!
