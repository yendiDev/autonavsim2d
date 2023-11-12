---
## AutoNavSim2D
This project provides a 2D simulation environment designed for the development and testing of path planning algorithms. It offers a flexible and interactive platform for researchers, developers, and hobbyists to experiment with various path planning strategies in a controlled virtual space.

This project is being maintained by [Clinton Anani](https://www.linkedin.com/in/clinton-anani-56a125196/).

---

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Demo](#demo)
- [Configuration](#configuration)
- [Requirements](#requirements)
- [Custom Planner](#custom-planner)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)
- [Contact](#contact)

### Installation

Follow these steps to install and set up the simulation environment on your system:

```bash
pip install autonavsim2D
```

### Usage

To use the simulation environment for basic path planning and navigation with the default Djikstra algorithm path planner, create a `main.py` file, and import the package:

```python
from autonavsim2d.autonavsim2d import AutoNavSim2D

nav = AutoNavSim2D(
    custom_planner='default', 
    window='amr'
)

nav.run()
```


To use the GUI, there are three steps involved to set the robot up, set a goal location, and generate obstacles:

+ The first left-click is to place the robot in any location of your choosing. You can click `reset` to clear the map.
+ The second left-click sets your goal location. The cell you left-click on as your goal location will be green. To remove that location, right-click on the cell.
+ And lastly, to create obstacles (colored black), left-click on anywhere on the map and drag, after setting the robot and choosing your goal location. To remove an obstacle from the map, simply right-click on it.

These steps must be followed one after the other in order to set the robot, set your goal location, and create obstacles. Check out the video below for a demo:

https://github.com/yendiDev/autonavsim2d/assets/57093800/6d96191d-1a85-4c3a-a542-0d29c1232b96





### Configuration

AutoNavSim2D can be customized in numerous ways. To launch the simulation environment with a starting page, set the `window` parameter to `default`:

```python
nav = AutoNavSim2D(
    custom_planner='default', 
    window='default'
)
```

To launch the simulation environment in map mode where you can begin visualization, set the `window` parameter to `amr`:

```python
nav = AutoNavSim2D(
    custom_planner='default', 
    window='amr'
)
```


### Features

Currently, AutoNavSim2D has the following features:

+ 2D path planning
+ Reactive autonomous navigation
+ Dynamic map generation
+ Support for custom graph-based path planning algorithms such as `A*`, `Djiktra` or `D*` 




### Demo

See a video demo of the simulation environment in action [here](https://x.com/oxncgen/status/1667243532166242340?s=20) or check out the screenshots below:

![AutoNavSim2D](https://github.com/yendiDev/self-driving-car-ros2/assets/57093800/1adf5e0c-d9d3-4b57-9d51-cc000458c41a)
![AutoNavSim2D](https://github.com/pytorch/pytorch/assets/57093800/f605eea7-4eff-4e54-8f66-e0a89cfe6844)


### Requirements

AutoNavSim2D is designed to be lightweight and memory efficient, so no dedicated hardware is required to run it. It is built on the pygame python package. To install pygame:

```bash
pip install pygame
```

### Custom Planner

Writing your own path planning algorithm to be used in AutoNavSim2D is really simple. First, the map (1147x872) is represented as shown below:

![Map](https://github.com/yendiDev/autonavsim2d/assets/57093800/6f71fa51-cfba-45e8-b325-c147ecb811e4)

Next the 2D matrix representation of the map, where 1 represents a free path and 0 represents an obstable is shown below. You will receive this matrix when writing your custom planner: 

```python
[
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    ...
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]
```

To use your custom path planner in AutoNavSim2D, write it in a function or class, return the optimal path as well as the time taken to calculate the path, and set it to the `custom_planner` parameter as seen below:

```python
from autonavsim2d import AutoNavSim2D

def my_planner(grid, matrix, start_loc, goal_loc):
    # your own custom path planning algorithm here
    path = []
    runtime = ''
    
    return (path, runtime)

nav = AutoNavSim2D(
    custom_planner=my_planner, 
    window='amr'
)

nav.run()
```

### Contributing

To contribute, please email me at kceequan01@gmail.com. We welcome contributions from the community! 

### License

This project is licensed under the MIT License - see the [LICENSE](https://www.github.com/yendiDev/autonavsim2d/blob/main/LICENSE) file for details.

### Acknowledgments

AutoNavSim was built on [Pygame](https://www.pygame.org/news). Thank you to the Pygame community for making this available :)

### Contact

For inquiries or feedback, feel free to reach out:
+ Email: kceequan01@gmail.com
+ Twitter: https://twitter.com/oxncgen
+ Linkedin: https://www.linkedin.com/in/clinton-anani-56a125196/

