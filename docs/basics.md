# Motion Planning Primer

This part of the website is meant to give the reader an intuitive
feel for the library. It is highly recommended that the reader goes
through this section before proceeding to the API reference or the
tutorials.

The library has been designed so that it is possible to use
pre-built planner as well as build new planners from scratch. The
syntax of the library seems natural once the user is familiar with
a few concepts and terminology.

## Terminology

**Point**: A point is any location in the 2D workspace.

**Path**: A path is a sequence of points.

**Map**: A map is a bitmap representing the occupied and free regions
in the space where the path is to be planned.

## Structure of Motion Planning Algorithms

We propose that most motion planning algorithms can be represented
as a combination of four basic components:

- Collision Checker
- Graph Builder
- Graph Search
- Interpolator

### Collision Checker

A collision checker is used in algorithms whenever they need to
check if a state of the robot causes collision with the workspace
or not. This library currently supports planning in 2D environments
therefore there is only a single pre-built collision checker built
into all the motion planners and cannot be changed.

### Graph Builder

A graph builder is an algorithm which takes a map of the workspace as
the input and generates a graph which is then used by the motion
planner for finding paths. For an algorithm which does not require
a graph but works directly on a map, it is possible to generate a
graph that contains every point of the map. Some graph builders are:

- Voronoi Roadmap Builder
- Probabilistic Roadmap Builder
- Visibility Roadmap

### Graph Search

A graph search is an algorithm which returns the path between two
nodes in a graph. This is one of the key player in determining the
efficiency of the planning algorithm. Some graph searches are:

- Dijkstra's Search
- Breadth First Search
- A\* Search

### Interpolator

An interpolator is the algorithm which gives the path that the robot
follows after processing the path from graph search. The interpolator
can perform smoothing, simplifications etc on the input path. An
interpolator can also be used to implement planners that rely only
on local information of the workspace. Planning using potential
fields can be thought of as an interpolator which works on the straight
line path between start and end points.
