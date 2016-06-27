# Algorithms
This page is meant to describe the various algorithm
implementations used in the library and possible ways
to improve these implementations. These have been
provided to help learners understand the algorithms
and help contributors in improving the implementation
by replacing the inefficient algorithms with efficient
ones.

## Graph Search

### Dijkstra's Search
```
procedure dijkstra_search(Graph g, Point in, Point out):

	for each vertex v in Graph:
		dist[v] <- INFINITY
		parent[v] <- UNDEFINED
	dist[in] = 0

	while True:
		curr = vertex with min dist  # uses sorting, can be improved by using a heap
		if no vertex in Graph:
			return "No path found"

		if curr is out:
			construct path from list of parents
			return path
		
		for n in neighbors of curr:
			d = dist[curr] + edge_weight(curr, n)
			if d < dist[n]:
				parent[n] = curr
				dist[n] = d

		delete curr from Graph
```

### Breadth First Search
```
procedure breadth_first_search(Graph g, Point in, Point out):

	for each vertex v in graph:
		visited[v] <- FALSE
		parent[v] <- UNDEFINED

	create an empty queue Q
	Q.push(in)

	visited[in] <- TRUE
	parent[in] <- in

	while Q is not empty:
		curr = Q.pop()

		for n in neighbors of curr:
			if visited[n] == FALSE:
				visited[n] <- TRUE
				parent[n] <- curr
				Q.push(n)

				if (curr == out):
					reconstruct the path to in
					return path
	return "no path found"
```

### A\* Search

## Graph Builder

### Probabiistic Graph Builder
### Voronoi Graph Builder

## Interpolator

### Potential Field Interpolator
