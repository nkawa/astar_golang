# A-Star Algorithm Implementation for Golang 

# Requirement
Go version > 1.15  

# install
`go get github.com/fukurin00/astar_golang`

# sample
```
var objects [][2]float64
objects = [][2]float64{{-10,-10},{10,0},{10,10}}
aStar := astar.NewAstar(objects, 2, 0.5)
route, err := aStar.Plan(-1, -1, 2, 1)
```




