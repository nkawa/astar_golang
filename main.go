package main

import (
	"fmt"

	astar "github.com/fukuirn00/astar_golang/src/astar"
)

func main() {
	objects := [2][6]float64{[6]float64{-5, -5, 2, 2, 0, 3}, [6]float64{10, 2, -2, 4, 5, 2}}

	astar.NewAstar(objects, 2, 0.5)
	route, err := astar.Plan(-1, -1, 3, 3)
	if err != nil {
		fmt.Print(route)
	}
}
