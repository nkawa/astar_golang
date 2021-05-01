package main

import (
	"fmt"

	astar "github.com/fukurin00/astar_golang"
)

func main() {
	var objects [][2]float64
	objects = append(objects, [2]float64{-5, -5})
	objects = append(objects, [2]float64{5, 5})
	objects = append(objects, [2]float64{3, 4})

	aStar := astar.NewAstar(objects, 2, 0.5)
	route, err := aStar.Plan(-1, -1, 2, 1)
	if err != nil {
		fmt.Print(err)
	} else {
		fmt.Print(route)
	}
}
