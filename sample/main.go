package main

import (
	"fmt"
	"math/rand"

	"github.com/Arafatk/glot"
	astar "github.com/fukurin00/astar_golang"
	_ "github.com/jbuchbinder/gopnm"
)

func convert2DPoint(obj [][2]float64) (obj2d [][]float64) {
	var xs []float64
	var ys []float64
	for _, o := range obj {
		xs = append(xs, o[0])
		ys = append(ys, o[1])
	}
	obj2d = append(obj2d, xs)
	obj2d = append(obj2d, ys)
	return obj2d
}

func main() {
	var objects [][2]float64
	for j := 0; j <= 10; j++ {
		objects = append(objects, [2]float64{10, float64(j)})
		objects = append(objects, [2]float64{float64(j), 10})
		objects = append(objects, [2]float64{0, float64(j)})
		objects = append(objects, [2]float64{float64(j), 0})
		rx := 10 * rand.Float64()
		ry := 10 * rand.Float64()
		objects = append(objects, [2]float64{rx, ry})
	}

	// visualization
	plot, _ := glot.NewPlot(2, false, false)
	plot.AddPointGroup("map", "points", convert2DPoint(objects))

	objectRadius := 0.9
	gridResolution := 0.5

	aStar := astar.NewAstar(objects, objectRadius, gridResolution)
	route, err := aStar.Plan(1, 9, 9, 1)
	if err != nil {
		fmt.Print(err, "\n")
	} else {
		fmt.Print("route length:", len(route), "\n")
	}
	plot.AddPointGroup("route", "points", convert2DPoint(route))
	plot.SetXrange(-2, 12)
	plot.SetYrange(-2, 12)
	plot.SavePlot("sample.png")

}
