package main

import (
	"fmt"

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
	objects, _ = astar.ObjectsFromImage("imgs/maiz.pgm", 100, 0, 0, 1)

	// for visualizing route
	plot, _ := glot.NewPlot(2, false, false)
	plot.AddPointGroup("map", "points", convert2DPoint(objects))

	objectRadius := 1.0   //passable size 1.0m
	gridResolution := 0.5 //grid size 0.5m

	aStar := astar.NewAstar(objects, objectRadius, gridResolution)
	route, err := aStar.Plan(10, 10, 120, 120) //from point(0,0) to point(120,120)
	if err != nil {
		fmt.Print(err, "\n")
	} else {
		fmt.Print("route length:", len(route), "\n")
	}

	// visualize route
	plot.AddPointGroup("route", "points", convert2DPoint(route))
	plot.SavePlot("imgs/route.png")

}
