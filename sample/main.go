package main

import (
	"fmt"
	"log"
	"sync"
	"time"

	_ "github.com/jbuchbinder/gopnm" // for reading pnm files

	astar "github.com/nkawa/astar_golang"
)

func SetRoute(field *Field, route [][2]float64) {

	for field.pixels == nil {
		log.Printf("Wait for update")
		time.Sleep(time.Millisecond * 300)
	}

	for i := range route {
		p := route[len(route)-i-1] // reverse order
		field.SetPoint(int(p[0]), int(p[1]), 0xff0000)
		time.Sleep(time.Millisecond * 3)
	}

}

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

func findRoute(field *Field, aStar *astar.Astar) {
	log.Printf("Wait for display ready")
	startOk := <-InitChannel
	log.Println("findRoute:display ready Wait 3sec", startOk)
	time.Sleep(time.Second * 3) // wait 3 seconds
	log.Println("findRoute: do route!")
	if startOk {
		for {
			cp := <-ClickChan
			route, err := aStar.Plan(float64(cp.X0), float64(cp.Y0), float64(cp.X1), float64(cp.Y1)) //from point(10,10) to point(120,120)
			if err != nil {
				fmt.Print(err, "\n")
			} else {
				fmt.Print("route length:", len(route), "\n")
			}
			SetRoute(field, route)
		}
	}
}

func main() {
	var objects [][2]float64
	var err error
	var wg sync.WaitGroup

	myField := &Field{}

	//	objects, err = astar.ObjectsFromImage("imgs/maiz.pgm", 100, 0, 0, 1)
	objects, err = astar.ObjectsFromImage("imgs/higashiyamaGhalf.pnm", 200, 0, 0, 1)
	if err != nil || objects == nil {
		log.Printf("Can't open map %v", err)
	}
	myField.Objects = objects
	// for visualizing route
	//	plot, _ := glot.NewPlot(2, false, false)
	//	plot.AddPointGroup("map", "points", convert2DPoint(objects))

	objectRadius := 1.0 //passable size 1.0m
	//	gridResolution := 0.5 //grid size 0.5m
	gridResolution := 1.0 //grid size 0.5m
	log.Printf("Generate Astar")

	aStar := astar.NewAstar(objects, objectRadius, gridResolution)
	aStar.UpdateObj = EbitenUpdate{
		EField: myField,
	}
	log.Printf("Do routing")

	go findRoute(myField, aStar)
	//	route, err := aStar.Plan(120, 120, 10, 10) //from point(10,10) to point(120,120)

	//	bt, err := json.Marshal(route)
	//	if err == nil {
	//		log.Printf("Marshal \n%s\n", string(bt))
	//	}
	// visualize route
	//	plot.AddPointGroup("route", "points", convert2DPoint(route))
	//	plot.SavePlot("imgs/route.png")

	wg.Add(1)

	RunEbiten(myField) // start display

	wg.Wait()
}
