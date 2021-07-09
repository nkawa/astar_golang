package astar

import (
	"fmt"
	"math"

	"github.com/fukurin00/astar_golang/tool"
)

//Astar planning class
type Astar struct {
	Resolution float64

	MinX int
	MaxX int
	MinY int
	MaxY int

	XWidth   int
	YWidth   int
	MaxIndex int

	ObjMap [][]bool //if object, it is true
}

type Point struct {
	X float64
	Y float64
}

func NewAstar(objects [][2]float64, objectRadius, resolution float64) *Astar {
	a := new(Astar)
	a.Resolution = resolution
	var xList []float64
	var yList []float64

	for _, obj := range objects {
		xList = append(xList, obj[0])
		yList = append(yList, obj[1])
	}
	a.MaxX = int(math.Round(tool.MaxFloat(xList)))
	a.MaxY = int(math.Round(tool.MaxFloat(yList)))
	a.MinX = int(math.Round(tool.MinFloat(xList)))
	a.MinY = int(math.Round(tool.MinFloat(yList)))

	a.XWidth = int(math.Round(float64(a.MaxX)-float64(a.MinX)) / resolution)
	a.YWidth = int(math.Round(float64(a.MaxY)-float64(a.MinY)) / resolution)

	a.ObjMap = make([][]bool, a.XWidth+1)
	for i := 0; i <= a.XWidth; i++ {
		a.ObjMap[i] = make([]bool, a.YWidth)
	}

	var ind int
	for iy := 0; iy < a.YWidth; iy++ {
		y := indToPos(iy, a.MinY, resolution)
		for ix := 1; ix < a.XWidth; ix++ {
			x := indToPos(ix, a.MinX, resolution)
			ind = iy*a.XWidth + ix
			for i := 0; i < len(objects); i++ {
				d := math.Hypot(objects[i][0]-x, objects[i][1]-y)
				if d <= objectRadius {
					a.ObjMap[ix][iy] = true
					break
				}
			}
		}
	}
	a.MaxIndex = ind
	return a
}

type AstarNode struct {
	Index int
	Ix    int
	Iy    int

	Cost      float64
	PrevIndex int

	Obj bool //obstacleならtrue
}

// for astar constructor
func newNode(ix, iy int, cost float64, pind int) *AstarNode {
	n := new(AstarNode)
	n.Ix = ix
	n.Iy = iy
	n.Cost = cost
	n.PrevIndex = pind
	return n
}

func indToPos(index, minP int, reso float64) float64 {
	pos := float64(index)*reso + float64(minP)
	return pos
}

func (a Astar) indToPosXY(index int) (float64, float64) {
	px := float64(a.MinX) + float64(index%a.XWidth)*a.Resolution
	py := float64(a.MinY) + float64(index/a.XWidth)*a.Resolution
	return px, py
}

func posToInd(p float64, minp int, reso float64) int {
	return int(math.Round((p - float64(minp)) / reso))
}

func (a Astar) indToIndXY(index int) (int, int) {
	ix := index % a.XWidth
	iy := index / a.XWidth
	return ix, iy
}

func heuristic(n1, n2 *AstarNode) float64 {
	w := 1.0
	d := w * math.Hypot(float64(n1.Ix)-float64(n2.Ix), float64(n1.Iy)-float64(n2.Iy))
	return d
}

func (a Astar) verifyGrid(index int) bool {
	if index > a.MaxIndex {
		return false
	}
	px, py := a.indToPosXY(index)

	if px < float64(a.MinX) {
		return false
	} else if py < float64(a.MinY) {
		return false
	} else if px >= float64(a.MaxX) {
		return false
	} else if py >= float64(a.MaxY) {
		return false
	}

	ix, iy := a.indToIndXY(index)
	if a.ObjMap[ix][iy] {
		return false
	}
	return true
}

func (a Astar) nodeToInd(n *AstarNode) int {
	index := n.Iy*a.XWidth + n.Ix
	return index
}

func (a Astar) indToPos(index, minP int) float64 {
	pos := float64(index)*a.Resolution + float64(minP)
	return pos
}

// Astar planing (sx,sy) is start, (gx,gy) is goal point
func (a *Astar) Plan(sx, sy, gx, gy float64) (route [][2]float64, err error) {
	nstart := newNode(posToInd(sx, a.MinX, a.Resolution), posToInd(sy, a.MinY, a.Resolution), 0.0, -1)
	ngoal := newNode(posToInd(gx, a.MinX, a.Resolution), posToInd(gy, a.MinY, a.Resolution), 0.0, -1)

	if !a.verifyGrid(a.nodeToInd(nstart)) {
		err = fmt.Errorf("start point (%f, %f) is not verified", sx, sy)
		return route, err
	}
	if !a.verifyGrid(a.nodeToInd(ngoal)) {
		err = fmt.Errorf("goal point (%f, %f) is not verified", gx, gy)
		return route, err
	}

	open_set := make(map[int]*AstarNode)
	close_set := make(map[int]*AstarNode)
	open_set[a.nodeToInd(nstart)] = nstart

	for {
		if len(open_set) == 0 {
			err = fmt.Errorf("fail searching point from (%f,%f) to (%f, %f): open set is empty", sx, sy, gx, gy)
			return route, err
		}

		minCost := 1e19
		minKey := -1
		for key, val := range open_set {
			calCost := val.Cost + heuristic(ngoal, val)
			if calCost < minCost {
				minCost = calCost
				minKey = key
			}
		}
		cId := minKey
		current := open_set[cId]

		if current.Ix == ngoal.Ix && current.Iy == ngoal.Iy {
			//log.Print("find goal")
			ngoal.PrevIndex = current.PrevIndex
			ngoal.Cost = current.Cost
			route = a.finalPath(ngoal, close_set)
			return route, nil
		}

		delete(open_set, cId)

		close_set[cId] = current

		var nId int
		var node *AstarNode
		motion := [8][3]float64{{1.0, 0, 1.0}, {0, 1.0, 1.0}, {-1.0, 0, 1.0}, {0, -1.0, 1.0}, {-1.0, -1.0, math.Sqrt(2)}, {-1.0, 1.0, math.Sqrt(2)}, {1.0, -1.0, math.Sqrt(2)}, {1.0, 1.0, math.Sqrt(2)}}
		for _, v := range motion {
			node = newNode(current.Ix+int(v[0]), current.Iy+int(v[1]), current.Cost+v[2], cId)
			nId = a.nodeToInd(node)

			if !a.verifyGrid(a.nodeToInd(node)) {
				continue
			}

			if _, ok := close_set[nId]; ok {
				continue
			}

			if _, ok := open_set[nId]; !ok {
				open_set[nId] = node
			}
		}
	}
}

// 最後に経路の順番にする
func (a Astar) finalPath(ngoal *AstarNode, closeSet map[int]*AstarNode) (route [][2]float64) {
	route = append(route, [2]float64{a.indToPos(ngoal.Ix, a.MinX), a.indToPos(ngoal.Iy, a.MinY)})

	pind := ngoal.PrevIndex
	for pind != -1 {
		n := closeSet[pind]
		route = append(route, [2]float64{a.indToPos(n.Ix, a.MinX), a.indToPos(n.Iy, a.MinY)})
		pind = n.PrevIndex
	}
	return route
}
