package main

import (
	"image/color"
	"log"
	"time"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"

	astar "github.com/nkawa/astar_golang"
)

type Field struct {
	Area        []bool
	OuterWidth  int
	OuterHeight int
	Objects     [][2]float64
	CObjects    [][3]int
	pixels      []byte
}

const (
	screenWidth  = 320
	screenHeight = 240
)

var started = false
var InitChannel chan bool

func init() {
	ebiten.SetWindowResizable(true)
	//	ebiten.SetWindowSize(screenWidth, screenHeight)
	ebiten.SetWindowSize(screenWidth*2, screenHeight*2)
	ebiten.SetWindowTitle("Astar Field")
	InitChannel = make(chan bool)
}

func pressedKey(str ebiten.Key) bool {
	inputArray := inpututil.PressedKeys()
	for _, v := range inputArray {
		if v == str {
			return true
		}
	}
	return false
}

// Update proceeds the game state.
// Update is called every tick (1/60 [s] by default).
func (g *Field) Update() error {
	// do routing step here!
	if !started {
		log.Printf("Update ebiten")
		//		if pressedKey(ebiten.KeyArrowUp) {
		InitChannel <- true
		started = true
		//		}
	}

	return nil
}

type EbitenUpdate struct {
	EField *Field
}

func setColor(p []byte, ix int, r, g, b, alpha byte) {
	p[ix] = r
	p[ix+1] = g
	p[ix+2] = b
	p[ix+3] = alpha
}

func (e EbitenUpdate) UpdateAstar(a *astar.Astar, color color.RGBA, wait int) {
	//	log.Printf("Update Aster %d, %v", a.Current.Index, color)
	cnode := a.Current
	if cnode != nil { // set current
		pix := e.EField.pixels
		if pix != nil {
			x := cnode.Ix
			y := cnode.Iy
			idx := int(y*screenWidth+x) * 4
			setColor(pix, idx, color.R, color.G, color.B, color.A)
			if wait > 0 {
				time.Sleep(time.Microsecond * time.Duration(wait))
			}
		}
	}
}

//func (g *Field) Draw(screen *ebiten.Image) {//
//	ebitenutil.DebugPrint(screen, "Hello world")
//}

func (g *Field) SetPoint(x, y int, color int) {
	if g.CObjects == nil {
		g.CObjects = make([][3]int, 0)
	}
	g.CObjects = append(g.CObjects, [3]int{x, y, color})
	//	log.Printf("Set %d %d %x", x, y, color)
}

// Draw draws the game screen.
// Draw is called every frame (typically 1/60[s] for 60Hz display).
func (g *Field) Draw(screen *ebiten.Image) {
	if g.pixels == nil {
		g.pixels = make([]byte, screenWidth*screenHeight*4)
		//		log.Printf("Len %d", len(g.pixels))
	}
	if g.Objects == nil {
		return
	}
	pix := g.pixels
	for _, o := range g.Objects { // draw pix for each objects
		x := o[0]
		y := o[1]
		idx := int((y*float64(screenWidth) + x)) * 4
		for k := 0; k < 3; k++ {
			pix[idx+k] = 0xff
		}
	}
	//	log.Printf("CObject len %d", len(g.CObjects))
	for _, o := range g.CObjects { // draw pix for each objects
		x := o[0]
		y := o[1]
		color := o[2]
		idx := int(y*screenWidth+x) * 4
		pix[idx] = 0xff //byte(color >> 24)
		pix[idx+1] = byte((color >> 16) % 0xff)
		pix[idx+2] = byte((color >> 8) % 0xff)
		pix[idx+3] = byte(color % 0xff)
	}
	// need to fit size
	screen.ReplacePixels(pix)
}

func (g *Field) Layout(outsideWidth, outsideHeight int) (int, int) {
	return screenWidth, screenHeight

	//	g.OuterWidth = outsideWidth
	//	g.OuterHeight = outsideHeight

	//	return outsideWidth, outsideHeight

}

func RunEbiten(myField *Field) {
	log.Printf("Starting Ebiten window")
	if err := ebiten.RunGame(myField); err != nil {
		log.Fatal(err)
	}
}