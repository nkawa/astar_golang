package astar

import (
	"image"
	"image/color"
	"log"
	"os"
)

// ObjectsFromImage read glay-scale image file and create objects list
func ObjectsFromImage(imgFile string, closeThreth, originX, originY, m2Pixel float64) ([][2]float64, error) {
	file, err := os.Open(imgFile)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	imData, _, err := image.Decode(file)
	if err != nil {
		return nil, err
	}

	bound := imData.Bounds()
	W := bound.Dx()
	H := bound.Dy()
	log.Printf("file loaded with %dx%d", W, H)

	data := make([][2]float64, 0)
	for j := H - 1; j >= 0; j-- {
		for i := 0; i < W; i++ {
			x := originX + m2Pixel*float64(i)
			//			y := originY + m2Pixel*float64(H-j-1)
			y := originY + m2Pixel*float64(j)
			oldPix := imData.At(i, j)
			pixel := color.GrayModel.Convert(oldPix)
			pixelU := color.GrayModel.Convert(pixel).(color.Gray).Y

			if float64(pixelU) < closeThreth {
				data = append(data, [2]float64{x, y})
			}
		}
	}
	return data, nil
}
