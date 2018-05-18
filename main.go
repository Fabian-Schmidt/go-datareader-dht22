package main

import (
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"math"
	"time"

	"github.com/fabian-schmidt/go-datareader-dht22/dht22"
	rpio "github.com/fabian-schmidt/go-datareader-dht22/go-rpio"
)

// AM2302/DHT22 - digital relative humidity and temperature sensor
// https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf

const (
	waitTimeBetweenRetry = 2 * time.Second
	numberOfRetries      = 5
)

var (
	pin                   int
	errMaxRetriesExceeded = errors.New("max retries exceeded")
	errParameterMissing   = errors.New("invalid or missing parameter")
)

type Output struct {
	Temperatur float32
	Humidity   float32
	//Vapor Pressure Deficit
	VPD   float64
	Retry int
}

func init() {
	flag.IntVar(&pin, "pin", 0, "pin")
}

func main() {
	flag.Parse()

	if pin <= 0 {
		panic(errParameterMissing)
	}

	out, err := readWithRetry(pin, numberOfRetries)
	if err != nil {
		panic(err)
	}

	calcVPD(*out)

	b, err := json.Marshal(*out)
	if err != nil {
		panic(err)
	}

	fmt.Println(string(b[:]))
}

func calcVPD(data Output) {
	// calculate vpd
	// J. Win. (https://physics.stackexchange.com/users/1680/j-win),
	// How can I calculate Vapor Pressure Deficit from Temperature and Relative Humidity?,
	// URL (version: 2011-02-03): https://physics.stackexchange.com/q/4553
	temperature64 := float64(data.Temperatur)
	humidity64 := float64(data.Humidity)

	es := 0.6108 * math.Exp(17.27*temperature64/(temperature64+237.3))
	ea := humidity64 / 100 * es

	// this equation returns a negative value (in kPa), which while technically correct,
	// is invalid in this case because we are talking about a deficit.
	vpd := (ea - es) * -1

	data.VPD = vpd
}

func readWithRetry(pinNumber int, maxRetry int) (*Output, error) {

	for iteration := 0; iteration < maxRetry; iteration++ {
		out, err := readData(pinNumber)
		if err != nil {
			iteration++
			time.Sleep(waitTimeBetweenRetry)
		} else {
			(*out).Retry = iteration
			return out, nil
		}
	}

	return nil, errMaxRetriesExceeded
}

func readData(pinNumber int) (*Output, error) {
	err := rpio.Open()
	if err != nil {
		return nil, err
	}
	defer rpio.Close()

	dataPin := rpio.Pin(pinNumber)

	data, err := dht22.ReadData(dataPin, false)
	if err != nil {
		return nil, err
	}

	out := Output{data.Temperatur, data.Humidity, 0, 0}
	return &out, nil
}
