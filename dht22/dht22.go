package dht22

import (
	"errors"
	"fmt"
	"time"

	rpio "github.com/fabian-schmidt/go-datareader-dht22/go-rpio"
)

var (
	errSensorNotFound = errors.New("sensor not found or not ready")
	errChecksum       = errors.New("checksum error")
	errHumidity       = errors.New("humidity range error")
	errTemperature    = errors.New("temperature range error")
)

const (
	logicalHighTreshold = (27 + (70-27)/2) * time.Microsecond
	dataLength          = 41
)

type data struct {
	Temperatur float32
	Humidity   float32
}

func ReadData(dataPin rpio.Pin, debugMode bool) (*data, error) {
	//Check signal is high
	dataPin.Mode(rpio.Input)
	var start = dataPin.Read()
	if start != rpio.High {
		return nil, errSensorNotFound
	}

	var (
		// early allocations before time critical code
		lengths  = make([]time.Duration, dataLength)
		iterator = 0
	)

	// Send dial pulse.
	dataPin.Mode(rpio.Output)

	// Set pin to low.
	dataPin.Write(rpio.Low)

	// Sleep 1 millisecond.
	time.Sleep(time.Millisecond)

	// Switch pin to input mode
	dataPin.Mode(rpio.Input)

	time.Sleep(time.Microsecond)

	// Read bunch of data from sensor
	// for futher processing in high level language.
	// Wait for next pulse 10ms maximum.
	for {
		duration, err := dataPin.TimePulse(rpio.High, time.Millisecond)
		if err != nil {
			if debugMode {
				//Remove the first data point.
				lengths = lengths[1:]

				fmt.Printf("1. Byte: %v\n", lengths[:8])
				fmt.Printf("2. Byte: %v\n", lengths[8:16])
				fmt.Printf("3. Byte: %v\n", lengths[16:24])
				fmt.Printf("4. Byte: %v\n", lengths[24:32])
				fmt.Printf("5. Byte: %v\n", lengths[32:40])
			}
			return nil, err
		}
		lengths[iterator] = duration
		iterator++
		if iterator >= dataLength {
			break
		}
	}
	//Remove the first data point.
	lengths = lengths[1:]

	if debugMode {
		fmt.Printf("1. Byte: %v\n", lengths[:8])
		fmt.Printf("2. Byte: %v\n", lengths[8:16])
		fmt.Printf("3. Byte: %v\n", lengths[16:24])
		fmt.Printf("4. Byte: %v\n", lengths[24:32])
		fmt.Printf("5. Byte: %v\n", lengths[32:40])
	}

	// convert to bytes
	bytes := make([]byte, 5)

	for i := range bytes {
		for j := 0; j < 8; j++ {
			bytes[i] <<= 1
			if lengths[i*8+j] > logicalHighTreshold {
				bytes[i] |= 0x01
			}
		}
	}

	if debugMode {
		fmt.Printf("Bytes: %v\n", bytes)
	}

	if err := checksum(bytes); err != nil {
		if debugMode {
			fmt.Println(errChecksum.Error)
		} else {
			return nil, errChecksum
		}
	}

	var (
		humidity    uint16
		temperature uint16
		data        data
	)

	// calculate humidity
	humidity |= uint16(bytes[0])
	humidity <<= 8
	humidity |= uint16(bytes[1])

	if humidity < 0 || humidity > 1000 {
		if debugMode {
			fmt.Println(errHumidity.Error)
		} else {
			return nil, errHumidity
		}
	}

	data.Humidity = float32(humidity) / 10

	// calculate temperature
	temperature |= uint16(bytes[2])
	temperature <<= 8
	temperature |= uint16(bytes[3])

	// check for negative temperature
	if temperature&0x8000 > 0 {
		data.Temperatur = float32(temperature&0x7FFF) / -10
	} else {
		data.Temperatur = float32(temperature) / 10
	}

	// datasheet operating range
	if data.Temperatur < -40 || data.Temperatur > 80 {
		if debugMode {
			fmt.Println(errTemperature.Error)
		} else {
			return nil, errTemperature
		}
	}

	return &data, nil
}

func checksum(bytes []byte) error {
	calcSum := byte(bytes[0] + bytes[1] + bytes[2] + bytes[3])

	if bytes[4] != calcSum {
		return errChecksum
	}

	return nil
}
