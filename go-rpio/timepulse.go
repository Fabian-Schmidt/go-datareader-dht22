package rpio

import (
	"errors"
	"time"
)

const (
	sleepTime = 100
)

var (
	errTimeout = errors.New("Timeout error")
)

func (pin Pin) TimePulse(state State, maxWaitTime time.Duration) (time.Duration, error) {
	endTime := time.Now().Add(maxWaitTime)
	aroundState := Low
	if state == Low {
		aroundState = High
	}

	// Wait for any previous pulse to end
	for {
		v := pin.Read()
		time.Sleep(sleepTime)

		if v == aroundState {
			break
		}

		if time.Now().After(endTime) {
			return 0, errTimeout
		}
	}

	// Wait until ECHO goes high
	for {
		v := pin.Read()
		time.Sleep(sleepTime)

		if v == state {
			break
		}

		if time.Now().After(endTime) {
			return 0, errTimeout
		}
	}

	pulseStartTime := time.Now() // Record time when ECHO goes high

	// Wait until ECHO goes low
	for {
		v := pin.Read()
		time.Sleep(sleepTime)

		if v == aroundState {
			break
		}

		if time.Now().After(endTime) {
			return 0, errTimeout
		}
	}

	// Calculate time lapsed for ECHO to transition from high to low
	return time.Since(pulseStartTime), nil
}
