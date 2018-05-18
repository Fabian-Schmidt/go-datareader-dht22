/*
Package rpio provides GPIO access on the Raspberry PI without any need
for external c libraries (eg. WiringPi or BCM2835).

Supports simple operations such as:
	- Pin mode/direction (input/output)
	- Pin write (high/low)
	- Pin read (high/low)
	- Pull up/down/off

Example of use:

	rpio.Open()
	defer rpio.Close()

	pin := rpio.Pin(4)
	pin.Output()

	for {
		pin.Toggle()
		time.Sleep(time.Second)
	}

The library use the raw BCM2835 pinouts, not the ports as they are mapped
on the output pins for the raspberry pi, and not the wiringPi convention.

            Rev 2 and 3 Raspberry Pi                        Rev 1 Raspberry Pi (legacy)
  +-----+---------+----------+---------+-----+      +-----+--------+----------+--------+-----+
  | BCM |   Name  | Physical | Name    | BCM |      | BCM | Name   | Physical | Name   | BCM |
  +-----+---------+----++----+---------+-----+      +-----+--------+----++----+--------+-----+
  |     |    3.3v |  1 || 2  | 5v      |     |      |     | 3.3v   |  1 ||  2 | 5v     |     |
  |   2 |   SDA 1 |  3 || 4  | 5v      |     |      |   0 | SDA    |  3 ||  4 | 5v     |     |
  |   3 |   SCL 1 |  5 || 6  | 0v      |     |      |   1 | SCL    |  5 ||  6 | 0v     |     |
  |   4 | GPIO  7 |  7 || 8  | TxD     | 14  |      |   4 | GPIO 7 |  7 ||  8 | TxD    |  14 |
  |     |      0v |  9 || 10 | RxD     | 15  |      |     | 0v     |  9 || 10 | RxD    |  15 |
  |  17 | GPIO  0 | 11 || 12 | GPIO  1 | 18  |      |  17 | GPIO 0 | 11 || 12 | GPIO 1 |  18 |
  |  27 | GPIO  2 | 13 || 14 | 0v      |     |      |  21 | GPIO 2 | 13 || 14 | 0v     |     |
  |  22 | GPIO  3 | 15 || 16 | GPIO  4 | 23  |      |  22 | GPIO 3 | 15 || 16 | GPIO 4 |  23 |
  |     |    3.3v | 17 || 18 | GPIO  5 | 24  |      |     | 3.3v   | 17 || 18 | GPIO 5 |  24 |
  |  10 |    MOSI | 19 || 20 | 0v      |     |      |  10 | MOSI   | 19 || 20 | 0v     |     |
  |   9 |    MISO | 21 || 22 | GPIO  6 | 25  |      |   9 | MISO   | 21 || 22 | GPIO 6 |  25 |
  |  11 |    SCLK | 23 || 24 | CE0     | 8   |      |  11 | SCLK   | 23 || 24 | CE0    |   8 |
  |     |      0v | 25 || 26 | CE1     | 7   |      |     | 0v     | 25 || 26 | CE1    |   7 |
  |   0 |   SDA 0 | 27 || 28 | SCL 0   | 1   |      +-----+--------+----++----+--------+-----+
  |   5 | GPIO 21 | 29 || 30 | 0v      |     |
  |   6 | GPIO 22 | 31 || 32 | GPIO 26 | 12  |
  |  13 | GPIO 23 | 33 || 34 | 0v      |     |
  |  19 | GPIO 24 | 35 || 36 | GPIO 27 | 16  |
  |  26 | GPIO 25 | 37 || 38 | GPIO 28 | 20  |
  |     |      0v | 39 || 40 | GPIO 29 | 21  |
  +-----+---------+----++----+---------+-----+

See the spec for full details of the BCM2835 controller:

https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2835/BCM2835-ARM-Peripherals.pdf
and https://elinux.org/BCM2835_datasheet_errata - for errors in that spec

*/
package rpio

import (
	"bytes"
	"encoding/binary"
	"os"
	"reflect"
	"sync"
	"syscall"
	"time"
	"unsafe"
)

type Mode uint8
type Pin uint8
type State uint8
type Pull uint8

// Memory offsets for gpio, see the spec for more details
const (
	bcm2835Base = 0x20000000
	gpioOffset  = 0x200000

	memLength = 4096
)

var (
	gpioBase int64
)

func init() {
	base := getBase()
	gpioBase = base + gpioOffset
}

// Pin mode, a pin can be set in Input or Output mode
const (
	Input Mode = iota
	Output
)

// State of pin, High / Low
const (
	Low State = iota
	High
)

// Pull Up / Down / Off
const (
	PullOff Pull = iota
	PullDown
	PullUp
)

// Arrays for 8 / 32 bit access to memory and a semaphore for write locking
var (
	memlock  sync.Mutex
	gpioMem  []uint32
	gpioMem8 []uint8
)

// Set pin as Input
func (pin Pin) Input() {
	PinMode(pin, Input)
}

// Set pin as Output
func (pin Pin) Output() {
	PinMode(pin, Output)
}

// Set pin High
func (pin Pin) High() {
	WritePin(pin, High)
}

// Set pin Low
func (pin Pin) Low() {
	WritePin(pin, Low)
}

// Toggle pin state
func (pin Pin) Toggle() {
	TogglePin(pin)
}

// Set pin Mode
func (pin Pin) Mode(mode Mode) {
	PinMode(pin, mode)
}

// Set pin state (high/low)
func (pin Pin) Write(state State) {
	WritePin(pin, state)
}

// Read pin state (high/low)
func (pin Pin) Read() State {
	return ReadPin(pin)
}

// Set a given pull up/down mode
func (pin Pin) Pull(pull Pull) {
	PullMode(pin, pull)
}

// Pull up pin
func (pin Pin) PullUp() {
	PullMode(pin, PullUp)
}

// Pull down pin
func (pin Pin) PullDown() {
	PullMode(pin, PullDown)
}

// Disable pullup/down on pin
func (pin Pin) PullOff() {
	PullMode(pin, PullOff)
}

// PinMode sets the mode (direction) of a given pin (Input, Output)
func PinMode(pin Pin, mode Mode) {

	// Pin fsel register, 0 or 1 depending on bank
	fselReg := uint8(pin) / 10
	shift := (uint8(pin) % 10) * 3
	f := uint32(0)

	const alt0 = 4 // 100
	const alt5 = 2 // 010

	switch mode {
	case Input:
		f = 0 // 000
	case Output:
		f = 1 // 001
	}

	memlock.Lock()
	defer memlock.Unlock()

	const pinMask = 7 // 0b111 - pinmode is 3 bits

	gpioMem[fselReg] = (gpioMem[fselReg] &^ (pinMask << shift)) | (f << shift)
}

// WritePin sets a given pin High or Low
// by setting the clear or set registers respectively
func WritePin(pin Pin, state State) {

	p := uint8(pin)

	// Clear register, 10 / 11 depending on bank
	// Set register, 7 / 8 depending on bank
	clearReg := p/32 + 10
	setReg := p/32 + 7

	memlock.Lock()
	defer memlock.Unlock()

	if state == Low {
		gpioMem[clearReg] = 1 << (p & 31)
	} else {
		gpioMem[setReg] = 1 << (p & 31)
	}

}

// Read the state of a pin
func ReadPin(pin Pin) State {
	// Input level register offset (13 / 14 depending on bank)
	levelReg := uint8(pin)/32 + 13

	if (gpioMem[levelReg] & (1 << uint8(pin&31))) != 0 {
		return High
	}

	return Low
}

// Toggle a pin state (high -> low -> high)
// TODO: probably possible to do this much faster without read
func TogglePin(pin Pin) {
	switch ReadPin(pin) {
	case Low:
		pin.High()
	case High:
		pin.Low()
	}
}

func PullMode(pin Pin, pull Pull) {
	// Pull up/down/off register has offset 38 / 39, pull is 37
	pullClkReg := pin/32 + 38
	pullReg := 37
	shift := pin % 32

	memlock.Lock()
	defer memlock.Unlock()

	switch pull {
	case PullDown, PullUp:
		gpioMem[pullReg] = gpioMem[pullReg]&^3 | uint32(pull)
	case PullOff:
		gpioMem[pullReg] = gpioMem[pullReg] &^ 3
	}

	// Wait for value to clock in, this is ugly, sorry :(
	time.Sleep(time.Nanosecond * 50)

	gpioMem[pullClkReg] = 1 << shift

	// Wait for value to clock in
	time.Sleep(time.Nanosecond * 50)

	gpioMem[pullReg] = gpioMem[pullReg] &^ 3
	gpioMem[pullClkReg] = 0

}

// Open and memory map GPIO memory range from /dev/gpiomem .
// Some reflection magic is used to convert it to a unsafe []uint32 pointer
func Open() (err error) {
	var file *os.File

	// try gpiomem (some extra functions like clock and pwm setting wont work)
	file, err = os.OpenFile("/dev/gpiomem", os.O_RDWR|os.O_SYNC, 0)
	if err != nil {
		return
	}
	// FD can be closed after memory mapping
	defer file.Close()

	memlock.Lock()
	defer memlock.Unlock()

	// Memory map GPIO registers to slice
	gpioMem, gpioMem8, err = memMap(file.Fd(), gpioBase)
	if err != nil {
		return
	}

	return nil
}

func memMap(fd uintptr, base int64) (mem []uint32, mem8 []byte, err error) {
	mem8, err = syscall.Mmap(
		int(fd),
		base,
		memLength,
		syscall.PROT_READ|syscall.PROT_WRITE,
		syscall.MAP_SHARED,
	)
	if err != nil {
		return
	}
	// Convert mapped byte memory to unsafe []uint32 pointer, adjust length as needed
	header := *(*reflect.SliceHeader)(unsafe.Pointer(&mem8))
	header.Len /= (32 / 8) // (32 bit = 4 bytes)
	header.Cap /= (32 / 8)
	mem = *(*[]uint32)(unsafe.Pointer(&header))
	return
}

// Close unmaps GPIO memory
func Close() error {
	memlock.Lock()
	defer memlock.Unlock()
	if err := syscall.Munmap(gpioMem8); err != nil {
		return err
	}

	return nil
}

// Read /proc/device-tree/soc/ranges and determine the base address.
// Use the default Raspberry Pi 1 base address if this fails.
func getBase() (base int64) {
	base = bcm2835Base
	ranges, err := os.Open("/proc/device-tree/soc/ranges")
	defer ranges.Close()
	if err != nil {
		return
	}
	b := make([]byte, 4)
	n, err := ranges.ReadAt(b, 4)
	if n != 4 || err != nil {
		return
	}
	buf := bytes.NewReader(b)
	var out uint32
	err = binary.Read(buf, binary.BigEndian, &out)
	if err != nil {
		return
	}
	return int64(out)
}
