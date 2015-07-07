package hackrf

/*
#cgo windows CFLAGS: -I.
#cgo windows LDFLAGS: -lhackrf -L.
#include "hackrf.h"

extern void go_callback(hackrf_transfer);
hackrf_sample_block_cb_fn get_go_cb() {
	return (hackrf_sample_block_cb_fn)(go_callback);
}
*/
import "C"

import (
	"fmt"
	"unsafe"
)

type HackRFError int

const (
	Success             HackRFError = 0
	True                HackRFError = 1
	InvalidParam        HackRFError = -2
	NotFound            HackRFError = -5
	Busy                HackRFError = -6
	NoMem               HackRFError = -11
	LibUSB              HackRFError = -1000
	Thread              HackRFError = -1001
	StreamingThreadErr  HackRFError = -1002
	StreamingStopped    HackRFError = -1003
	StreamingExitCalled HackRFError = -1004
	Other               HackRFError = -9999
)

func (e HackRFError) String() string {
	switch e {
	case Success:
		return "Success"
	case True:
		return "True"
	case InvalidParam:
		return "InvalidParam"
	case NotFound:
		return "NotFound"
	case Busy:
		return "Busy"
	case NoMem:
		return "NoMem"
	case LibUSB:
		return "LibUSB"
	case Thread:
		return "Thread"
	case StreamingThreadErr:
		return "StreamingThreadErr"
	case StreamingStopped:
		return "StreamingStopped"
	case StreamingExitCalled:
		return "StreamingExitCalled"
	case Other:
		return "Other"
	}

	return "InvalidError"
}

func (e HackRFError) Error() string {
	return e.String()
}

func Error(e C.int) error {
	if e == 0 {
		return nil
	}
	return HackRFError(e)
}

type BoardID byte

const (
	Jellybean BoardID = iota
	Jawbreaker
	HackRFOne
)

func (bid BoardID) String() string {
	switch bid {
	case Jellybean:
		return "Jellybean"
	case Jawbreaker:
		return "Jawbreaker"
	case HackRFOne:
		return "HackRF One"
	}
	return "InvalidBoardID"
}

type HardwareID struct {
	PartID   [2]uint32
	SerialNo [4]uint32
}

func (hid HardwareID) String() string {
	return fmt.Sprintf("{PartID: %08X SerialNo: %08X}", hid.PartID, hid.SerialNo)
}

type HackRF struct {
	ptr *C.hackrf_device
}

// hackrf_init()
func (h HackRF) Init() error {
	return Error(C.hackrf_init())
}

// hackrf_open(hackrf_device** device)
func (h *HackRF) Open() error {
	h.ptr = new(C.hackrf_device)
	return Error(C.hackrf_open((**C.hackrf_device)(&h.ptr)))
}

// hackrf_close(hackrf_device* device)
func (h *HackRF) Close() error {
	return Error(C.hackrf_close((*C.hackrf_device)(h.ptr)))
}

// hackrf_exit()
func (h *HackRF) Exit() error {
	return Error(C.hackrf_exit())
}

type CallbackFunc func([]int8) int

// hackrf_start_rx(hackrf_device* device, hackrf_sample_block_cb_fn callback, void* rx_ctx)
func (h *HackRF) StartRX(callback CallbackFunc) error {
	localCallback = callback

	return Error(C.hackrf_start_rx(
		(*C.hackrf_device)(h.ptr),
		(C.hackrf_sample_block_cb_fn)(C.get_go_cb()),
		nil,
	))
}

// hackrf_stop_rx(hackrf_device* device)
func (h *HackRF) StopRX() error {
	return Error(C.hackrf_stop_rx(
		(*C.hackrf_device)(h.ptr),
	))
}

// hackrf_start_tx(hackrf_device* device, hackrf_sample_block_cb_fn callback, void* tx_ctx);
func (h *HackRF) StartTX(callback CallbackFunc) error {
	localCallback = callback

	return Error(C.hackrf_start_tx(
		(*C.hackrf_device)(h.ptr),
		(C.hackrf_sample_block_cb_fn)(C.get_go_cb()),
		nil,
	))
}

// hackrf_stop_tx(hackrf_device* device);
func (h *HackRF) StopTX() error {
	return Error(C.hackrf_stop_tx(
		(*C.hackrf_device)(h.ptr),
	))
}

// hackrf_board_id_read(hackrf_device* device, uint8_t* value)
func (h HackRF) BoardID() (bid BoardID, err error) {
	err = Error(C.hackrf_board_id_read(
		(*C.hackrf_device)(h.ptr),
		(*C.uint8_t)(unsafe.Pointer(&bid))),
	)
	return bid, err
}

// hackrf_board_partid_serialno_read(hackrf_device* device, read_partid_serialno_t* read_partid_serialno)
func (h HackRF) HardwareID() (hid HardwareID, err error) {
	err = Error(C.hackrf_board_partid_serialno_read(
		(*C.hackrf_device)(h.ptr),
		(*C.read_partid_serialno_t)(unsafe.Pointer(&hid))),
	)
	return hid, err
}

// hackrf_version_string_read(hackrf_device* device, char* version, uint8_t length)
func (h HackRF) Version() (str string, err error) {
	version := make([]C.char, 256)
	err = Error(C.hackrf_version_string_read(
		(*C.hackrf_device)(h.ptr),
		(*C.char)(&version[0]),
		255,
	))

	str = C.GoString((*C.char)(&version[0]))
	return str, err
}

// hackrf_is_streaming(hackrf_device* device)
func (h HackRF) IsStreaming() (isStreaming bool, err error) {
	result := Error(C.hackrf_is_streaming(
		(*C.hackrf_device)(h.ptr),
	))

	if result == StreamingThreadErr || result == True {
		err = nil
	}

	return result == True, err
}

// hackrf_max2837_read(hackrf_device* device, uint8_t register_number, uint16_t* value)
func (h HackRF) Max2837Read(register uint8, value uint16) error {
	return Error(C.hackrf_max2837_read(
		(*C.hackrf_device)(h.ptr),
		(C.uint8_t)(register),
		(*C.uint16_t)(&value),
	))
}

// hackrf_max2837_write(hackrf_device* device, uint8_t register_number, uint16_t value)
func (h HackRF) Max2837Write(register uint8, value uint16) error {
	return Error(C.hackrf_max2837_write(
		(*C.hackrf_device)(h.ptr),
		(C.uint8_t)(register),
		(C.uint16_t)(value),
	))
}

// hackrf_si5351c_read(hackrf_device* device, uint16_t register_number, uint16_t* value)
func (h HackRF) SI5351CRead(register, value uint16) error {
	return Error(C.hackrf_si5351c_read(
		(*C.hackrf_device)(h.ptr),
		(C.uint16_t)(register),
		(*C.uint16_t)(&value),
	))
}

// hackrf_si5351c_write(hackrf_device* device, uint16_t register_number, uint16_t value)
func (h HackRF) SI5351CWrite(register, value uint16) error {
	return Error(C.hackrf_si5351c_write(
		(*C.hackrf_device)(h.ptr),
		(C.uint16_t)(register),
		(C.uint16_t)(value),
	))
}

// hackrf_rffc5071_read(hackrf_device* device, uint8_t register_number, uint16_t* value)
func (h HackRF) RFFC5071Read(register uint8, value uint16) error {
	return Error(C.hackrf_rffc5071_read(
		(*C.hackrf_device)(h.ptr),
		(C.uint8_t)(register),
		(*C.uint16_t)(&value),
	))
}

// hackrf_rffc5071_write(hackrf_device* device, uint8_t register_number, uint16_t value)
func (h HackRF) RFFC5071Write(register uint8, value uint16) error {
	return Error(C.hackrf_rffc5071_write(
		(*C.hackrf_device)(h.ptr),
		(C.uint8_t)(register),
		(C.uint16_t)(value),
	))
}

// hackrf_spiflash_erase(hackrf_device* device)
func (h HackRF) SPIFlashErase() error {
	return Error(C.hackrf_spiflash_erase(
		(*C.hackrf_device)(h.ptr),
	))
}

// hackrf_spiflash_read(hackrf_device* device, const uint32_t address, const uint16_t length, unsigned char* data)
func (h HackRF) SPIFlashRead(address uint32, length uint16, data []uint8) error {
	return Error(C.hackrf_spiflash_read(
		(*C.hackrf_device)(h.ptr),
		(C.uint32_t)(address),
		(C.uint16_t)(length),
		(*C.uchar)(&data[0]),
	))
}

// hackrf_spiflash_write(hackrf_device* device, const uint32_t address, const uint16_t length, unsigned char* const data)
func (h HackRF) SPIFlashWrite(address uint32, length uint16, data []uint8) error {
	return Error(C.hackrf_spiflash_write(
		(*C.hackrf_device)(h.ptr),
		(C.uint32_t)(address),
		(C.uint16_t)(length),
		(*C.uchar)(&data[0]),
	))
}

// hackrf_cpld_write(hackrf_device* device, unsigned char* const data, const unsigned int total_length)
func (h HackRF) CPLDWrite(data []uint8, length uint) error {
	return Error(C.hackrf_cpld_write(
		(*C.hackrf_device)(h.ptr),
		(*C.uchar)(&data[0]),
		(C.uint)(length),
	))
}

// hackrf_set_freq(hackrf_device* device, const uint64_t freq_hz)
func (h HackRF) SetFreq(freq uint64) error {
	return Error(C.hackrf_set_freq(
		(*C.hackrf_device)(h.ptr),
		(C.uint64_t)(freq),
	))
}

// hackrf_set_baseband_filter_bandwidth(hackrf_device* device, const uint32_t bandwidth_hz)
func (h HackRF) SetBasebandFilterBandwidth(bandwidth uint32) error {
	return Error(C.hackrf_set_baseband_filter_bandwidth(
		(*C.hackrf_device)(h.ptr),
		(C.uint32_t)(bandwidth),
	))
}

var BasebandFilterBandwidth = []uint32{
	1.75e6,
	2.50e6,
	3.50e6,
	5e6,
	5.5e6,
	6e6,
	7e6,
	8e6,
	9e6,
	10e6,
	12e6,
	14e6,
	15e6,
	20e6,
	24e6,
	28e6,
}

// Valid/preferred sample rates (MHz):
// 8, 10, 12.5, 16, 20

type RFPathFilter uint32

const (
	ByPass RFPathFilter = iota
	LowPass
	HighPass
)

func (rfpf RFPathFilter) String() string {
	switch rfpf {
	case ByPass:
		return "ByPass"
	case LowPass:
		return "LowPass"
	case HighPass:
		return "HighPass"
	}
	return "InvalidRFPathFilter"
}

// hackrf_set_freq_explicit(hackrf_device* device, const uint64_t if_freq_hz, const uint64_t lo_freq_hz, const enum rf_path_filter path)
func (h HackRF) SetFreqExplicit(ifFreq, loFreq uint64, rfFilterPath RFPathFilter) error {
	return Error(C.hackrf_set_freq_explicit(
		(*C.hackrf_device)(h.ptr),
		(C.uint64_t)(ifFreq),
		(C.uint64_t)(loFreq),
		uint32(rfFilterPath),
	))
}

// hackrf_set_sample_rate(hackrf_device* device, const double freq_hz)
func (h HackRF) SetSampleRate(freq float64) error {
	return Error(C.hackrf_set_sample_rate(
		(*C.hackrf_device)(h.ptr),
		(C.double)(freq),
	))
}

// hackrf_set_sample_rate_manual(hackrf_device* device, const uint32_t freq_hz, const uint32_t divider)
func (h HackRF) SetSampleRateManual(freq, divider uint32) error {
	return Error(C.hackrf_set_sample_rate_manual(
		(*C.hackrf_device)(h.ptr),
		(C.uint32_t)(freq),
		(C.uint32_t)(divider),
	))
}

// hackrf_set_amp_enable(hackrf_device* device, const uint8_t value)
func (h HackRF) SetAmp(enable bool) error {
	var value byte
	if enable {
		value = 1
	}
	return Error(C.hackrf_set_amp_enable(
		(*C.hackrf_device)(h.ptr),
		(C.uint8_t)(value),
	))
}

// hackrf_set_lna_gain(hackrf_device* device, uint32_t value)
func (h HackRF) SetLNAGain(db uint32) error {
	return Error(C.hackrf_set_lna_gain(
		(*C.hackrf_device)(h.ptr),
		(C.uint32_t)(db),
	))
}

// hackrf_set_vga_gain(hackrf_device* device, uint32_t value)
func (h HackRF) SetVGAGain(db uint32) error {
	return Error(C.hackrf_set_vga_gain(
		(*C.hackrf_device)(h.ptr),
		(C.uint32_t)(db),
	))
}

// hackrf_set_txvga_gain(hackrf_device* device, uint32_t value)
func (h HackRF) SetTXVGAGain(db uint32) error {
	return Error(C.hackrf_set_txvga_gain(
		(*C.hackrf_device)(h.ptr),
		(C.uint32_t)(db),
	))
}

// hackrf_set_antenna_enable(hackrf_device* device, const uint8_t value)
func (h HackRF) SetAntennaPower(enable bool) error {
	var value byte
	if enable {
		value = 1
	} else {
		value = 0
	}
	return Error(C.hackrf_set_antenna_enable(
		(*C.hackrf_device)(h.ptr),
		(C.uint8_t)(value),
	))
}

// uint32_t hackrf_compute_baseband_filter_bw_round_down_lt(const uint32_t bandwidth_hz);
func ComputeBaseBandFilterBWNearest(freq uint32) (bw uint32) {
	return uint32(C.hackrf_compute_baseband_filter_bw_round_down_lt((C.uint32_t)(freq)))
}

// uint32_t hackrf_compute_baseband_filter_bw(const uint32_t bandwidth_hz);
func ComputeBaseBandFilterBWBest(freq uint32) (bw uint32) {
	return uint32(C.hackrf_compute_baseband_filter_bw((C.uint32_t)(freq)))
}
