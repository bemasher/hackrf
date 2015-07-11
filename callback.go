package hackrf

// #cgo windows CFLAGS: -I.
// #cgo windows LDFLAGS: -lhackrf -L.
// #include "hackrf.h"
import "C"
import "unsafe"

const maxBufferLength = 262144

var localCallback CallbackFunc

//export go_callback
func go_callback(transfer C.hackrf_transfer) C.int {
	buf := (*[maxBufferLength]int8)(unsafe.Pointer(transfer.buffer))
	return C.int(localCallback(buf[:transfer.valid_length]))
}
