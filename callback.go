package hackrf

// #cgo windows CFLAGS: -I.
// #cgo windows LDFLAGS: -lhackrf -L.
// #include "hackrf.h"
import "C"

import (
	"reflect"
	"unsafe"
)

var localCallback CallbackFunc

//export go_callback
func go_callback(transfer C.hackrf_transfer) C.int {
	var buf []int8
	slice := (*reflect.SliceHeader)(unsafe.Pointer(&buf))
	slice.Data = uintptr(unsafe.Pointer(transfer.buffer))
	slice.Len = int(transfer.valid_length)
	slice.Cap = int(transfer.valid_length)

	return C.int(localCallback(buf))
}
