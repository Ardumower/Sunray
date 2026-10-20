import ctypes as C
from pathlib import Path
import numpy as np

class Runtime:
    def __init__(self, model):
        self.lib = C.CDLL(str(Path(__file__).with_name('librknn_features.so')))
        self.lib.texture_init.argtypes = [C.c_char_p, C.POINTER(C.c_uint64), C.POINTER(C.c_int)]
        self.lib.texture_run.argtypes = [C.c_uint64, C.c_void_p, C.c_void_p, C.c_uint]
        self.lib.texture_close.argtypes = [C.c_uint64]
        self.context = C.c_uint64()
        shape = (C.c_int * 4)()
        if self.lib.texture_init(str(model).encode(), C.byref(self.context), shape):
            raise RuntimeError('Cannot initialize texture RKNN model')
        self.shape = tuple(shape)
    def inference(self, inputs):
        data = np.ascontiguousarray(inputs[0], dtype=np.float32)
        if data.shape != (1, 512, 512, 3):
            raise ValueError('Expected normalized 512x512 RGB input')
        output = np.empty(self.shape, dtype=np.float32)
        if self.lib.texture_run(self.context, data.ctypes.data, output.ctypes.data, output.size):
            raise RuntimeError('Texture RKNN inference failed')
        return [output]
