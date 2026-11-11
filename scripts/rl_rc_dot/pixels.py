"""BT.709 TV-range conversion reused from the frozen baseline pipeline."""
import ctypes as C
import ctypes.util
import numpy as np
import subprocess

W,H,FB=480,320,480*320*3//2

class RGB709:
    """libswscale BT.709 TV I420 -> full-range RGB; byte-checked against FFmpeg."""
    def __init__(self):
        lib = ctypes.util.find_library('swscale')
        util = ctypes.util.find_library('avutil')
        if not lib or not util:
            raise RuntimeError('libswscale/libavutil shared libraries required; setup installs libswscale-dev')
        self.s = C.CDLL(lib)
        self.u = C.CDLL(util)
        self.u.av_get_pix_fmt.argtypes = [C.c_char_p]
        self.u.av_get_pix_fmt.restype = C.c_int
        self.s.sws_getContext.argtypes = [C.c_int]*7 + [C.c_void_p]*3
        self.s.sws_getContext.restype = C.c_void_p
        self.s.sws_getCoefficients.argtypes = [C.c_int]
        self.s.sws_getCoefficients.restype = C.POINTER(C.c_int)
        self.s.sws_setColorspaceDetails.argtypes = [C.c_void_p,C.POINTER(C.c_int),C.c_int,C.POINTER(C.c_int),C.c_int,C.c_int,C.c_int,C.c_int]
        self.s.sws_setColorspaceDetails.restype = C.c_int
        self.s.sws_scale.argtypes = [C.c_void_p,C.POINTER(C.POINTER(C.c_uint8)),C.POINTER(C.c_int),C.c_int,C.c_int,C.POINTER(C.POINTER(C.c_uint8)),C.POINTER(C.c_int)]
        self.s.sws_scale.restype = C.c_int
        self.s.sws_freeContext.argtypes = [C.c_void_p]
        self.ctx = self.s.sws_getContext(W,H,self.u.av_get_pix_fmt(b'yuv420p'),W,H,self.u.av_get_pix_fmt(b'rgb24'),4, None,None,None)
        if not self.ctx:
            raise RuntimeError('sws_getContext failed')
        coeff = self.s.sws_getCoefficients(1)
        if self.s.sws_setColorspaceDetails(self.ctx,coeff,0,coeff,1,0,1<<16,1<<16) < 0:
            raise RuntimeError('sws_setColorspaceDetails failed')

    def __call__(self, yuv):
        yuv = np.ascontiguousarray(yuv, dtype=np.uint8).reshape(-1)
        if len(yuv) != FB:
            raise ValueError('RGB input frame size mismatch')
        rgb = np.empty((H,W,3),np.uint8)
        ptr = C.POINTER(C.c_uint8)
        src = (ptr*4)(*[C.cast(C.c_void_p(yuv.ctypes.data+off),ptr) for off in (0,W*H,W*H*5//4)], ptr())
        dst = (ptr*4)(rgb.ctypes.data_as(ptr),ptr(),ptr(),ptr())
        if self.s.sws_scale(self.ctx,src,(C.c_int*4)(W,W//2,W//2,0),0,H,dst,(C.c_int*4)(W*3,0,0,0)) != H:
            raise RuntimeError('sws_scale failed')
        return rgb

    def close(self):
        if self.ctx:
            self.s.sws_freeContext(self.ctx)
            self.ctx = None

def verify_conversion(ffmpeg="ffmpeg"):
    """Require byte equality with the frozen FFmpeg RGB conversion protocol."""
    raw=np.random.default_rng(7).integers(16,236,size=FB,dtype=np.uint8)
    converter=RGB709()
    try:actual=converter(raw).tobytes()
    finally:converter.close()
    command=[str(ffmpeg),"-hide_banner","-nostdin","-loglevel","error","-f","rawvideo",
             "-pixel_format","yuv420p","-video_size","480x320","-framerate","30","-i","pipe:0",
             "-vf","scale=in_range=tv:out_range=pc:in_color_matrix=bt709:out_color_matrix=bt709,format=rgb24",
             "-pix_fmt","rgb24","-frames:v","1","-f","rawvideo","pipe:1"]
    expected=subprocess.run(command,input=raw.tobytes(),capture_output=True,check=True,timeout=30).stdout
    if actual!=expected:raise ValueError("RGB709 differs byte-for-byte from the locked FFmpeg protocol")
    return {"rgb_bytes":len(actual),"byte_exact":True}

