import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import ctypes.util
import shutil
import unittest
from scripts.rl_rc_dot.pixels import verify_conversion

@unittest.skipUnless(ctypes.util.find_library("swscale") and ctypes.util.find_library("avutil") and shutil.which("ffmpeg"),
                     "Requires shared libswscale/libavutil and FFmpeg (Linux CI/Kaggle setup)")
class ColorProtocol(unittest.TestCase):
    def test_exact_frozen_ffmpeg_rgb_conversion(self):
        self.assertTrue(verify_conversion()["byte_exact"])

if __name__=="__main__":unittest.main()
