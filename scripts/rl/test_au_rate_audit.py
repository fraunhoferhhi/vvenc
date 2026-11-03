"""Synthetic tests for VVenC AU bit accounting by POC and output order."""
import tempfile
import unittest
from pathlib import Path

from au_rate_audit import read_au_bits


class TestAUBits(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.bitstream = self.root / "stream.266"
        self.csv = self.root / "au_bits.csv"
        self.bitstream.write_bytes(bytes(range(20)))
        # Non-monotonic POC order intentionally tests GOP reordering.
        self.csv.write_text(
            "output_step,poc,au_bytes,au_bits\n"
            "1,0,5,40\n"
            "2,2,8,64\n"
            "3,1,7,56\n"
        )

    def tearDown(self):
        self.tmp.cleanup()

    def test_exact_bitstream_sum_and_reordered_poc(self):
        rows = read_au_bits(self.csv, self.bitstream, 3)
        self.assertEqual([r.poc for r in rows], [0, 2, 1])
        self.assertEqual([r.cumulative_bits for r in rows], [40, 104, 160])
        self.assertEqual(rows[-1].cumulative_bits, self.bitstream.stat().st_size * 8)

    def test_reject_missing_poc(self):
        self.csv.write_text(
            "output_step,poc,au_bytes,au_bits\n"
            "1,0,5,40\n2,2,8,64\n3,2,7,56\n"
        )
        with self.assertRaises(ValueError):
            read_au_bits(self.csv, self.bitstream, 3)

    def test_reject_wrong_bit_sum(self):
        self.bitstream.write_bytes(b"wrong-size")
        with self.assertRaises(ValueError):
            read_au_bits(self.csv, self.bitstream, 3)

    def test_reject_wrong_output_order(self):
        self.csv.write_text(
            "output_step,poc,au_bytes,au_bits\n"
            "1,0,5,40\n3,2,8,64\n2,1,7,56\n"
        )
        with self.assertRaises(ValueError):
            read_au_bits(self.csv, self.bitstream, 3)


if __name__ == "__main__":
    unittest.main()
