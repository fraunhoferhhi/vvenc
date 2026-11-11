import io
import unittest
from bridge import read_complete_line
from spec import STATE_FIELDS,parse_state


class GrowingStateFile(unittest.TestCase):
    def test_fragmented_record_waits_and_then_parses_once(self):
        record="DOT_PROXY 0 "+" ".join(["0.1"]*(STATE_FIELDS-2))+"\n"
        split=len(record)//2
        stream=io.StringIO(record[:split])
        self.assertEqual(read_complete_line(stream),"")
        self.assertEqual(stream.tell(),0)
        self.assertEqual(read_complete_line(stream),"")
        stream.seek(0,io.SEEK_END)
        stream.write(record[split:])
        stream.seek(0)
        line=read_complete_line(stream)
        self.assertEqual(parse_state(line,allow_proxy=True).poc,0)
        self.assertEqual(read_complete_line(stream),"")

    def test_partial_record_does_not_repeat_previous_complete_record(self):
        stream=io.StringIO("first\nsecond")
        self.assertEqual(read_complete_line(stream),"first\n")
        start=stream.tell()
        self.assertEqual(read_complete_line(stream),"")
        self.assertEqual(stream.tell(),start)
        stream.seek(0,io.SEEK_END)
        stream.write("\n")
        stream.seek(start)
        self.assertEqual(read_complete_line(stream),"second\n")
        self.assertEqual(read_complete_line(stream),"")

    def test_complete_malformed_record_is_still_rejected(self):
        with self.assertRaises(ValueError):
            parse_state(read_complete_line(io.StringIO("DOT_PROXY 0 0.1\n")),allow_proxy=True)


if __name__=="__main__":
    unittest.main()
