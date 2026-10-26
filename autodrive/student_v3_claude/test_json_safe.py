"""Status topics and decision logs never raise on numpy values (follow-20261010T055542Z)."""
import json
import unittest

import numpy as np

from json_safe import json_default


class JsonSafeTests(unittest.TestCase):
    def test_numpy_scalars_and_arrays_become_plain_json(self):
        record = dict(flag=np.bool_(True), speed=np.float32(1.5), count=np.int64(3),
                      path=np.array([[1., 2.], [3., 4.]]), nested=[dict(ok=np.bool_(False))])
        self.assertEqual(json.loads(json.dumps(record, default=json_default)),
                         dict(flag=True, speed=1.5, count=3, path=[[1., 2.], [3., 4.]], nested=[dict(ok=False)]))

    def test_unknown_objects_are_written_as_text(self):
        self.assertEqual(json.loads(json.dumps(dict(f=len), default=json_default))['f'], str(len))


if __name__ == '__main__':
    unittest.main()
