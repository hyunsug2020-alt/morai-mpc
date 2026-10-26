import io,sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import torch
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'student_v3_claude'))
import gui


def jpeg(seed):
    data=np.random.default_rng(seed).integers(0,256,(72,96,3),dtype=np.uint8)
    out=io.BytesIO();Image.fromarray(data).save(out,format='JPEG');return out.getvalue()


def reference(rows,width,height):
    return torch.stack([torch.stack([torch.from_numpy((np.asarray(Image.open(io.BytesIO(x)).convert('RGB').resize((width,height),Image.Resampling.BILINEAR),dtype=np.float32)/255-gui.IMAGE_MEAN)/gui.IMAGE_STD).permute(2,0,1) for x in row]) for row in rows]).unsqueeze(0)


class ImageWindowCacheTest(unittest.TestCase):
    def test_shared_history_decodes_only_new_frames_and_keeps_exact_inputs(self):
        runtime=gui.Runtime.__new__(gui.Runtime);runtime.config={'width':64,'height':48}
        rows=[[jpeg(t*3+c) for c in range(3)] for t in range(4)]
        original=gui.Image.open
        with patch.object(gui.Image,'open',wraps=original) as opened:
            actual,display=runtime.image_tensor(rows);self.assertEqual(opened.call_count,12)
            opened.reset_mock();actual2,_=runtime.image_tensor(rows);self.assertEqual(opened.call_count,0)
            self.assertTrue(torch.equal(actual,actual2))
            new_rows=rows[1:]+[[jpeg(30+c) for c in range(3)]]
            opened.reset_mock();actual3,_=runtime.image_tensor(new_rows);self.assertEqual(opened.call_count,3)
        self.assertTrue(torch.equal(actual,reference(rows,64,48)))
        self.assertTrue(torch.equal(actual3,reference(new_rows,64,48)))
        saved=display[0].copy();display[0][:]=0
        _,display2=runtime.image_tensor(rows);self.assertTrue(np.array_equal(display2[0],saved))

    def test_eviction_is_bounded_and_resolution_changes_are_not_reused(self):
        runtime=gui.Runtime.__new__(gui.Runtime);runtime.config={'width':32,'height':24}
        first=[[jpeg(100+c) for c in range(3)]]
        for t in range(30):runtime.image_tensor([[jpeg(200+t*3+c) for c in range(3)]])
        self.assertLessEqual(len(runtime._image_cache),64)
        actual,_=runtime.image_tensor(first);self.assertTrue(torch.equal(actual,reference(first,32,24)))
        runtime.config={'width':64,'height':48}
        actual,_=runtime.image_tensor(first);self.assertTrue(torch.equal(actual,reference(first,64,48)))


if __name__=='__main__':unittest.main()
