import tempfile
import unittest
from pathlib import Path
import struct
import time
import numpy as np
from service import Store, DEFAULTS, settings, encode, atomic

class FakeExtractor:
    def inference(self, inputs):
        # Deterministic input-dependent features, no NPU in storage unit tests.
        mean=inputs[0].mean(axis=(1,2)).reshape(1,3,1,1)
        return [np.tile(mean,(1,1,16,16)).transpose(0,2,3,1)]

class TrainingTest(unittest.TestCase):
    def test_full_image_grid_covers_edges_without_changing_training_roi(self):
        from features import patch_positions, patch_vectors_from_map
        for size in (16,32,33,96):
            boxes=patch_positions(0,0,640,360,size,include_partial=True)
            coverage=np.zeros((360,640),np.uint8)
            for x,y in boxes:coverage[y:y+size,x:x+size]+=1
            self.assertTrue(np.all(coverage==1))
            vectors=patch_vectors_from_map((360,640,3),np.ones((32,32,160),np.float32),dict(x=0,y=0,width=640,height=360),size,include_partial=True)
            self.assertEqual(vectors.shape,(len(boxes),320))
            self.assertTrue(np.isfinite(vectors).all())
        self.assertEqual(len(patch_positions(128,108,512,234,32)),36)

    def test_validation(self):
        for key,value in [('startY',70),('roiWidth',float('nan')),('patchSize',0)]:
            config={**DEFAULTS,key:value}
            with self.assertRaises(ValueError): settings(config)
    def test_storage_training_delete_and_restore(self):
        with tempfile.TemporaryDirectory() as root:
            root=Path(root); model=root/'model';model.write_bytes(b'test')
            store=Store(root/'data',root/'run',model,FakeExtractor())
            with self.assertRaises(ValueError):store.rpc(dict(action='train'))
            for label,color in [('obstacle',(10,20,220)),('no obstacle',(20,220,10))]:
                image=np.empty((360,640,3),np.uint8);image[:]=color
                atomic(store.run/'frame.bin',struct.pack('<Q',int(time.monotonic()*1000))+encode(image))
                store.rpc(dict(action='save',label=label,config=DEFAULTS))
            page=store.rpc(dict(action='list'))
            self.assertEqual(page['total'],2)
            self.assertEqual(page['images'][0]['roi'],dict(x=128,y=108,width=384,height=126))
            store.train(store.items())
            self.assertEqual(store.job['state'],'complete',store.job)
            restored=Store(root/'data',root/'run',model,FakeExtractor())
            self.assertIsNotNone(restored.classifier)
            with self.assertRaises(ValueError):store.rpc(dict(action='delete',id='../classifier'))
            ident=page['images'][0]['id'];store.rpc(dict(action='delete',id=ident))
            self.assertEqual(len(store.items()),1)
            self.assertFalse((store.samples/(ident+'.jpg')).exists())
            atomic(store.run/'frame.bin',struct.pack('<Q',1)+encode(image))
            with self.assertRaises(ValueError):store.rpc(dict(action='save',label='obstacle',config=DEFAULTS))

if __name__=='__main__':unittest.main()
