import tempfile,time,struct,json,threading
from pathlib import Path
import numpy as np
from service import Store,DEFAULTS,atomic,encode
from features import mlp_predict_arrays
with tempfile.TemporaryDirectory(prefix='sunray-npu-test-') as directory:
    root=Path(directory)
    store=Store(root/'data',root/'run',Path(__file__).resolve().parent.parent/'models/texture-features.rknn')
    images=[]
    for label,color in [('obstacle',(10,20,220)),('no obstacle',(20,220,10))]:
        image=np.empty((360,640,3),np.uint8);image[:]=color;images.append(image)
        atomic(store.run/'frame.bin',struct.pack('<Q',int(time.monotonic()*1000))+encode(image))
        store.rpc(dict(action='save',label=label,config=DEFAULTS))
    t=time.monotonic();store.train(store.items());assert store.job['state']=='complete',store.job
    print('Real NPU training:',store.job,'seconds',round(time.monotonic()-t,2))
    assert store.classifier is not None
    store.last_stamp=0
    atomic(store.run/'frame.bin',struct.pack('<Q',int(time.monotonic()*1000))+encode(images[0]))
    threading.Thread(target=store.evaluate,daemon=True).start()
    for _ in range(50):
        if store.evaluation:break
        time.sleep(.1)
    assert store.evaluation and store.evaluation['patches']==240 and store.evaluation['roiPatches']==36,(store.evaluation,store.error)
    assert (store.run/'overlay.bin').stat().st_size>100
    print('Real NPU patch evaluation:',store.evaluation)
    restored=Store(root/'data',root/'run',Path(__file__).resolve().parent.parent/'models/texture-features.rknn')
    assert restored.classifier is not None
    print('PASS local RKNN extraction, MLP training, colored patch overlay, persistent classifier')
