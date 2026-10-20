#!/usr/bin/env python3
"""Robot-local training store and display-only texture evaluation."""
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from features import image_feature_map, patch_vectors_from_map, patch_positions, fit_mlp_classifier, mlp_predict_arrays

DEFAULTS = dict(roiWidth=60, startY=30, stopY=65, patchSize=32, minimumRed=25)

def settings(value):
    if not isinstance(value, dict) or set(value) != set(DEFAULTS):
        raise ValueError('All five training settings are required')
    for key, low, high in [('roiWidth',10,100),('startY',0,90),('stopY',5,100),('patchSize',16,96),('minimumRed',1,100)]:
        v = value[key]
        if type(v) not in (int,float) or not np.isfinite(v) or int(v) != v or not low <= v <= high:
            raise ValueError('Invalid '+key)
    if value['stopY'] - value['startY'] < 5:
        raise ValueError('stopY must be at least 5% below startY')
    return {k:int(v) for k,v in value.items()}

def roi_for(config, width=640, height=360):
    x = round(width*(100-config['roiWidth'])/200)
    y = round(height*config['startY']/100)
    return dict(x=x,y=y,width=width-2*x,height=round(height*config['stopY']/100)-y)

def atomic(path, data):
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_bytes(data)
    os.replace(temporary, path)

def encode(image, quality=85):
    ok, data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY,quality])
    if not ok: raise ValueError('JPEG encoding failed')
    return data.tobytes()

def read_frame(path):
    data = path.read_bytes()
    if len(data) < 9: raise ValueError('No camera image')
    stamp, = struct.unpack('<Q', data[:8])
    if not 0 <= time.monotonic()*1000-stamp < 2000:
        raise ValueError('Camera image is stale; enable robot camera first')
    image = cv2.imdecode(np.frombuffer(data[8:],np.uint8),cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (360,640): raise ValueError('Invalid camera image')
    return stamp,image

class Store:
    def __init__(self, root, runtime, model, extractor=None):
        self.root=Path(root); self.root.mkdir(parents=True,exist_ok=True)
        self.run=Path(runtime); self.run.mkdir(parents=True,exist_ok=True)
        self.samples=self.root/'samples'; self.samples.mkdir(exist_ok=True)
        self.lock=threading.RLock(); self.npu=threading.Lock()
        self.config=settings(json.loads((self.root/'settings.json').read_text())) if (self.root/'settings.json').exists() else DEFAULTS.copy()
        self.model=Path(model); self.model_hash=hashlib.sha256(self.model.read_bytes()).hexdigest()
        self.extractor=extractor; self.classifier=None; self.error=''; self.job={'state':'idle'}
        self.last_stamp=0; self.evaluation=None
        if (self.root/'classifier.npz').exists():
            with np.load(self.root/'classifier.npz',allow_pickle=False) as c:
                if str(c['modelHash'])==self.model_hash: self.classifier={k:c[k].copy() for k in c.files}
    def feature_map(self, image):
        with self.npu:
            if self.extractor is None:
                from rknn_runtime import Runtime
                self.extractor=Runtime(self.model)
            fmap=image_feature_map(self.extractor,cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
        return fmap
    def vectors(self, image, config):
        return patch_vectors_from_map(image.shape,self.feature_map(image),roi_for(config),config['patchSize'])
    def items(self):
        return sorted(self.samples.glob('*.json'),key=lambda p:p.stat().st_mtime,reverse=True)
    def class_counts(self):
        counts={"obstacle":0,"no obstacle":0}
        for path in self.items():
            label=json.loads(path.read_text())["label"]
            counts[label]+=1
        return counts
    def rpc(self, request):
        action=request.get('action')
        with self.lock:
            if action=='status':
                return dict(config=self.config, count=len(self.items()), classCounts=self.class_counts(), trained=self.classifier is not None, job=self.job, error=self.error, evaluation=self.evaluation, evaluationOnly=True)
            if action=='configure':
                self.config=settings(request.get('config'))
                atomic(self.root/'settings.json',json.dumps(self.config).encode())
                return dict(config=self.config)
            if action=='list':
                offset=request.get('offset',0)
                if type(offset)!=int or offset<0: raise ValueError('Invalid page')
                paths=self.items(); result=[]
                for path in paths[offset:offset+12]:
                    meta=json.loads(path.read_text())
                    thumb=path.with_suffix('.thumb.jpg')
                    result.append(dict(id=path.stem,label=meta['label'],roi=meta['roi'],thumbnail=base64.b64encode(thumb.read_bytes()).decode()))
                return dict(images=result,total=len(paths),offset=offset,classCounts=self.class_counts())
            if action in ('save','delete','train') and self.job['state']=='running':
                raise ValueError('Training is running; wait before changing samples')
            if action=='save':
                label=request.get('label')
                if label not in ('obstacle','no obstacle'): raise ValueError('Invalid label')
                if len(self.items())>=500: raise ValueError('Limit of 500 training images reached')
                config=settings(request.get('config'))
                stamp,image=read_frame(self.run/'frame.bin')
                ident=uuid.uuid4().hex
                roi=roi_for(config); thumb=cv2.resize(image,(160,90))
                color=(0,0,255) if label=='obstacle' else (0,220,0)
                cv2.rectangle(thumb,(round(roi['x']/4),round(roi['y']/4)),(round((roi['x']+roi['width'])/4),round((roi['y']+roi['height'])/4)),color,2)
                atomic(self.samples/(ident+'.jpg'),encode(image,95))
                atomic(self.samples/(ident+'.thumb.jpg'),encode(thumb,65))
                atomic(self.samples/(ident+'.json'),json.dumps(dict(label=label,config=config,roi=roi,created=time.time(),captureMs=stamp)).encode())
                return dict(id=ident)
            if action=='delete':
                ident=request.get('id','')
                if not isinstance(ident,str) or not re.fullmatch('[0-9a-f]{32}',ident): raise ValueError('Invalid image ID')
                path=self.samples/(ident+'.json')
                if not path.exists(): raise ValueError('Image does not exist')
                path.unlink()
                for ext in ('.jpg','.thumb.jpg'):(self.samples/(ident+ext)).unlink(missing_ok=True)
                return dict(deleted=ident)
            if action=='train':
                paths=self.items()
                labels={json.loads(p.read_text())['label'] for p in paths}
                if labels!={'obstacle','no obstacle'}: raise ValueError('Save at least one image of each class first')
                self.job=dict(state='running',done=0,total=len(paths)); self.error=''
                threading.Thread(target=self.train,args=(paths,),daemon=True).start()
                return dict(job=self.job)
            raise ValueError('Unknown training action')
    def train(self, paths):
        try:
            groups={'obstacle':[],'no obstacle':[]}
            # Each image retains its own ROI and patch size from capture time.
            for i,path in enumerate(paths):
                meta=json.loads(path.read_text()); image=cv2.imread(str(path.with_suffix('.jpg')))
                if image is None: raise ValueError('Unreadable training image '+path.stem)
                vectors=self.vectors(image,meta['config'])
                if not vectors.size: raise ValueError('ROI contains no complete patches')
                # Bound memory and prevent large ROIs dominating smaller examples.
                if len(vectors)>64:vectors=vectors[np.linspace(0,len(vectors)-1,64,dtype=int)]
                groups[meta['label']].append(vectors)
                with self.lock:self.job=dict(state='running',done=i+1,total=len(paths))
            normal=np.vstack(groups['no obstacle']); obstacle=np.vstack(groups['obstacle'])
            data=np.vstack([normal,obstacle]); mean=data.mean(0); std=np.maximum(data.std(0),.025)
            fit=fit_mlp_classifier((normal-mean)/std,(obstacle-mean)/std)
            model=dict(mean=mean,std=std,w1=fit['w1'],b1=fit['b1'],w2=fit['w2'],b2=np.array(fit['b2']),modelHash=np.array(self.model_hash))
            with open(self.root/'classifier.tmp','wb') as f:np.savez(f,**model)
            os.replace(self.root/'classifier.tmp',self.root/'classifier.npz')
            with self.lock:
                self.classifier=model
                self.job=dict(state='complete',images=len(paths),patches=len(data),epochs=fit['epochs'])
                self.last_stamp=0
        except Exception as error:
            with self.lock:self.job=dict(state='failed',message=str(error))
    def evaluate(self):
        while True:
            try:
                stamp,image=read_frame(self.run/'frame.bin')
                with self.lock:
                    config=self.config.copy(); model=self.classifier; training=self.job['state']=='running'
                if stamp==self.last_stamp: time.sleep(.1);continue
                self.last_stamp=stamp; start=time.monotonic()
                roi=roi_for(config); x,y,w,h=(roi[k] for k in ('x','y','width','height'))
                height,width=image.shape[:2]
                boxes=patch_positions(0,0,width,height,config['patchSize'],include_partial=True)
                evaluation=None; label='NOT TRAINED - DISPLAY ONLY'
                if training:label='TRAINING - DISPLAY ONLY'
                elif model is not None:
                    fmap=self.feature_map(image)
                    full_roi=dict(x=0,y=0,width=width,height=height)
                    vectors=patch_vectors_from_map(image.shape,fmap,full_roi,config['patchSize'],include_partial=True)
                    roi_vectors=patch_vectors_from_map(image.shape,fmap,roi,config['patchSize'])
                    if not len(roi_vectors):raise ValueError('ROI contains no complete patches')
                    p=mlp_predict_arrays((vectors-model['mean'])/model['std'],model['w1'],model['b1'],model['w2'],model['b2'])
                    red=p>=.5; full_ratio=float(red.mean())
                    roi_p=mlp_predict_arrays((roi_vectors-model['mean'])/model['std'],model['w1'],model['b1'],model['w2'],model['b2'])
                    roi_red=roi_p>=.5; ratio=float(roi_red.mean()); blocked=ratio>=config['minimumRed']/100
                    evaluation=dict(red=int(red.sum()),patches=len(red),redPercent=round(full_ratio*100,1),roiRed=int(roi_red.sum()),roiPatches=len(roi_red),roiRedPercent=round(ratio*100,1),obstacle=bool(blocked),captureMs=stamp)
                    overlay=image.copy()
                    for (px,py),is_red in zip(boxes,red):
                        cv2.rectangle(overlay,(px,py),(px+config['patchSize']-1,py+config['patchSize']-1),(0,0,240) if is_red else (0,200,0),-1)
                        cv2.rectangle(image,(px,py),(px+config['patchSize']-1,py+config['patchSize']-1),(230,230,230),1)
                    image=cv2.addWeighted(image,.25,overlay,.75,0)
                    for px,py in boxes:
                        cv2.rectangle(image,(px,py),(px+config["patchSize"]-1,py+config["patchSize"]-1),(30,30,30),1)
                    label=('ROI OBSTACLE' if blocked else 'ROI CLEAR')+f' {ratio*100:.0f}% | FULL {full_ratio*100:.0f}% RED - DISPLAY ONLY'
                cv2.rectangle(image,(x,y),(x+w,y+h),(0,200,255),2)
                cv2.rectangle(image,(0,0),(640,24),(0,0,0),-1)
                cv2.putText(image,label,(5,17),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
                atomic(self.run/'overlay.bin',struct.pack('<Q',stamp)+encode(image))
                with self.lock:self.evaluation=evaluation; self.error=''
            except FileNotFoundError: pass
            except Exception as error:
                with self.lock:self.error=str(error); self.evaluation=None
            time.sleep(.2)

class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup(); self.connection.settimeout(2)
    def log_message(self,*args):pass
    def do_POST(self):
        try:
            length=int(self.headers.get('Content-Length','0'))
            if self.path!='/rpc' or not 0<length<=4096: raise ValueError('Invalid request')
            request=json.loads(self.rfile.read(length)); result=self.server.store.rpc(request)
            response=dict(ok=True,**result)
        except Exception as error: response=dict(ok=False,error=str(error))
        data=json.dumps(response,separators=(',',':')).encode()
        self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)

if __name__=='__main__':
    cv2.setNumThreads(1)
    store=Store(os.environ.get('SUNRAY_TRAINING_DATA','/home/orangepi/.local/share/sunray/training'),os.environ.get('SUNRAY_TEXTURE_RUNTIME','/dev/shm/sunray-texture'),Path(__file__).parent.parent/'models/texture-features.rknn')
    threading.Thread(target=store.evaluate,daemon=True).start()
    server=ThreadingHTTPServer(('127.0.0.1',18741),Handler);server.store=store;server.serve_forever()
