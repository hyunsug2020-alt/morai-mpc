"""Numerical equivalence and timing against the existing CPU semantic mask path."""
import json,sys,time,statistics
from pathlib import Path
import cv2,numpy as np,torch,onnxruntime as ort

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'autodrive'),str(ROOT/'semantic/pipeline')]
from semantic_gpu_masks import GpuMaskInference,mask_tensors
from run_onnx import prediction_masks,preprocess
from camera_config import CAMERA_PAIRS
views=('front','left','right');active=[[i for i in CAMERA_PAIRS[v]['supervised_class_ids'] if i!=4] for v in views]
torch.set_num_threads(1);cv2.setNumThreads(1)
rng=np.random.default_rng(7)
for threshold in (.25,.5,.75):
 x=rng.standard_normal((3,6,64,96)).astype(np.float32)
 x[:,:,0:3,0:3]=0
 x[:,5,0,0]=np.nan
 cutoff=np.float32(np.log(threshold/(1-threshold)))
 x[:,:,10,10:13]=-100
 x[:,0,10,10:13]=[np.nextafter(cutoff,np.float32(-np.inf)),cutoff,np.nextafter(cutoff,np.float32(np.inf))]
 expected=np.stack(prediction_masks(x,views,'logits',threshold,(4,)))
 actual=mask_tensors(torch.from_numpy(x).cuda(),active,threshold).cpu().numpy()
 assert np.array_equal(expected,actual),(threshold,np.count_nonzero(expected!=actual))
print('Random, tie, NaN and threshold-boundary masks match exactly',flush=True)
model=ROOT/'semantic/models/semantic_b1_noyellow_v2_final_20261002/best_fp16.onnx'
meta=json.loads(model.with_suffix('.onnx.json').read_text())
with np.load('/frames/frames.npz') as frames:
 images=[cv2.imdecode(frames[v+'_jpeg'],cv2.IMREAD_REDUCED_COLOR_2 if v=='front' else cv2.IMREAD_COLOR) for v in views]
batch=np.stack([preprocess(im,640,384)[0] for im in images])
opts=ort.SessionOptions();opts.intra_op_num_threads=2;opts.inter_op_num_threads=1
opts.add_session_config_entry('session.intra_op.allow_spinning','0');opts.add_session_config_entry('session.inter_op.allow_spinning','0')
session=ort.InferenceSession(str(model),sess_options=opts,providers=['CUDAExecutionProvider','CPUExecutionProvider'])
gpu=GpuMaskInference(session,'logits',active,len(meta['class_names']),640,384,float(meta['threshold']))
def cpu():
 return prediction_masks(session.run(['logits'],{'input':batch})[0],views,'logits',float(meta['threshold']),(4,))
def accelerated():return gpu.run(batch)
for _ in range(3):cpu();accelerated()
expected=np.stack(cpu());actual=np.stack(accelerated())
assert np.array_equal(expected,actual),np.count_nonzero(expected!=actual)
result={'outputs_equal':True,'tested_thresholds':[.25,.5,.75]}
for name,fn in [('cpu_postprocess',cpu),('gpu_postprocess',accelerated)]:
 wall=[];cpu_time=[]
 for _ in range(30):
  a=time.perf_counter();b=time.process_time();fn();cpu_time.append((time.process_time()-b)*1000);wall.append((time.perf_counter()-a)*1000)
 result[name]={'median_wall_ms':round(statistics.median(wall),3),'p95_wall_ms':round(sorted(wall)[int(.95*(len(wall)-1))],3),'median_cpu_ms':round(statistics.median(cpu_time),3)}
print(json.dumps(result),flush=True)
if len(sys.argv)>1:Path(sys.argv[1]).write_text(json.dumps(result,indent=2)+'\n')
