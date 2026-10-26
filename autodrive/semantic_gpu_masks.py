"""Keep semantic logits on CUDA and copy only byte masks back to the CPU."""
import onnxruntime as ort
import numpy as np
import torch


def mask_tensors(logits, active_ids, threshold):
    cutoff = np.log(threshold / (1.0 - threshold))
    if not 0 < threshold < 1:
        raise ValueError('threshold must be between zero and one')
    compare_dtype = torch.float64 if np.result_type(np.dtype('float32'), cutoff) == np.float64 else logits.dtype
    masks = []
    for values, ids in zip(logits, active_ids):
        selected = torch.as_tensor(ids, dtype=torch.long, device=logits.device)
        if selected.numel() == 0:
            raise ValueError('no active semantic classes')
        best_values, best = values.index_select(0, selected).max(dim=0)
        mask = selected[best].to(torch.uint8)
        mask.masked_fill_(best_values.to(compare_dtype) < float(cutoff), 255)
        masks.append(mask)
    return torch.stack(masks)


class GpuMaskInference:
    def __init__(self, session, output_name, active_ids, class_count, width, height, threshold):
        if session.get_providers()[0] != 'CUDAExecutionProvider':
            raise RuntimeError('GPU mask inference requires CUDA')
        if not 0 < threshold < 1:
            raise ValueError('threshold must be between zero and one')
        output = next(x for x in session.get_outputs() if x.name == output_name)
        if output.type != 'tensor(float)':
            raise ValueError('GPU mask inference requires float32 logits')
        self.session, self.input_name = session, session.get_inputs()[0].name
        self.active_ids, self.threshold = active_ids, threshold
        self.logits = torch.empty((len(active_ids), class_count, height, width), dtype=torch.float32, device='cuda:0')
        self.run_options = ort.RunOptions()
        self.run_options.only_execute_path_to_fetches = True
        self.binding = session.io_binding()
        self.binding.bind_output(output_name, device_type='cuda', device_id=0,
                                 element_type=np.float32, shape=tuple(self.logits.shape), buffer_ptr=self.logits.data_ptr())

    def run(self, batch):
        # Input remains identical to OpenCV's established preprocessing.
        batch = np.ascontiguousarray(batch, dtype=np.float32)
        self.binding.bind_cpu_input(self.input_name, batch)
        self.session.run_with_iobinding(self.binding, self.run_options)
        self.binding.synchronize_outputs()
        result = mask_tensors(self.logits, self.active_ids, self.threshold).cpu().numpy()
        return [result[i] for i in range(len(result))]
