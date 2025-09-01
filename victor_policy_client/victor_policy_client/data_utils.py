"""Data utils ported from Zixuan Huang"""
from typing import Dict
import numpy as np
import torch
import multiprocessing as mp
import ctypes

def to_numpy(x):
    if torch.is_tensor(x):
        return x.detach().cpu().numpy()
    elif isinstance(x, dict):
        return {k: to_numpy(v) for k, v in x.items()}
    elif isinstance(x, list):
        return [to_numpy(v) for v in x]
    else:
        return x

class SmartDict(Dict):
    """
    A slicable dictionary for numpy and tensor object.
    d = {"a": torch.ones(8, 3), "b": torch.ones(8, 4)}
    custom_dict = SlicedDict(d)
    sliced_result = custom_dict[:3]
    sliced_result = custom_dict[:3, 1]
    sliced_result = custom_dict[0, 0]
    sliced_result = custom_dict[:3, 1:2]
    """

    def __init__(self, *args, backend="torch", dtype="float", **kwargs):
        """Transform a nested Dict to SmartDict
        
        Args:
            *args: Arguments to pass to Dict constructor
            backend (str): The backend to use for tensor operations ("numpy" or "torch" or None)
            dtype (str): Data type to use ("float" or "half")
            **kwargs: Keyword arguments to pass to Dict constructor
        """
        super().__init__(*args, **kwargs)
        self.backend = backend
        if backend == "torch":
            self.dtype = torch.float if dtype == "float" else torch.half
        elif backend == "numpy":
            self.dtype = np.float32 if dtype == "float" else np.float16
        else:
            raise ValueError(f"Invalid backend: {backend}")
        # Transform nested dicts to SmartDict with same backend
        for k, v in self.items():
            if isinstance(v, dict):
                self[k] = SmartDict(v, backend=backend, dtype=dtype)

    def __repr__(self, level=0):
        """Print the nested structure of the SmartDict"""
        ret = ""
        for key, value in self.items():
            if isinstance(value, dict):
                ret += "  " * level + f"{key}:\n"
                ret += value.__repr__(level + 1)
            elif torch.is_tensor(value) or isinstance(value, np.ndarray):
                ret += "  " * level + f"{key}: {value.shape}  {value.dtype}\n"
            elif isinstance(value, list):
                ret += "  " * level + f"{key}: list {len(value)}\n"
            elif isinstance(value, int) or isinstance(value, float):
                ret += "  " * level + f"{key}: scalar\n"
        return ret

    def __str__(self):
        return self.__repr__()

    def apply(self, func):
        return dict_apply(self, func)

    @property
    def device(self):
        for v in self.values():
            if torch.is_tensor(v):
                return v.device
        return None

    def _to_shared_memory(self):
        # Convert arrays in self.dataset to shared memory arrays, preserving dtype
        np_to_ctypes = {
            np.dtype('float16'): ctypes.c_float,
            np.dtype('float32'): ctypes.c_float,
            np.dtype('float64'): ctypes.c_double,
            np.dtype('int32'): ctypes.c_int32,
            np.dtype('int64'): ctypes.c_int64,
            np.dtype('uint8'): ctypes.c_uint8,
            np.dtype('int8'): ctypes.c_int8,
            np.dtype('uint16'): ctypes.c_uint16,
            np.dtype('int16'): ctypes.c_int16,
            # Add more types as needed
        }
        for key in self:
            arr = self[key]
            if torch.is_tensor(arr):
                arr = arr.share_memory_()
            elif isinstance(arr, np.ndarray):
                arr_dtype = arr.dtype
                if arr_dtype not in np_to_ctypes:
                    raise TypeError(f"Unsupported dtype {arr_dtype} for shared array.")
                c_type = np_to_ctypes[arr_dtype]
                shared_array_base = mp.Array(c_type, int(np.prod(arr.shape)))
                shared_array = np.ctypeslib.as_array(shared_array_base.get_obj())
                shared_array = shared_array.reshape(arr.shape)
                np.copyto(shared_array, arr)
                self[key] = shared_array

    def __len__(self):
        for v in self.values():
            if hasattr(v, "__len__"):
                return len(v)
        return 0

    def __setitem__(self, key, value):
        if isinstance(key, int):
            raise TypeError("Integer keys are not allowed in SmartDict")
        if isinstance(key, str):
            super().__setitem__(key, value)
        elif isinstance(key, slice) or isinstance(key, tuple):
            assert len(value) == len(
                self
            ), "The length of the value should be the same as the length of the dict"
            for k, v in self.items():
                v[key] = value[k]

    def __getitem__(self, key):
        if (
            isinstance(key, slice)
            or torch.is_tensor(key)
            or isinstance(key, np.ndarray)
            or isinstance(key, tuple)
            or key is None
            or isinstance(key, int)
        ):
            sliced_dict = SmartDict()
            # check if the values are of the same length
            if key is not None:
                lens = [len(v) for v in self.values()]
                assert len(set(lens)) == 1, "Try to slice a SmartDict with varied length of values."

            for k, v in self.items():
                sliced_dict[k] = v[key]
            return sliced_dict
        else:
            return super().__getitem__(key)

    def add(self, key, value):
        if torch.is_tensor(value) and self.backend == "numpy":
            value = value.detach().cpu().numpy().astype(self.dtype)
        elif isinstance(value, np.ndarray) and self.backend == "torch":
            value = torch.from_numpy(value).type(self.dtype)
        if key not in self.keys():
            self[key] = [value]
        else:
            self[key].append(value)

    def to_numpy(self, dtype=np.float32):
        new_dict = SmartDict()
        for k, v in self.items():
            if torch.is_tensor(v):
                new_dict[k] = v.detach().float().cpu().numpy().astype(dtype)
            elif isinstance(v, SmartDict):
                new_dict[k] = v.to_numpy(dtype)
            else:
                new_dict[k] = v.astype(dtype)
        return new_dict

    def to_tensor(self, device="cuda", dtype=torch.float32):
        new_dict = SmartDict()
        for k, v in self.items():
            if isinstance(v, np.ndarray):
                new_dict[k] = torch.tensor(v, device=device, dtype=dtype)
            elif torch.is_tensor(v):
                new_dict[k] = v.to(device, dtype=dtype)
            elif isinstance(v, Dict) or isinstance(v, SmartDict):
                new_dict[k] = v.to_tensor(device, dtype)
            elif isinstance(v, list):
                if isinstance(v[0], SmartDict):
                    new_dict[k] = [x.to_tensor(device, dtype) for x in v]
                elif isinstance(v[0], np.ndarray) or isinstance(v[0], int):
                    new_dict[k] = [torch.tensor(x, device=device, dtype=dtype) for x in v]
            else:
                new_dict[k] = v
        return new_dict

    def to(self, device):
        return self.to_tensor(device)

    def clone(self):
        """Deep copy of the SmartDict"""
        return self.apply(lambda x: x.clone() if torch.is_tensor(x) else x.copy())

    def copy(self):
        """Shallow copy of the SmartDict"""
        return SmartDict(super().copy())
    
    def __add__(self, other):
        out = SmartDict()
        for k in self.keys():
            out[k] = self.get(k, 0) + other[k]
        return out

    def __iadd__(self, other):
        for k in other.keys():
            self[k] = self.get(k, 0) + other[k]
        return self

    def __div__(self, other: float | Dict):
        out = SmartDict()
        for k in self.keys():
            out[k] = self.get(k, 0) / other
        return out

class SmartQueue(SmartDict):
    def __init__(self, max_size, reverse_queue=False, *args, **kwargs):
        """
        A queue that can be sliced and stacked.
        reverse_queue: If True, the queue is reversed. New item is added to the front of the queue.
        """
        super().__init__(*args, **kwargs)
        self.max_size = max_size
        self.reverse_queue = reverse_queue

    def add(self, key, value):
        if torch.is_tensor(value) and self.backend == "numpy":
            value = value.detach().cpu().numpy()
        elif isinstance(value, np.ndarray) and self.backend == "torch":
            value = torch.from_numpy(value)
        if key not in self.keys():
            self[key] = [value] * self.max_size
        else:
            if self.reverse_queue:
                self[key].pop()
                self[key].insert(0, value)
            else:
                self[key].pop(0)
                self[key].append(value)

