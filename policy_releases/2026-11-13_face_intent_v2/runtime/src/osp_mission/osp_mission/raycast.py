"""Equivalent NumPy/native backends for the evaluator's private ray sensor."""

import ctypes
from functools import lru_cache
from pathlib import Path

import numpy as np


@lru_cache(maxsize=1)
def native_function():
    path=Path(__file__).with_name('_raycast.so')
    if not path.exists():raise RuntimeError('Native raycast missing: run python3 training_robust/build_raycast.py')
    library=ctypes.CDLL(str(path))
    function=library.osp_raycast
    array=lambda dtype:np.ctypeslib.ndpointer(dtype=dtype,flags='C_CONTIGUOUS')
    function.argtypes=[array(np.uint8),array(np.int8),ctypes.c_int,ctypes.c_int,
        ctypes.c_double,ctypes.c_double,ctypes.c_double,array(np.float64),array(np.float64),ctypes.c_int,
        array(np.float64),ctypes.c_int,array(np.int32),array(np.int32),array(np.int32),array(np.uint8),array(np.uint8)]
    function.restype=None
    return function


def cast(truth,observed,x,y,resolution,angles,ranges,backend='numpy'):
    cosine=np.cos(angles);sine=np.sin(angles)
    if backend=='native':
        ys=np.empty(len(angles),dtype=np.int32);xs=np.empty_like(ys);first=np.empty_like(ys)
        inside=np.empty(len(angles),dtype=np.uint8);hits=np.empty_like(inside)
        native_function()(np.ascontiguousarray(truth,dtype=np.uint8),observed,*truth.shape,
            x,y,resolution,cosine,sine,len(angles),ranges,len(ranges),ys,xs,first,inside,hits)
        return ys,xs,first,inside.astype(bool),hits.astype(bool)
    if backend!='numpy':raise ValueError('Unknown raycast backend')
    xx=np.floor((x+cosine[:,None]*ranges)/resolution).astype(int)
    yy=np.floor((y+sine[:,None]*ranges)/resolution).astype(int)
    inside=(xx>=0)&(xx<truth.shape[1])&(yy>=0)&(yy<truth.shape[0])
    xx=xx.clip(0,truth.shape[1]-1);yy=yy.clip(0,truth.shape[0]-1)
    hits=truth[yy,xx]|~inside
    first=np.argmax(hits,axis=1);first[~hits.any(axis=1)]=len(ranges)-1
    visible=(np.arange(len(ranges))[None,:]<=first[:,None])&inside
    observed[yy[visible],xx[visible]]=truth[yy[visible],xx[visible]].astype(np.int8)*100
    beam=np.arange(len(angles))
    return yy[beam,first],xx[beam,first],first,inside[beam,first],hits[beam,first]
