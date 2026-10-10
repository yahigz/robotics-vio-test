"""Pure-PyTorch replacement for the two torch_scatter functions DBA-Fusion uses
(scatter_sum in dbaf/geom/ba.py, scatter_mean in dbaf/droid_net.py).
Avoids building torch_scatter against Colab's torch. Semantics follow torch_scatter
for a 1-D `index` broadcast along `dim`."""
import torch


def _broadcast(index, src, dim):
    if dim < 0:
        dim = src.dim() + dim
    if index.dim() == 1:
        shape = [1] * src.dim()
        shape[dim] = -1
        index = index.view(shape)
    return index.expand_as(src), dim


def scatter_sum(src, index, dim=-1, out=None, dim_size=None):
    index, dim = _broadcast(index, src, dim)
    if out is None:
        size = list(src.shape)
        if dim_size is not None:
            size[dim] = int(dim_size)
        elif index.numel() == 0:
            size[dim] = 0
        else:
            size[dim] = int(index.max()) + 1
        out = torch.zeros(size, dtype=src.dtype, device=src.device)
    return out.scatter_add_(dim, index, src)


def scatter_add(src, index, dim=-1, out=None, dim_size=None):
    return scatter_sum(src, index, dim, out, dim_size)


def scatter_mean(src, index, dim=-1, out=None, dim_size=None):
    s = scatter_sum(src, index, dim, out, dim_size)
    ones = torch.ones_like(src)
    cnt = scatter_sum(ones, index, dim, None, s.shape[dim if dim >= 0 else src.dim() + dim])
    return s / cnt.clamp(min=1)
