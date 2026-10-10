"""Prepare the DBA-Fusion checkout for building against a modern torch / any GPU.

Why: upstream setup.py (a) hardcodes -gencode up to sm_86 with NO PTX, so the kernels do not
load on Ada (L4, sm_89), Hopper (sm_90) or Blackwell; (b) calls setup() twice in one file
(awkward for wheels); (c) a few torch APIs it uses are deprecated (`Tensor.type()` inside
AT_DISPATCH_*), removed in recent torch.

What it does (idempotent):
  1. writes setup_droid_backends.py and setup_lietorch.py (same sources/flags as upstream,
     arch list = the requested compute capability + PTX),
  2. patches `X.type()` -> `X.scalar_type()` in the AT_DISPATCH macros of src/*.cu,
  3. optionally (--eigen_dir) points the Eigen include to another directory.
Usage: python patch_build.py --root DBA-Fusion --arch 7.5 [--eigen_dir /usr/include/eigen3]
"""
import argparse
import os
import re

TEMPLATE = '''import os.path as osp
from setuptools import setup
from torch.utils.cpp_extension import BuildExtension, CUDAExtension
ROOT = osp.dirname(osp.abspath(__file__))
EIGEN = {eigen!r} or osp.join(ROOT, 'thirdparty/eigen')
GENCODE = {gencode!r}
{body}
'''

DROID = '''setup(
    name='droid_backends', version='0.0.0+dbafusion',
    ext_modules=[CUDAExtension('droid_backends',
        include_dirs=[EIGEN],
        sources=['src/droid.cpp', 'src/droid_kernels.cu', 'src/correlation_kernels.cu', 'src/altcorr_kernel.cu'],
        extra_compile_args={'cxx': ['-O3', '-std=c++17'], 'nvcc': ['-O3', '-std=c++17'] + GENCODE})],
    cmdclass={'build_ext': BuildExtension})'''

LIETORCH = '''setup(
    name='lietorch', version='0.2+dbafusion', description='Lie Groups for PyTorch',
    packages=['lietorch'], package_dir={'': 'thirdparty/lietorch'},
    ext_modules=[CUDAExtension('lietorch_backends',
        include_dirs=[osp.join(ROOT, 'thirdparty/lietorch/lietorch/include'), EIGEN],
        sources=['thirdparty/lietorch/lietorch/src/lietorch.cpp',
                 'thirdparty/lietorch/lietorch/src/lietorch_gpu.cu',
                 'thirdparty/lietorch/lietorch/src/lietorch_cpu.cpp'],
        extra_compile_args={'cxx': ['-O2', '-std=c++17'], 'nvcc': ['-O2', '-std=c++17'] + GENCODE})],
    cmdclass={'build_ext': BuildExtension})'''


def gencode(arch: str):
    sm = arch.replace(".", "")
    return [f"-gencode=arch=compute_{sm},code=sm_{sm}", f"-gencode=arch=compute_{sm},code=compute_{sm}"]


def patch_dispatch(path):
    s = open(path).read()
    n = re.sub(r"(AT_DISPATCH_[A-Z_]+\()(\w+)\.type\(\)", r"\1\2.scalar_type()", s)
    if n != s:
        open(path, "w").write(n)
    return n != s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--arch", required=True, help="compute capability, e.g. 7.5 (T4), 8.9 (L4), 8.0 (A100)")
    ap.add_argument("--eigen_dir", default=None)
    a = ap.parse_args()
    g = gencode(a.arch)
    for name, body in (("setup_droid_backends.py", DROID), ("setup_lietorch.py", LIETORCH)):
        with open(os.path.join(a.root, name), "w") as f:
            f.write(TEMPLATE.format(eigen=a.eigen_dir, gencode=g, body=body))
    changed = [p for p in ("src/correlation_kernels.cu", "src/altcorr_kernel.cu") if patch_dispatch(os.path.join(a.root, p))]
    print("wrote setup_droid_backends.py, setup_lietorch.py (arch %s); patched: %s" % (a.arch, changed or "nothing (already patched)"))


if __name__ == "__main__":
    main()
