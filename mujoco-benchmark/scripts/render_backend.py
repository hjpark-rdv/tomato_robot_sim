"""Scoped GLVND configuration; do not modify the host's graphics installation."""
import ctypes.util
import os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
if os.environ['MUJOCO_GL']=='egl' and ctypes.util.find_library('EGL_nvidia'):
    os.environ.setdefault('__EGL_VENDOR_LIBRARY_FILENAMES',str(Path(__file__).resolve().parents[1]/'config/nvidia_egl.json'))

def info():
    from OpenGL import GL
    return dict(backend=os.environ['MUJOCO_GL'],vendor=GL.glGetString(GL.GL_VENDOR).decode(),renderer=GL.glGetString(GL.GL_RENDERER).decode())
