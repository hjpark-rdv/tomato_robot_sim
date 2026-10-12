"""Official MuJoCo Filament rendering for saved-state replay.

This module never advances physics. Studio registers MuJoCo's packaged
Filament material provider, then the official rendering.filament API renders
the scene. Configuring the same scene twice works around 3.15's color-grading
setter using the previous options on its first call. The returned RGB also
feeds the GUI, so offscreen and interactive views share the same renderer.
"""
from __future__ import annotations

import gc
import os
import time

import numpy as np

_RESOURCE_PROVIDER_REGISTERED = False


def configure_graphics_environment():
    """Select the existing NVIDIA X server in this render process only."""
    os.environ['DISPLAY'] = os.environ.get('S300_MUJOCO_RENDER_DISPLAY', ':0')
    os.environ.setdefault('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')


class FilamentRenderer:
    """Renderer-compatible adapter around MuJoCo's official Filament backend."""

    def __init__(self, model, height=1080, width=1920, quality='ultra'):
        if quality not in ('balanced', 'ultra'):
            raise ValueError('Filament quality must be balanced or ultra')
        if width <= 0 or height <= 0:
            raise ValueError('Render dimensions must be positive')
        # MuJoCo 3.15 CreateSkinMesh allocates its index buffer with the
        # vertex count, then supplies 3*face_count indices. Reject old render
        # caches in Python rather than allowing the native renderer to exit.
        if model.nskin and np.any(model.skin_vertnum != 3 * model.skin_facenum):
            raise ValueError('MuJoCo 3.15 Filament indexed-skin compatibility '
                             'requires vertex count = 3 * face count; '
                             'rebuild the render_assets visual cache')
        configure_graphics_environment()
        import mujoco as mj
        import mujoco._render_filament as mjrf
        from mujoco.experimental.studio.renderer import Renderer as StudioRenderer
        from mujoco.rendering.filament.renderer import Renderer
        self._mj = mj
        self._mjrf = mjrf
        self.model = model
        self.width = int(width)
        self.height = int(height)
        self.quality = quality
        self._data = None
        self._camera = None
        self._option = mj.MjvOption()
        start = time.perf_counter()
        global _RESOURCE_PROVIDER_REGISTERED
        if not _RESOURCE_PROVIDER_REGISTERED:
            registrar = StudioRenderer('opengl_headless')
            del registrar
            _RESOURCE_PROVIDER_REGISTERED = True
        self._context = mjrf.Context(mjrf.ContextConfig(
            graphics_api=mjrf.GraphicsApi.GRAPHICS_API_OPENGL))
        self._renderer = Renderer(self._context)
        self._objects = mjrf.ModelObjects(self._context, model)
        self._scene = self._renderer.scene('scene')
        self._scene.configure_from_model(model)
        self._scene.configure_from_model(model)
        self._lights = mjrf.ModelLights(self._scene, self._objects)
        self._renderables = mjrf.ModelRenderables(self._scene, self._objects)
        self._pixels = np.empty((self.height, self.width, 3), dtype=np.uint8)
        self._renderer.target('image', self._pixels)
        self._renderer.view('view', scene='scene', target='image')
        self.initialization_s = time.perf_counter() - start
        self.last_render_s = None
        self.backend = 'filament_opengl'
        self.color_grading_configure_calls = 2

    def update_scene(self, data, camera=None, scene_option=None):
        if self._renderer is None:
            raise RuntimeError('Renderer is closed')
        self._data = data
        self._camera = camera
        if scene_option is not None:
            self._option = scene_option

    def render(self):
        if self._renderer is None:
            raise RuntimeError('Renderer is closed')
        if self._data is None:
            raise RuntimeError('update_scene must precede render')
        start = time.perf_counter()
        self._lights.update(self._data)
        self._renderables.set_options(self._option)
        self._renderables.update(self._data)
        camera = self._camera
        if camera is None:
            camera = self._mj.MjvCamera()
            self._mj.mjv_defaultFreeCamera(self.model, camera)
        gl_camera = self._mj.mjv_camera2GLCamera(self.model, self._data, camera)
        fields = ('pos', 'forward', 'up', 'frustum_center', 'frustum_width',
                  'frustum_bottom', 'frustum_top', 'frustum_near', 'frustum_far',
                  'orthographic')
        self._renderer.update_camera('view', self._mjrf.Camera(
            **{name:getattr(gl_camera, name) for name in fields}))
        self._renderer.render()
        self.last_render_s = time.perf_counter() - start
        return self._pixels

    def resize(self, width, height):
        if width <= 0 or height <= 0:
            raise ValueError('Render dimensions must be positive')
        width, height = int(width), int(height)
        if (width, height) != (self.width, self.height):
            self.width, self.height = width, height
            self._pixels = np.empty((height, width, 3), dtype=np.uint8)
            self._renderer.target('image', self._pixels)

    def close(self):
        self._renderables = None
        self._lights = None
        self._objects = None
        self._scene = None
        self._renderer = None
        self._context = None
        self._pixels = None
        self._data = None
        gc.collect()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def orbit_camera(camera, dx, dy):
    """Mouse orbit only changes the replay camera, never model/data state."""
    camera.azimuth = float(camera.azimuth + dx * .25)
    camera.elevation = float(np.clip(camera.elevation + dy * .25, -89, 89))


def pan_camera(camera, dx, dy, window_height):
    yaw = np.deg2rad(camera.azimuth)
    horizontal = np.array([-np.sin(yaw), np.cos(yaw), 0.])
    scale = 1.3 * camera.distance / max(window_height, 1)
    camera.lookat[:] += horizontal * dx * scale + np.array([0., 0., dy * scale])


def zoom_camera(camera, wheel):
    camera.distance = float(np.clip(camera.distance * np.exp(-.12 * wheel), .01, 1000))


class FilamentWindow:
    """GLFW image presentation with orbit/pan/zoom and pause controls.

    Filament still renders every pixel.  GLFW only displays its completed RGB
    buffer.  Left drag orbits, right drag pans, wheel zooms, Space pauses,
    Escape closes.  The caller retains ownership of replay time and states.
    """

    def __init__(self, camera, width=1920, height=1080,
                 title='Farmily MuJoCo | Filament | saved-state replay'):
        configure_graphics_environment()
        import glfw
        from OpenGL import GL
        self._glfw, self._gl = glfw, GL
        if not glfw.init():
            raise RuntimeError('Cannot initialize GLFW on ' + os.environ['DISPLAY'])
        self._window = glfw.create_window(width, height, title, None, None)
        if not self._window:
            glfw.terminate()
            raise RuntimeError('Cannot create the Filament replay presentation window')
        glfw.make_context_current(self._window)
        glfw.swap_interval(0)
        self._texture = GL.glGenTextures(1)
        self._camera = camera
        self._last_cursor = None
        self.paused = False
        glfw.set_cursor_pos_callback(self._window, self._cursor)
        glfw.set_scroll_callback(self._window, self._scroll)
        glfw.set_key_callback(self._window, self._key)

    @property
    def size(self):
        return self._glfw.get_framebuffer_size(self._window)

    def _cursor(self, window, x, y):
        glfw = self._glfw
        old = self._last_cursor
        self._last_cursor = (x, y)
        if old is None:
            return
        dx, dy = x - old[0], y - old[1]
        # Fixed cameras become free before a user moves the view.
        if (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS or
                glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS):
            self._camera.type = 0  # mjCAMERA_FREE
        if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
            orbit_camera(self._camera, dx, dy)
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS:
            pan_camera(self._camera, dx, dy, self.size[1])

    def _scroll(self, window, xoffset, yoffset):
        self._camera.type = 0  # mjCAMERA_FREE
        zoom_camera(self._camera, yoffset)

    def _key(self, window, key, scancode, action, mods):
        glfw = self._glfw
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)
        elif key == glfw.KEY_SPACE:
            self.paused = not self.paused

    def is_running(self):
        return self._window is not None and not self._glfw.window_should_close(self._window)

    def poll_events(self):
        self._glfw.poll_events()

    def present(self, rgb):
        glfw, gl = self._glfw, self._gl
        glfw.make_context_current(self._window)
        width, height = self.size
        gl.glViewport(0, 0, width, height)
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.glOrtho(0, 1, 0, 1, -1, 1)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        gl.glEnable(gl.GL_TEXTURE_2D)
        gl.glBindTexture(gl.GL_TEXTURE_2D, self._texture)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
        gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)
        rgb = np.ascontiguousarray(rgb)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, rgb.shape[1], rgb.shape[0],
                        0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, rgb)
        gl.glColor3f(1., 1., 1.)
        gl.glBegin(gl.GL_QUADS)
        for uv, xy in (((0., 1.), (0., 0.)), ((1., 1.), (1., 0.)),
                       ((1., 0.), (1., 1.)), ((0., 0.), (0., 1.))):
            gl.glTexCoord2f(*uv)
            gl.glVertex2f(*xy)
        gl.glEnd()
        glfw.swap_buffers(self._window)

    def close(self):
        if self._window is not None:
            self._glfw.make_context_current(self._window)
            self._gl.glDeleteTextures([self._texture])
            self._glfw.destroy_window(self._window)
            self._window = None
            self._glfw.terminate()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
