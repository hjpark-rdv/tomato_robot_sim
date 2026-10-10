"""Pace an idle inspection window without changing physics or render quality."""
import carb
import omni.timeline
import omni.kit.viewport.utility as vu


class ViewerPacing:
    """Restore the original run-loop settings whenever the timeline plays.

    Keeping this separate from the simulation loop prevents a remote-viewing
    frame cap from silently changing the existing physics smoke measurement.
    No USD attributes, renderer quality, resolution or collision settings change.
    """

    PREFIX = '/app/runLoops/main/'
    KEYS = ('rateLimitEnabled', 'rateLimitFrequency', 'rateLimitUseBusyLoop')

    def __init__(self, fps=30):
        if not 0 <= fps <= 240:
            raise ValueError('viewer FPS must be between 0 and 240 (0 keeps original pacing)')
        self.fps = fps
        self.settings = carb.settings.get_settings()
        self.original = {k: self.settings.get(self.PREFIX + k) for k in self.KEYS}
        self.timeline = omni.timeline.get_timeline_interface()
        self.subscription = self.timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._on_timeline_event, name='Harvest idle viewport pacing')
        viewport = vu.get_active_viewport()
        self.settings.set(f'/persistent/app/viewport/{viewport.id}/hud/renderFPS/visible', True)
        self.refresh()

    def _restore(self):
        for key, value in self.original.items():
            path = self.PREFIX + key
            if value is None:
                self.settings.destroy_item(path)
            else:
                self.settings.set(path, value)

    def refresh(self):
        if self.fps and not self.timeline.is_playing():
            self.settings.set(self.PREFIX + 'rateLimitFrequency', self.fps)
            self.settings.set(self.PREFIX + 'rateLimitUseBusyLoop', False)
            self.settings.set(self.PREFIX + 'rateLimitEnabled', True)
        else:
            self._restore()

    def _on_timeline_event(self, event):
        if event.type in (int(omni.timeline.TimelineEventType.PLAY),
                          int(omni.timeline.TimelineEventType.PAUSE),
                          int(omni.timeline.TimelineEventType.STOP)):
            self.refresh()

    def close(self):
        self.subscription = None
        self._restore()
