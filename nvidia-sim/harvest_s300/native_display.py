"""Bounded read-only X protocol validation before an existing native Kit job.

A forwarding socket can exist and accept connects while its upstream display
has vanished.  Only a successful xdpyinfo protocol reply is used here.  This
does not create displays, inspect/reset GPUs or certify Vulkan initialization.
"""
import copy
import subprocess


class NativeDisplayError(RuntimeError):
    def __init__(self,message,proof):
        super().__init__(message);self.proof=copy.deepcopy(proof)


def validated_native_display_environment(environment,*,probe_runner=None,timeout_s=2.):
    """Return copied env/proof; an explicit override is never substituted."""
    if isinstance(timeout_s,bool) or not isinstance(timeout_s,(float,int)) or not 0<timeout_s<=2.:
        raise ValueError('Display protocol probe must be bounded within2seconds')
    env=dict(environment);run=subprocess.run if probe_runner is None else probe_runner
    inherited=env.get('DISPLAY');explicit='S300_NATIVE_DISPLAY' in env
    requested=env.get('S300_NATIVE_DISPLAY') if explicit else inherited
    proof=dict(schema='s300-native-display-precheck-v1',inherited_display=inherited,
        explicit_override=explicit,requested_display=requested,selected_display=None,
        probe_timeout_s=float(timeout_s),attempts=[],xserver_created=False,gpu_reset=False,
        physics_settings_changed=False,vulkan_device_creation_verified=False)
    if explicit and (not isinstance(requested,str) or not requested.strip()):
        proof['status']='invalid_explicit_override'
        raise NativeDisplayError('S300_NATIVE_DISPLAY must name a working existing X display',proof)
    candidates=[requested] if explicit else [value for value in (inherited,':0')
        if isinstance(value,str) and value.strip()]
    candidates=list(dict.fromkeys(candidates))
    for display in candidates:
        query_env=dict(env,DISPLAY=display)
        attempt=dict(display=display,protocol_query='xdpyinfo')
        try:
            result=run(['xdpyinfo','-display',display],env=query_env,capture_output=True,
                text=True,timeout=float(timeout_s),check=False)
            valid=(result.returncode==0 and isinstance(result.stdout,str) and
                any(line.lstrip().startswith('name of display:') for line in result.stdout.splitlines()))
            attempt.update(returncode=result.returncode,status='valid_x_protocol_reply' if valid else 'display_query_failed',
                stderr_excerpt=str(result.stderr or '')[:300])
        except subprocess.TimeoutExpired:
            valid=False;attempt['status']='display_query_timeout'
        except OSError as error:
            valid=False;attempt.update(status='display_query_unavailable',error=type(error).__name__+': '+str(error))
        proof['attempts'].append(attempt)
        if valid:
            env['DISPLAY']=display
            proof.update(status='validated',selected_display=display,
                selection='explicit_override' if explicit else
                    'inherited_display' if display==inherited else 'existing_display0_fallback',
                inherited_display_replaced=display!=inherited)
            return env,proof
    proof['status']='no_valid_existing_display'
    message=('Explicit S300_NATIVE_DISPLAY failed its bounded X protocol check; no fallback was applied'
        if explicit else 'No working inherited DISPLAY or existing :0; native Kit job was not launched')
    raise NativeDisplayError(message,proof)
