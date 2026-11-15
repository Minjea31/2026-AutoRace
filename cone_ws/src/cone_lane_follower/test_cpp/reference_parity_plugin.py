"""Compare native kernels on every call made by the original regression suite."""
import functools
import inspect
import cone_native as native
from cone_lane_follower.node import RoiNode, BOUNDARY_DEFAULTS
from test_parity import close


def pytest_sessionstart(session):
    names=('sample_boundary','segments_cross','select_boundary_chain','select_boundaries',
           'boundary_width_matches','match_cross_sections','match_projected_sections',
           'smooth_center_path','offset_boundary','point_near_boundary','quintic_bridge',
           'offset_safe','valid_center_geometry','unified_path','path_control_geometry',
           'path_errors','path_stop_reason','path_length')
    for name in names:
        original=getattr(RoiNode,name)
        signature=inspect.signature(original)
        target=getattr(native,name)
        def make(original,signature,target,name):
            @functools.wraps(original)
            def checked(*args,**kwargs):
                result=original(*args,**kwargs)
                arguments=signature.bind(*args,**kwargs)
                arguments.apply_defaults()
                call=dict(arguments.arguments)
                call.pop('diagnostics',None)
                if name=='select_boundary_chain':call['parameters']=dict(BOUNDARY_DEFAULTS,**(call['parameters'] or {}))
                # Native bindings use positional arguments for pure kernels.
                other=target(*call.values())
                close(result,other)
                return result
            return checked
        setattr(RoiNode,name,staticmethod(make(original,signature,target,name)))
    from cone_lane_follower.path_memory import PathMemory
    init=PathMemory.__init__
    def initialize(self):
        init(self)
        self._native_memory=native.PathMemory()
    PathMemory.__init__=initialize
    for name in ('clear','store','refresh','replace_from_vehicle','remaining'):
        original=getattr(PathMemory,name)
        def make_memory(original,name):
            @functools.wraps(original)
            def checked(self,*args,**kwargs):
                if not hasattr(self,'_native_memory'):
                    return original(self,*args,**kwargs)
                twin=self._native_memory
                for key in ('points','index','finished','update_reason'):setattr(twin,key,getattr(self,key))
                result=original(self,*args,**kwargs)
                other=getattr(twin,name)(*args,**kwargs)
                close(result,other)
                for key in ('points','index','finished','update_reason'):close(getattr(self,key),getattr(twin,key))
                return result
            return checked
        setattr(PathMemory,name,make_memory(original,name))
