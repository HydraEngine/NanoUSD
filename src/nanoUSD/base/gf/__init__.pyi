

class GfBBox3d:
    def __init__(self) -> None: ...

    def Set(self, arg0: "pxrInternal_v0_24_11__pxrReserved__::GfRange3d", arg1: "pxrInternal_v0_24_11__pxrReserved__::GfMatrix4d", /) -> None: ...

    def SetMatrix(self, arg: "pxrInternal_v0_24_11__pxrReserved__::GfMatrix4d", /) -> None: ...

    def SetRange(self, arg: "pxrInternal_v0_24_11__pxrReserved__::GfRange3d", /) -> None: ...

    def GetRange(self) -> "pxrInternal_v0_24_11__pxrReserved__::GfRange3d": ...

    def GetBox(self) -> "pxrInternal_v0_24_11__pxrReserved__::GfRange3d": ...