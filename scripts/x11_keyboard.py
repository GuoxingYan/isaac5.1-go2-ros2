"""Real X11 keyboard delivery to this process' own Isaac window."""
import ctypes as C
import os

class X11Keyboard:
    def __init__(self):
        self.x=C.CDLL('libX11.so.6')
        self.xt=C.CDLL('libXtst.so.6')
        self.x.XOpenDisplay.argtypes=[C.c_char_p];self.x.XOpenDisplay.restype=C.c_void_p
        self.d=self.x.XOpenDisplay(None)
        if not self.d: raise RuntimeError('No X11 display')
        self.x.XDefaultRootWindow.argtypes=[C.c_void_p];self.x.XDefaultRootWindow.restype=C.c_ulong
        self.x.XQueryTree.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_ulong),C.POINTER(C.POINTER(C.c_ulong)),C.POINTER(C.c_uint)]
        self.x.XFetchName.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_char_p)]
        self.x.XInternAtom.argtypes=[C.c_void_p,C.c_char_p,C.c_int];self.x.XInternAtom.restype=C.c_ulong
        self.x.XGetWindowProperty.argtypes=[C.c_void_p,C.c_ulong,C.c_ulong,C.c_long,C.c_long,C.c_int,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_int),C.POINTER(C.c_ulong),C.POINTER(C.c_ulong),C.POINTER(C.POINTER(C.c_ubyte))]
        self.x.XFree.argtypes=[C.c_void_p]
        self.x.XSetInputFocus.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_ulong]
        self.x.XRaiseWindow.argtypes=[C.c_void_p,C.c_ulong]
        self.x.XFlush.argtypes=[C.c_void_p]
        self.x.XStringToKeysym.argtypes=[C.c_char_p];self.x.XStringToKeysym.restype=C.c_ulong
        self.x.XKeysymToKeycode.argtypes=[C.c_void_p,C.c_ulong];self.x.XKeysymToKeycode.restype=C.c_uint
        self.xt.XTestFakeKeyEvent.argtypes=[C.c_void_p,C.c_uint,C.c_int,C.c_ulong]
        root=self.x.XDefaultRootWindow(self.d)
        self.window=self.find(root)
        if not self.window: raise RuntimeError('Isaac window owned by process not found')
        self.x.XRaiseWindow(self.d,self.window)
        self.x.XSetInputFocus(self.d,self.window,1,0)
        self.x.XFlush(self.d)
        for key in ["W","A","S","D","Z","C"]:
            self.event(key,False)

    def find(self,window):
        atom=self.x.XInternAtom(self.d,b'_NET_WM_PID',False)
        typ=C.c_ulong();fmt=C.c_int();n=C.c_ulong();remaining=C.c_ulong();data=C.POINTER(C.c_ubyte)()
        self.x.XGetWindowProperty(self.d,window,atom,0,1,False,6,C.byref(typ),C.byref(fmt),C.byref(n),C.byref(remaining),C.byref(data))
        if n.value and data:
            pid=C.cast(data,C.POINTER(C.c_ulong))[0]
            self.x.XFree(data)
            if pid==os.getpid(): return window
        root=C.c_ulong();parent=C.c_ulong();children=C.POINTER(C.c_ulong)();count=C.c_uint()
        self.x.XQueryTree(self.d,window,C.byref(root),C.byref(parent),C.byref(children),C.byref(count))
        found=None
        for i in range(count.value):
            found=self.find(children[i])
            if found: break
        if children: self.x.XFree(children)
        return found

    def event(self,key,press):
        self.x.XSetInputFocus(self.d,self.window,1,0)
        self.x.XFlush(self.d)
        symbol='space' if key=='SPACE' else key.lower()
        code=self.x.XKeysymToKeycode(self.d,self.x.XStringToKeysym(symbol.encode()))
        if not self.xt.XTestFakeKeyEvent(self.d,code,int(press),0): raise RuntimeError('XTest key delivery failed')
        self.x.XFlush(self.d)
