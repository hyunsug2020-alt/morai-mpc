"""Clear MORAI's sensor-editor pause without changing desktop keyboard focus."""
import json,time
from contextlib import closing
import websocket

def pause_state():
    with closing(websocket.create_connection('ws://127.0.0.1:9091',timeout=3)) as ws:
        ws.send(json.dumps({'op':'call_service','id':'semantic_pause_state','service':'/Service_MoraiEventCmd',
                            'args':{'request':{'option':0}}}))
        while True:
            reply=json.loads(ws.recv())
            if reply.get('id')=='semantic_pause_state':
                if not reply.get('result'):raise RuntimeError('MORAI 일시정지 상태 조회 실패')
                return bool(reply['values']['response']['set_pause'])

def resume_sensor_pause():
    if not pause_state():return
    from Xlib import display,XK,X
    from Xlib.protocol import event
    d=display.Display()
    try:
        root=d.screen().root;p=root.get_full_property(d.intern_atom('_NET_CLIENT_LIST'),X.AnyPropertyType)
        windows=[]
        for wid in p.value if p is not None else []:
            w=d.create_resource_object('window',wid)
            if w.get_wm_class()==('Simulator.x86_64','Simulator.x86_64'):windows.append(w)
        if len(windows)!=1:raise RuntimeError('재개할 MORAI 창을 하나로 식별할 수 없습니다.')
        w=windows[0];key=d.keysym_to_keycode(XK.string_to_keysym('Escape'))
        args=dict(time=X.CurrentTime,root=root,window=w,child=X.NONE,root_x=0,root_y=0,event_x=0,event_y=0,state=0,detail=key,same_screen=1)
        w.send_event(event.KeyPress(**args),event_mask=X.KeyPressMask)
        w.send_event(event.KeyRelease(**args),event_mask=X.KeyReleaseMask);d.sync()
        for _ in range(20):
            time.sleep(.1)
            if not pause_state():return
        # A freshly started Unity window ignores synthetic events unless focused: focus it and send a real key via XTEST.
        from Xlib.ext import xtest
        w.raise_window();w.set_input_focus(X.RevertToParent,X.CurrentTime);d.sync();time.sleep(.3)
        xtest.fake_input(d,X.KeyPress,key);xtest.fake_input(d,X.KeyRelease,key);d.sync()
    finally:d.close()
    for _ in range(20):
        time.sleep(.1)
        if not pause_state():return
    raise RuntimeError('MORAI 센서 로드 후 일시정지 해제를 확인하지 못했습니다.')
