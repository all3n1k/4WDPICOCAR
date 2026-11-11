from pathlib import Path
import importlib
import io
import sys
import types
from email.message import Message
from unittest.mock import patch
import pytest

@pytest.fixture
def dashboard(monkeypatch,tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    monkeypatch.setitem(sys.modules,'cv2',types.ModuleType('cv2'))
    m=importlib.import_module('dimos_lite.dashboard')
    m.state.manual_cmds=[]
    return m

def test_bind_failure_does_not_kill_an_existing_process(dashboard):
    with patch.object(dashboard.socketserver,'ThreadingTCPServer',side_effect=OSError('busy')),patch('os.kill') as kill:
        with pytest.raises(OSError):dashboard.start_dashboard()
        kill.assert_not_called()

@pytest.mark.parametrize('headers,body,status',[
    ({'Origin':'https://attacker.invalid'},b'{"command":"forward"}',403),
    ({},b'[]',400),
    ({},b'{"command":"unexpected"}',400),
    ({},b'{"command":"forward"}',200),
])
def test_control_validation_precedes_motion_queue(dashboard,headers,body,status):
    handler=object.__new__(dashboard.DashboardHandler)
    handler.path='/api/control';handler.headers=Message()
    for k,v in {'Host':'localhost:8080','Content-Type':'application/json','Content-Length':str(len(body)),**headers}.items():handler.headers[k]=v
    handler.rfile=io.BytesIO(body);handler.wfile=io.BytesIO();handler.connection=types.SimpleNamespace(settimeout=lambda _:None)
    statuses=[];handler.send_response=statuses.append;handler.send_error=lambda code,*_:statuses.append(code);handler.send_header=lambda *_:None;handler.end_headers=lambda:None
    handler.do_POST()
    assert statuses[0]==status
    assert dashboard.state.manual_cmds==(['forward']if status==200 else [])
