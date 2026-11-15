"""Desktop entry point with a local error log, including pythonw launches."""
import datetime
import os
from pathlib import Path
import sys
import traceback

if __name__ == '__main__':
    os.environ.setdefault('QT_OPENGL','software')
    folder=Path(__file__).resolve().parents[1]/'data'/'logs';folder.mkdir(parents=True,exist_ok=True)
    log=(folder/f"lab-{datetime.datetime.now():%Y%m%d-%H%M%S}.log").open('a',encoding='utf-8',buffering=1)
    sys.stdout=log;sys.stderr=log
    def error_hook(kind,value,tb):traceback.print_exception(kind,value,tb,file=log)
    sys.excepthook=error_hook
    try:
        from furuta_lab.app import main
        main()
    except Exception:
        traceback.print_exc(file=log)
        raise
