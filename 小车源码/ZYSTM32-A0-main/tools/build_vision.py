"""Rebuild Keil ARMCC5 firmware without the uVision GUI.
python tools/build_vision.py [path-to-ARMCC-bin]
"""
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
cwd = root/'USER'
compiler = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('D:/keil5/ARM/ARMCC/bin')
out = root/'OBJ'
out.mkdir(exist_ok=True)
tree = ET.parse(cwd/'Template.uvprojx')
includes = tree.find('.//Cads/VariousControls/IncludePath').text.split(';')
defines = tree.find('.//Cads/VariousControls/Define').text.split(',')
base = [str(compiler/'armcc.exe'), '--cpu', 'Cortex-M3', '--thumb', '-g', '-O0', '--apcs=interwork']
for define in defines:
    base.append('-D'+define)
for path in includes:
    base += ['-I', path]
objects = []
log = []


def run(command):
    result = subprocess.run(command, cwd=cwd, capture_output=True)
    text = (result.stdout+result.stderr).decode('gbk', errors='replace')
    log.append(text)
    if text:
        print(text)
    (root/'vision_build.log').write_text('\n'.join(log), encoding='utf-8')
    result.check_returncode()


for entry in tree.findall('.//Groups/Group/Files/File'):
    path = entry.findtext('FilePath')
    if not path or not path.endswith(('.c', '.s')):
        continue
    obj = '../OBJ/'+Path(path.replace('\\', '/')).stem+'.o'
    if path.endswith('.c'):
        run(base+['-c', path, '-o', obj])
    else:
        run([str(compiler/'armasm.exe'), '--cpu', 'Cortex-M3', '--thumb', '-g', path, '-o', obj])
    objects.append(obj)

run([str(compiler/'armlink.exe'), '--cpu', 'Cortex-M3',
     '--scatter', '../tools/vision.sct', '--entry', 'Reset_Handler', '--strict',
     '--map', '--symbols', '--info', 'sizes,totals', '--list', '../OBJ/vision.map',
     '--output', '../OBJ/vision.axf']+objects)
run([str(compiler/'fromelf.exe'), '--i32combined', '--output', '../OBJ/vision.hex', '../OBJ/vision.axf'])
print('PASS: full ARMCC build/link; OBJ/vision.hex (Flash configuration page reserved)')
