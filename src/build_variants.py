"""Build standalone TA submission files from one maintained source."""
from pathlib import Path

root = Path(__file__).resolve().parents[1]
source = (root/'src/controller.py').read_text(encoding='utf-8')
for name, scale in [('unscaled',1.0),('scaled_2p8',2.8)]:
    target = root/'controllers'/name/'student_controller2.py'
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(source.replace('GAMMA = 1.0',f'GAMMA = {scale}')
                      .replace('BETA = 1.0',f'BETA = {scale}'),encoding='utf-8')
    print(target)
