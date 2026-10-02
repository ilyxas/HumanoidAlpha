"""Create the single current Mac/Linux handoff from this working project."""
import hashlib
from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT.parent / 'HumanoidAlpha_Delivery'
SKIP_DIRS = {'.venv', '.runtime', '__pycache__', '.git', 'node_modules', 'shots'}
REPORTS_NEEDED = {'reports/control_inventory.json', 'reports/collision/model_before.xml'}


def main():
    DEST.mkdir(exist_ok=True)
    archive = DEST / 'HumanoidAlpha.zip'
    temporary = DEST / 'HumanoidAlpha.zip.tmp'
    files = []
    for path in sorted(ROOT.rglob('*')):
        relative = path.relative_to(ROOT)
        if any(part in SKIP_DIRS for part in relative.parts):
            continue
        if not path.is_file() or path.is_symlink():
            continue
        if path.name == '.DS_Store' or path.suffix in {'.pyc', '.log', '.jsonl'}:
            continue
        if relative.parts[0] == 'reports' and relative.as_posix() not in REPORTS_NEEDED:
            continue
        files.append((path, relative))
    with zipfile.ZipFile(temporary, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as output:
        for path, relative in files:
            output.write(path, 'HumanoidAlpha/' + relative.as_posix())
    with zipfile.ZipFile(temporary) as check:
        bad = check.testzip()
        if bad:
            raise RuntimeError(f'Archive verification failed: {bad}')
        for required in ['START.command', 'INSTALL_RU.md', 'scripts/launch.py',
                         'physics/collision_guard.py', 'viewer/observation.html',
                         'viewer/observation.js', 'viewer/console.html']:
            check.getinfo('HumanoidAlpha/' + required)
        assert len([n for n in check.namelist() if n.startswith('HumanoidAlpha/assets/collision/')]) == 27
    temporary.replace(archive)
    shutil.copyfile(ROOT / 'INSTALL_RU.md', DEST / 'INSTALL_RU.md')
    with archive.open('rb') as source:
        digest = hashlib.file_digest(source, 'sha256').hexdigest()
    (DEST / 'SHA256SUMS.txt').write_text(f'{digest}  HumanoidAlpha.zip\n')
    (DEST / 'README.txt').write_text('Актуальная поставка Humanoid Alpha.\n'
                                   'Передай HumanoidAlpha.zip и INSTALL_RU.md.\n'
                                   'Исходная рабочая папка: ' + str(ROOT) + '\n')
    print(f'{archive}\n{len(files)} files; {archive.stat().st_size / 1024**2:.1f} MiB\nSHA-256: {digest}')


if __name__ == '__main__':
    main()
