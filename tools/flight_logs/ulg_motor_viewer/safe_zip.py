"""Validate all ZIP members before reading/extracting user-supplied sources."""
import stat
from pathlib import Path, PurePosixPath
from zipfile import ZipFile


def inspect_zip(path, max_bytes=8 * 1024 * 1024):
    with ZipFile(path) as archive:
        members = archive.infolist()
        seen = set()
        total = 0
        for member in members:
            name = member.filename
            parts = PurePosixPath(name).parts
            mode = member.external_attr >> 16
            if (not name or "\\" in name or ":" in name or name.startswith("/")
                    or ".." in parts or "\x00" in name
                    or stat.S_ISLNK(mode)
                    or (stat.S_IFMT(mode) and not (stat.S_ISREG(mode) or stat.S_ISDIR(mode)))):
                raise ValueError(f"Unsafe ZIP member: {name!r}")
            normalized = str(PurePosixPath(name))
            if normalized.casefold() in seen:
                raise ValueError(f"Duplicate ZIP member: {name!r}")
            seen.add(normalized.casefold())
            total += member.file_size
            if total > max_bytes:
                raise ValueError("ZIP uncompressed size exceeds limit")
            if member.flag_bits & 1:
                raise ValueError("Encrypted ZIP member")
        return [(m.filename, m.file_size) for m in members]


def extract_sources(path, destination):
    inspect_zip(path)
    destination = Path(destination).resolve()
    with ZipFile(path) as archive:
        for member in archive.infolist():
            target = (destination / member.filename).resolve()
            if destination not in target.parents:
                raise ValueError("ZIP target escapes destination")
            if member.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with target.open("xb") as output:
                    output.write(archive.read(member))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--extract-to", type=Path)
    args = parser.parse_args()
    print(inspect_zip(args.archive))
    if args.extract_to:
        extract_sources(args.archive, args.extract_to)
