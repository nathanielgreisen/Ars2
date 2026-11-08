"""Run deterministic firmware reset/failure tests without a connected robot."""
from pathlib import Path
import subprocess
import tempfile

here = Path(__file__).resolve().parent
root = here.parents[1]
with tempfile.TemporaryDirectory(prefix="ars2-reset-test-") as directory:
    binary = Path(directory) / "reset_test"
    subprocess.run([
        "clang++", "-std=c++17", "-DARS2_DEBUG=0", "-I", str(here / "mocks"),
        "-I", str(root / "include"), str(here / "reset_test.cpp"),
        str(root / "src/pinpoint.cpp"), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
