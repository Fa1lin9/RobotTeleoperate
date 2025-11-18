from pathlib import Path

print(" current file path is ", Path(__file__).resolve())
print(" current file path is ", Path(__file__).resolve().parent)
print(" current file path is ", Path(__file__).resolve().parent.parent)
print(" current file path is ", Path(__file__).resolve().parent.parent.parent)
