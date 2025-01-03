import numpy as np


def test(a,b):
    import torch
    print(torch.cuda.is_available())
    return a + b**2
    