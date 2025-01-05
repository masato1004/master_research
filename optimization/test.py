import numpy as np
import torch


def test(a,b):
    print(torch.cuda.is_available())
    return a + b**2
    